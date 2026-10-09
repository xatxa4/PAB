/*
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * Original to this project.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef USB_DAC_PCM_H
#define USB_DAC_PCM_H

/*
 * Pure helpers for the USB sound card's data path: unpacking a packet of PCM
 * into the 32 bit frames audio_out wants, the index arithmetic of the ring
 * between the USB interrupt and the main loop, the volume to gain table and the
 * feedback filter and controller.
 * They include nothing from the SDK so that a host compiler can check them against hand-computed values.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Interleaved stereo frames from one USB packet, left aligned in 32 bits.
/// subframe_bytes is 2 (16 bit) or 3 (24 bit); anything else writes nothing.
/// A trailing partial frame is ignored: a packet cut short by the host or the
/// bus must not leave half a frame (and so a swapped channel) in the stream.
/// Returns the frames written, 2 * that many int32_t to dst.
static inline size_t usb_dac_unpack(const uint8_t * src, size_t len, uint32_t subframe_bytes, int32_t * dst){
    if (subframe_bytes != 2 && subframe_bytes != 3) return 0;
    size_t frames = len / (2 * subframe_bytes);
    // shift as unsigned: a signed shift of a negative value is undefined
    if (subframe_bytes == 2){
        for (size_t i = 0; i < frames * 2; i++){
            uint16_t s = (uint16_t) ((uint16_t) src[2 * i] | (uint16_t) ((uint16_t) src[2 * i + 1] << 8));
            dst[i] = (int32_t) ((uint32_t) s << 16);
        }
    } else {
        for (size_t i = 0; i < frames * 2; i++){
            const uint8_t * p = src + 3 * i;
            dst[i] = (int32_t) ((uint32_t) p[0] << 8 | (uint32_t) p[1] << 16 | (uint32_t) p[2] << 24);
        }
    }
    return frames;
}

// Single-producer single-consumer ring of frames. The indices count frames
// forever and are never reduced modulo the capacity: they wrap at 2^32, which
// a power-of-two capacity divides, so write - read stays the fill level (in
// unsigned arithmetic) and index & (capacity - 1) the slot. That removes the
// full-versus-empty ambiguity of indices that are kept below the capacity.

static inline uint32_t usb_dac_ring_level(uint32_t write, uint32_t read){
    return write - read;
}

static inline uint32_t usb_dac_ring_free(uint32_t write, uint32_t read, uint32_t capacity){
    return capacity - (write - read);
}

static inline uint32_t usb_dac_ring_slot(uint32_t index, uint32_t capacity){
    return index & (capacity - 1u);
}

// ---- volume -----------------------------------------------------------------

#define USB_DAC_GAIN_UNITY     65536    // audio_out's AUDIO_OUT_UNITY_GAIN
#define USB_DAC_VOLUME_MIN_DB  (-90)

// 65536 * 10^(-dB / 20) rounded to nearest, indexed by -dB (0 .. 90). Whole
// dB only: the descriptors advertise a 1 dB resolution, so that is all a host
// can ask for. Generated with a script and checked against pow() on the host.
static const uint32_t usb_dac_gain_table[91] = {
    65536, 58409, 52057, 46396, 41350, 36854, 32846, 29274,
    26090, 23253, 20724, 18471, 16462, 14672, 13076, 11654,
    10387, 9257, 8250, 7353, 6554, 5841, 5206, 4640,
    4135, 3685, 3285, 2927, 2609, 2325, 2072, 1847,
    1646, 1467, 1308, 1165, 1039, 926, 825, 735,
    655, 584, 521, 464, 414, 369, 328, 293,
    261, 233, 207, 185, 165, 147, 131, 117,
    104, 93, 83, 74, 66, 58, 52, 46,
    41, 37, 33, 29, 26, 23, 21, 18,
    16, 15, 13, 12, 10, 9, 8, 7,
    7, 6, 5, 5, 4, 4, 3, 3,
    3, 2, 2
};

/// Gain for audio_out_set_volume() from the signed 8.8 dB value the host sets.
/// The value is rounded to the nearest whole dB (halves away from zero, so
/// -0.5 dB is -1 dB) and clamped to -90 .. 0: anything above 0 dB is unity,
/// anything below -90 dB, 0x8000 ("-infinity") included, is the -90 dB gain.
/// Muting is the caller's job.
static inline uint32_t usb_dac_volume_gain(int16_t volume_8_8){
    int v = volume_8_8;
    int db = v >= 0 ? (v + 128) / 256 : -((-v + 128) / 256);
    if (db > 0) db = 0;
    if (db < USB_DAC_VOLUME_MIN_DB) db = USB_DAC_VOLUME_MIN_DB;
    return usb_dac_gain_table[-db];
}

// ---- feedback ---------------------------------------------------------------
//
// The host sends, every 1 ms, fb / 16384 frames (10.14 fixed point, the
// fraction carried over), using the last value it read from the feedback
// endpoint. Our DAC eats frames at its own clock, so the queue (ring plus what
// audio_out holds) walks away from where it started unless the value is
// steered. The queue is measured, low-pass filtered, and the value is
// nominal plus a correction proportional to the error. All integers.
//
// Loop dynamics. With e the queue error in frames and d the drift in frames
// per second, a correction of Kp units (10.14) per frame of error changes the
// host's rate by Kp / 16384 frames per ms per frame of error, so
//     de/dt = -(Kp * 1000 / 16384) * e - d
// a first order loop with tau = 16384 / (1000 * Kp) seconds and, for a
// constant drift, a steady state error e = -d / (Kp * 1000 / 16384). Kp = 8
// gives tau = 2.05 s, so the loop is slow next to the filter below (about
// 80 ms) and next to the 8 ms the host takes to read the value and the 1 to 2
// ms it takes to act on it: no sustained oscillation is possible.
//
// What a proportional loop leaves behind: it holds the queue e frames off its
// setpoint to produce a correction of d. For 300 ppm that is e = 59 frames at
// 96 kHz (472 units / 8) and 27 at 44.1 kHz, against start margins of 384 and
// 176 frames. Under a sixth of the margin, so there is no integral term.

#define USB_DAC_FB_KP            8          // 10.14 units per frame of error
#define USB_DAC_FB_FILTER_SHIFT  4          // alpha = 1/16 per ~5 ms pass
#define USB_DAC_FB_CLAMP_DIV     128        // nominal +/- nominal / 128, as upstream

struct usb_dac_fb {
    bool    active;         // false: send the nominal value
    int32_t setpoint;       // frames
    int32_t filtered_q8;    // queue, low-passed, frames * 256
};

static inline void usb_dac_fb_reset(struct usb_dac_fb * f){
    f->active = false;
    f->setpoint = 0;
    f->filtered_q8 = 0;
}

/// Takes the queue measured now as the level to hold, and starts steering from
/// the measurement itself so the first pass is no step. The start gate has
/// already waited for the main loop's phase allowance, so the level is safe.
static inline void usb_dac_fb_start(struct usb_dac_fb * f, uint32_t total_frames){
    f->setpoint = (int32_t) total_frames;
    f->filtered_q8 = (int32_t) (total_frames * 256u);   // no step on the first pass
    f->active = true;
}

/// Feeds one measurement of the queue in frames, returns the 10.14 feedback
/// value to send. The queue is a few thousand frames at most, so Q8 and the
/// product below stay far inside 32 and 64 bits at 96 kHz.
static inline uint32_t usb_dac_fb_update(struct usb_dac_fb * f, uint32_t total_frames, uint32_t nominal){
    if (!f->active) return nominal;
    int32_t x = (int32_t) (total_frames * 256u);
    // division, not a shift of a possibly negative value; it rounds towards
    // zero, so the filter stops within 15/256 of a frame of its input
    f->filtered_q8 += (x - f->filtered_q8) / (1 << USB_DAC_FB_FILTER_SHIFT);
    // queue above the setpoint: ask for fewer frames, and the other way round
    int64_t err_q8 = (int64_t) f->setpoint * 256 - f->filtered_q8;
    int64_t corr = err_q8 * USB_DAC_FB_KP / 256;
    int64_t limit = nominal / USB_DAC_FB_CLAMP_DIV;
    if (corr > limit) corr = limit;
    if (corr < -limit) corr = -limit;
    return (uint32_t) ((int64_t) nominal + corr);
}

/// (fb - nominal) in parts per million, for the log.
static inline int32_t usb_dac_fb_ppm(uint32_t fb, uint32_t nominal){
    return (int32_t) (((int64_t) fb - (int64_t) nominal) * 1000000 / (int64_t) nominal);
}

// ---- charger detection ------------------------------------------------------
//
// A box in sound card mode on a phone charger has no host and is silently
// useless. A dedicated charger (BC1.2 DCP) shorts D+ to D-; with our D+ pull-up
// on, both lines then read high, which the controller reports as line state
// SE1 (3). A host never holds SE1, so seeing it for a while, with no address
// ever assigned, means "charger". What a host does NOT look like is just
// "quiet for N seconds": TVs and TV boxes power their ports long before their
// OS enumerates, so a plain timeout would drop exactly those into Bluetooth.

#define USB_DAC_LINE_SE0 0u
#define USB_DAC_LINE_J   1u
#define USB_DAC_LINE_K   2u
#define USB_DAC_LINE_SE1 3u     // as the SDK's rp2040_usb_device_enumeration.c names them

/// A gap between samples longer than this is not "every sample": restart the
/// count rather than vouch for time nobody looked at.
#define USB_DAC_CHARGER_MAX_GAP_MS 100u

enum usb_dac_charger_verdict {
    USB_DAC_CHARGER_KEEP_GOING = 0,
    USB_DAC_CHARGER_IS_CHARGER,     // SE1 held, no host ever addressed us
    USB_DAC_CHARGER_NO_HOST,        // secondary criterion only: never addressed in time
};

struct usb_dac_charger {
    bool     se1_run;           // the current run of SE1 samples
    bool     host_seen;         // an address was seen once: never fall back again
    uint32_t se1_since_ms;
    uint32_t last_ms;
    bool     have_last;
};

static inline void usb_dac_charger_init(struct usb_dac_charger * c){
    c->se1_run = false;
    c->host_seen = false;
    c->se1_since_ms = 0;
    c->last_ms = 0;
    c->have_last = false;
}

/// One sample. elapsed_ms counts from usb_device_start(); line_state is the
/// controller's 2 bit LINE_STATE; address_seen is "dev_addr_ctrl's address is
/// nonzero right now". The primary criterion looks only at the first
/// window_ms; se1_ms of unbroken SE1 are needed. no_host_ms == 0 disables the
/// secondary criterion, otherwise it fires when no address has been seen by
/// then, wherever the line is.
static inline enum usb_dac_charger_verdict usb_dac_charger_step(
        struct usb_dac_charger * c, uint32_t elapsed_ms, uint32_t line_state,
        bool address_seen, uint32_t window_ms, uint32_t se1_ms, uint32_t no_host_ms){
    if (address_seen) c->host_seen = true;
    if (c->host_seen) return USB_DAC_CHARGER_KEEP_GOING;

    if (no_host_ms && elapsed_ms >= no_host_ms) return USB_DAC_CHARGER_NO_HOST;

    if (c->have_last && elapsed_ms - c->last_ms > USB_DAC_CHARGER_MAX_GAP_MS){
        c->se1_run = false;
    }
    c->last_ms = elapsed_ms;
    c->have_last = true;

    if (elapsed_ms > window_ms || line_state != USB_DAC_LINE_SE1){
        c->se1_run = false;
        return USB_DAC_CHARGER_KEEP_GOING;
    }
    if (!c->se1_run){
        c->se1_run = true;
        c->se1_since_ms = elapsed_ms;
    }
    if (elapsed_ms - c->se1_since_ms >= se1_ms) return USB_DAC_CHARGER_IS_CHARGER;
    return USB_DAC_CHARGER_KEEP_GOING;
}

#endif
