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
 * between the USB interrupt and the main loop, and the volume to gain table.
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

#endif
