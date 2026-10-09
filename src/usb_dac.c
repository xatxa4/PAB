/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2025 BambooMaster
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * The descriptor layout and the control-request handling follow pico-playground's
 * apps/usb_sound_card (Raspberry Pi, BSD-3-Clause) as modified in BambooMaster's
 * usb_sound_card_hires (the changes are MIT). The descriptor structs, address
 * validation, state handling and everything after them are original to this
 * project (MIT).
 *
 * SPDX-License-Identifier: BSD-3-Clause AND MIT
 *
 * ---- BSD-3-Clause (Raspberry Pi (Trading) Ltd.) ----
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ---- MIT (BambooMaster; xatxa4) ----
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

/*
 * usb_dac.c - USB sound card mode.
 *
 * audio_out is source-agnostic, so a UAC device only has to unpack incoming
 * packets into 32 bit frames and call audio_out_service() from this loop. This
 * file is the USB half. At this stage the device enumerates, answers the class
 * requests and plays 16 and 24 bit audio at 44.1, 48, 88.2 and 96 kHz, with
 * the host's volume and mute applied. The feedback endpoint still sends the
 * nominal value, so the ring between the USB interrupt and the output slowly
 * drifts and drops or pads audio, counted.
 *
 * The control-request, alternate-setting and endpoint handlers run in the USB
 * interrupt, so they only touch the small state struct and the ring and never
 * print. The main loop decides when to play and reports what changed.
 *
 * References worth following, in order of closeness to this design:
 *   BambooMaster/usb_sound_card_hires - same i2s.pio, async feedback, hi-res
 *   pico-playground apps/usb_sound_card - the simpler original
 */

#include "usb_dac.h"
#include "usb_dac_pcm.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "pico/usb_device.h"
#include "hardware/watchdog.h"

#include "audio_out.h"
#include "app_mode.h"
#include "mode_button.h"

#define USB_DAC_DEFAULT_RATE   48000    // what TV boxes, Android and PipeWire use

// Some hosts stop sending without selecting alt 0; no packet for this long
// with an alt selected means the stream is over.
#define USB_DAC_STREAM_TIMEOUT_US 50000u
// Slack the start gate waits for on top of one output buffer, in milliseconds
// of audio at the current rate (176 frames at 44.1 kHz, 384 at 96 kHz).
#define USB_DAC_START_MARGIN_MS 4u

// What the board draws is not known yet. Measure it with the DAC attached and
// set this to that plus a margin: iPhones and iPads (and some hubs and OTG
// Y-cables) refuse a device that asks for more than about 100 mA.
#ifndef USB_DAC_MAX_POWER_MA
#define USB_DAC_MAX_POWER_MA   100
#endif
static_assert(USB_DAC_MAX_POWER_MA >= 2 && USB_DAC_MAX_POWER_MA <= 500,
              "bMaxPower is mA/2 in one byte");

#define VENDOR_ID              0x2E8Au
#define PRODUCT_ID             0xFEDDu
// Windows caches descriptors per VID/PID/bcdDevice, so bump this whenever the
// descriptors change during development, or the old ones keep being used.
#define DEVICE_RELEASE         0x0100u

#define AUDIO_OUT_ENDPOINT     0x01u
#define AUDIO_FEEDBACK_ENDPOINT 0x82u

// 24 bit stereo at 96 kHz, 97 frames (the 1 kHz packet plus one extra frame
// when the host's clock runs ahead of ours). pico-extras sizes the endpoints
// once, from the alt 1 descriptor, so every alt must carry this same value.
#define AUDIO_MAX_PACKET_SIZE  582u

#define ENTITY_FEATURE_UNIT    2u       // input terminal 1, output terminal 3
#define FEATURE_MUTE_CONTROL   1u
#define FEATURE_VOLUME_CONTROL 2u
#define ENDPOINT_FREQ_CONTROL  1u

// UAC 1.0 class requests
#define AUDIO_REQ_SET_CUR      0x01u
#define AUDIO_REQ_GET_CUR      0x81u
#define AUDIO_REQ_GET_MIN      0x82u
#define AUDIO_REQ_GET_MAX      0x83u
#define AUDIO_REQ_GET_RES      0x84u

// Volume is signed 8.8 fixed point dB. -90 dB is the floor, 1 dB steps.
#define ENCODE_DB(x)           ((int16_t) ((x) * 256))
#define VOLUME_MIN             ENCODE_DB(-90)
#define VOLUME_MAX             ENCODE_DB(0)
#define VOLUME_RES             ENCODE_DB(1)

#define RATE_COUNT             4
#define ALT_COUNT              3        // 0 = idle, 1 = 16 bit, 2 = 24 bit

static const uint32_t supported_rates[RATE_COUNT] = {44100, 48000, 88200, 96000};

// ---- UAC 1.0 class-specific descriptors -------------------------------------
// Written out here rather than vendoring LUFA's headers, which keeps the
// licence surface to BSD-3-Clause and MIT. Every sizeof is pinned to the
// bLength the specification gives, so a typo cannot go out on the wire.

#define CS_INTERFACE           0x24u
#define CS_ENDPOINT            0x25u

struct __packed ac_header_descriptor {          // UAC 1.0 4.3.2, one streaming interface
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint16_t bcdADC;
    uint16_t wTotalLength;
    uint8_t  bInCollection;
    uint8_t  baInterfaceNr[1];
};
static_assert(sizeof(struct ac_header_descriptor) == 9, "UAC1 AC header is 8 + n");

struct __packed input_terminal_descriptor {     // 4.3.2.1
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bNrChannels;
    uint16_t wChannelConfig;
    uint8_t  iChannelNames;
    uint8_t  iTerminal;
};
static_assert(sizeof(struct input_terminal_descriptor) == 12, "UAC1 input terminal is 12");

struct __packed feature_unit_descriptor {       // 4.3.2.5, two channels plus master, 1 byte controls
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bUnitID;
    uint8_t  bSourceID;
    uint8_t  bControlSize;
    uint8_t  bmaControls[3];
    uint8_t  iFeature;
};
static_assert(sizeof(struct feature_unit_descriptor) == 10, "UAC1 feature unit is 7 + (ch + 1) * n");

struct __packed output_terminal_descriptor {    // 4.3.2.2
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  iTerminal;
};
static_assert(sizeof(struct output_terminal_descriptor) == 9, "UAC1 output terminal is 9");

struct __packed as_general_descriptor {         // 4.5.2
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bTerminalLink;
    uint8_t  bDelay;
    uint16_t wFormatTag;
};
static_assert(sizeof(struct as_general_descriptor) == 7, "UAC1 AS general is 7");

struct __packed format_type1_descriptor {       // Formats 2.2.5, discrete rates
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bFormatType;
    uint8_t  bNrChannels;
    uint8_t  bSubFrameSize;
    uint8_t  bBitResolution;
    uint8_t  bSamFreqType;
    uint8_t  tSamFreq[RATE_COUNT][3];
};
static_assert(sizeof(struct format_type1_descriptor) == 8 + 3 * RATE_COUNT,
              "UAC1 type I format is 8 + 3 * rates");

struct __packed cs_endpoint_descriptor {        // 4.6.1.2
    uint8_t  bLength, bDescriptorType, bDescriptorSubtype;
    uint8_t  bmAttributes;
    uint8_t  bLockDelayUnits;
    uint16_t wLockDelay;
};
static_assert(sizeof(struct cs_endpoint_descriptor) == 7, "UAC1 CS endpoint is 7");

// One streaming alternate setting: interface, format, data endpoint and its
// class-specific part, then the feedback endpoint.
struct __packed audio_stream_alt {
    struct usb_interface_descriptor interface;
    struct as_general_descriptor general;
    struct format_type1_descriptor format;
    struct usb_endpoint_descriptor_long data_ep;
    struct cs_endpoint_descriptor data_cs;
    struct usb_endpoint_descriptor_long feedback_ep;
};
static_assert(sizeof(struct audio_stream_alt) == 9 + 7 + 20 + 9 + 7 + 9, "one alt is 61 bytes");

struct __packed audio_device_config {
    struct usb_configuration_descriptor descriptor;
    struct usb_interface_descriptor ac_interface;
    struct __packed {
        struct ac_header_descriptor header;
        struct input_terminal_descriptor input_terminal;
        struct feature_unit_descriptor feature_unit;
        struct output_terminal_descriptor output_terminal;
    } ac_audio;
    struct usb_interface_descriptor as_zero_interface;
    struct audio_stream_alt alt1;               // 16 bit
    struct audio_stream_alt alt2;               // 24 bit
};
// usb_device copies the whole thing into a static buffer, guarded in Release
// only by an assert.
static_assert(sizeof(struct audio_device_config) <= PICO_USBDEV_MAX_DESCRIPTOR_SIZE,
              "configuration descriptor does not fit PICO_USBDEV_MAX_DESCRIPTOR_SIZE");
static_assert(sizeof(struct audio_device_config) == 189, "9 + 9 + 40 + 9 + 2 * 61");

#define SAMPLE_FREQ(f) {(uint8_t) (f), (uint8_t) ((f) >> 8), (uint8_t) ((f) >> 16)}

#define AUDIO_STREAM_ALT(alt, subframe, bits) { \
    .interface = { \
        .bLength            = sizeof(struct usb_interface_descriptor), \
        .bDescriptorType    = USB_DT_INTERFACE, \
        .bInterfaceNumber   = 1, \
        .bAlternateSetting  = (alt), \
        .bNumEndpoints      = 2, \
        .bInterfaceClass    = 0x01, /* audio */ \
        .bInterfaceSubClass = 0x02, /* audio streaming */ \
        .bInterfaceProtocol = 0x00, \
    }, \
    .general = { \
        .bLength            = sizeof(struct as_general_descriptor), \
        .bDescriptorType    = CS_INTERFACE, \
        .bDescriptorSubtype = 0x01, /* AS_GENERAL */ \
        .bTerminalLink      = 1, \
        .bDelay             = 1, \
        .wFormatTag         = 1,    /* PCM */ \
    }, \
    .format = { \
        .bLength            = sizeof(struct format_type1_descriptor), \
        .bDescriptorType    = CS_INTERFACE, \
        .bDescriptorSubtype = 0x02, /* FORMAT_TYPE */ \
        .bFormatType        = 1,    /* type I */ \
        .bNrChannels        = 2, \
        .bSubFrameSize      = (subframe), \
        .bBitResolution     = (bits), \
        .bSamFreqType       = RATE_COUNT, \
        .tSamFreq           = {SAMPLE_FREQ(44100), SAMPLE_FREQ(48000), \
                               SAMPLE_FREQ(88200), SAMPLE_FREQ(96000)}, \
    }, \
    .data_ep = { \
        .bLength          = sizeof(struct usb_endpoint_descriptor_long), \
        .bDescriptorType  = USB_DT_ENDPOINT, \
        .bEndpointAddress = AUDIO_OUT_ENDPOINT, \
        .bmAttributes     = 0x05,   /* isochronous, asynchronous */ \
        .wMaxPacketSize   = AUDIO_MAX_PACKET_SIZE, \
        .bInterval        = 1, \
        .bRefresh         = 0, \
        .bSyncAddr        = AUDIO_FEEDBACK_ENDPOINT, \
    }, \
    .data_cs = { \
        .bLength            = sizeof(struct cs_endpoint_descriptor), \
        .bDescriptorType    = CS_ENDPOINT, \
        .bDescriptorSubtype = 0x01, /* EP_GENERAL */ \
        .bmAttributes       = 0x01, /* sampling frequency control */ \
        .bLockDelayUnits    = 0, \
        .wLockDelay         = 0, \
    }, \
    .feedback_ep = { \
        .bLength          = sizeof(struct usb_endpoint_descriptor_long), \
        .bDescriptorType  = USB_DT_ENDPOINT, \
        .bEndpointAddress = AUDIO_FEEDBACK_ENDPOINT, \
        .bmAttributes     = 0x11,   /* isochronous, feedback */ \
        .wMaxPacketSize   = 3, \
        .bInterval        = 1, \
        .bRefresh         = 3,      /* every 8 ms */ \
        .bSyncAddr        = 0, \
    }, \
}

static const struct audio_device_config audio_device_config = {
    .descriptor = {
        .bLength             = sizeof(struct usb_configuration_descriptor),
        .bDescriptorType     = USB_DT_CONFIG,
        .wTotalLength        = sizeof(struct audio_device_config),
        .bNumInterfaces      = 2,
        .bConfigurationValue = 1,
        .iConfiguration      = 0,
        .bmAttributes        = 0x80,    // bus powered
        .bMaxPower           = USB_DAC_MAX_POWER_MA / 2,
    },
    .ac_interface = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 0,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 0,
        .bInterfaceClass    = 0x01,     // audio
        .bInterfaceSubClass = 0x01,     // audio control
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0,
    },
    .ac_audio = {
        .header = {
            .bLength            = sizeof(struct ac_header_descriptor),
            .bDescriptorType    = CS_INTERFACE,
            .bDescriptorSubtype = 0x01, // HEADER
            .bcdADC             = 0x0100,
            .wTotalLength       = sizeof(audio_device_config.ac_audio),
            .bInCollection      = 1,
            .baInterfaceNr      = {1},
        },
        .input_terminal = {
            .bLength            = sizeof(struct input_terminal_descriptor),
            .bDescriptorType    = CS_INTERFACE,
            .bDescriptorSubtype = 0x02, // INPUT_TERMINAL
            .bTerminalID        = 1,
            .wTerminalType      = 0x0101,   // USB streaming
            .bAssocTerminal     = 0,
            .bNrChannels        = 2,
            .wChannelConfig     = 0x0003,   // left front, right front
            .iChannelNames      = 0,
            .iTerminal          = 0,
        },
        .feature_unit = {
            .bLength            = sizeof(struct feature_unit_descriptor),
            .bDescriptorType    = CS_INTERFACE,
            .bDescriptorSubtype = 0x06, // FEATURE_UNIT
            .bUnitID            = ENTITY_FEATURE_UNIT,
            .bSourceID          = 1,
            .bControlSize       = 1,
            .bmaControls        = {0x03, 0x00, 0x00},   // mute + volume on master only
            .iFeature           = 0,
        },
        .output_terminal = {
            .bLength            = sizeof(struct output_terminal_descriptor),
            .bDescriptorType    = CS_INTERFACE,
            .bDescriptorSubtype = 0x03, // OUTPUT_TERMINAL
            .bTerminalID        = 3,
            .wTerminalType      = 0x0301,   // speaker
            .bAssocTerminal     = 0,
            .bSourceID          = ENTITY_FEATURE_UNIT,
            .iTerminal          = 0,
        },
    },
    .as_zero_interface = {
        .bLength            = sizeof(struct usb_interface_descriptor),
        .bDescriptorType    = USB_DT_INTERFACE,
        .bInterfaceNumber   = 1,
        .bAlternateSetting  = 0,
        .bNumEndpoints      = 0,
        .bInterfaceClass    = 0x01,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x00,
        .iInterface         = 0,
    },
    .alt1 = AUDIO_STREAM_ALT(1, 2, 16),
    .alt2 = AUDIO_STREAM_ALT(2, 3, 24),
};

static const struct usb_device_descriptor device_descriptor = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = 0x0110,
    .bDeviceClass       = 0,            // class comes from the interfaces
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = 64,
    .idVendor           = VENDOR_ID,
    .idProduct          = PRODUCT_ID,
    .bcdDevice          = DEVICE_RELEASE,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

// Index 0 (the language list) is answered by the stack itself.
static char serial_string[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static const char *get_descriptor_string(uint index){
    switch (index){
        case 1: return "xatxa4";
        case 2: return "PAB";
        case 3: return serial_string;
        default: return "";
    }
}

// ---- state shared between the USB interrupt and the main loop ---------------

static struct {
    volatile uint8_t  alt;          // 0 idle, 1 = 16 bit, 2 = 24 bit
    volatile uint32_t rate;         // Hz, always one of supported_rates
    volatile int16_t  volume;       // signed 8.8 dB
    volatile bool     mute;
} dac_state = {
    .alt = 0,
    .rate = USB_DAC_DEFAULT_RATE,
    .volume = VOLUME_MAX,
    .mute = false,
};

// ---- ring between the USB interrupt and the output --------------------------

// Interleaved stereo int32 frames. The USB interrupt is the only producer and
// the audio_out fill callback (main loop) the only consumer. 4096 frames is
// 42 ms at 96 kHz, a bit more than the 3 output buffers D9 asks for (3 x 1115
// frames at the highest rate, 11610 us each), and 32 KB of RAM. The most the
// ring has to hold is the start threshold, one more buffer that the output
// takes in the meantime, and the packet that arrives on top:
// 1115 + 384 + 1115 + 97 = 2711 frames at 96 kHz.
#define USB_DAC_RING_FRAMES    4096u
static_assert((USB_DAC_RING_FRAMES & (USB_DAC_RING_FRAMES - 1)) == 0,
              "the ring index math masks with capacity - 1");
#if defined(PICO_AUDIO_I2S_BUFFER_US) && defined(PICO_AUDIO_I2S_MAX_SAMPLE_RATE)
static_assert(USB_DAC_RING_FRAMES >=
              3 * (((uint64_t) PICO_AUDIO_I2S_MAX_SAMPLE_RATE * PICO_AUDIO_I2S_BUFFER_US + 500000) / 1000000),
              "the ring must hold 3 audio_out buffers at the highest rate");
static_assert(USB_DAC_RING_FRAMES >=
              2 * (((uint64_t) PICO_AUDIO_I2S_MAX_SAMPLE_RATE * PICO_AUDIO_I2S_BUFFER_US + 500000) / 1000000)
              + (uint64_t) PICO_AUDIO_I2S_MAX_SAMPLE_RATE * USB_DAC_START_MARGIN_MS / 1000 + 97,
              "the ring must hold the start threshold, a buffer and a packet at the highest rate");
#endif

static int32_t ring[USB_DAC_RING_FRAMES * 2];
// Free running frame counts, see usb_dac_pcm.h. Everything runs on one core,
// where the main loop can be interrupted by the USB interrupt but never the
// other way round, so ordering is all that is needed: the producer writes the
// frames before it advances write_index, the consumer copies them out before
// it advances read_index. Neither side takes a lock.
static volatile uint32_t write_index;       // USB interrupt only
static volatile uint32_t read_index;        // main loop only

// Bumped in the USB interrupt on every SET_INTERFACE and whenever SET_CUR
// changes the rate. The main loop restarts the stream when it differs from the
// one playback started with, so an alt 2 -> 0 -> 2 bounce between two loop
// iterations, which leaves alt and rate as they were, still flushes the ring
// instead of playing what the old selection left in it.
static volatile uint32_t stream_generation;

static volatile bool     accepting;         // set by the main loop only: false drops packets
static volatile bool     have_packet;       // a packet arrived since the alt was selected
static volatile uint32_t last_packet_us;    // time_us_32() of the last packet with data
static volatile uint32_t ring_underruns;    // fill had to pad
static volatile uint32_t ring_overruns;     // packet dropped, ring full
static volatile uint32_t packets_discarded; // packet dropped while a change was in progress

static bool rate_supported(uint32_t rate){
    for (int i = 0; i < RATE_COUNT; i++){
        if (supported_rates[i] == rate) return true;
    }
    return false;
}

// Nominal explicit feedback, samples per frame in 10.14 fixed point, sent as
// three bytes LSB first. Rounded to nearest, so 44.1 kHz is 0x0B0666 and
// 88.2 kHz 0x160CCD. rate * 16384 stays below 2^32 up to 262 kHz.
static uint32_t feedback_nominal(uint32_t rate){
    return (rate * 16384u + 500u) / 1000u;
}

// ---- control requests (USB interrupt) ---------------------------------------

enum control_id { CTRL_NONE, CTRL_MUTE, CTRL_VOLUME, CTRL_RATE };

// Work out which control a class request addresses, or CTRL_NONE if it is not
// one we have. The stack routes by interface or endpoint number only, so the
// entity (wIndex high byte) and channel have to be checked here.
static enum control_id decode_control(const struct usb_setup_packet *setup){
    uint8_t cs = (uint8_t) (setup->wValue >> 8);
    uint8_t cn = (uint8_t) setup->wValue;
    switch (setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK){
        case USB_REQ_TYPE_RECIPIENT_INTERFACE:
            if (setup->wIndex != ((ENTITY_FEATURE_UNIT << 8) | 0u)) return CTRL_NONE;
            if (cn != 0) return CTRL_NONE;      // master only
            if (cs == FEATURE_MUTE_CONTROL) return CTRL_MUTE;
            if (cs == FEATURE_VOLUME_CONTROL) return CTRL_VOLUME;
            return CTRL_NONE;
        case USB_REQ_TYPE_RECIPIENT_ENDPOINT:
            if (setup->wIndex != AUDIO_OUT_ENDPOINT) return CTRL_NONE;
            if (cn != 0) return CTRL_NONE;
            if (cs == ENDPOINT_FREQ_CONTROL) return CTRL_RATE;
            return CTRL_NONE;
        default:
            return CTRL_NONE;
    }
}

static uint control_length(enum control_id id){
    switch (id){
        case CTRL_MUTE:   return 1;
        case CTRL_VOLUME: return 2;
        case CTRL_RATE:   return 3;
        default:          return 0;
    }
}

// Reply with `data`, as many bytes as the control has, but never more than the
// host asked for: a longer data stage than wLength is a protocol error.
static bool reply(const struct usb_setup_packet *setup, enum control_id id, uint32_t data){
    uint len = control_length(id);
    if (setup->wLength == 0) return false;
    if (len > setup->wLength) len = setup->wLength;
    usb_start_tiny_control_in_transfer(data, len);
    return true;
}

static bool do_get_request(const struct usb_setup_packet *setup, enum control_id id){
    switch (setup->bRequest){
        case AUDIO_REQ_GET_CUR:
            switch (id){
                case CTRL_MUTE:   return reply(setup, id, dac_state.mute ? 1u : 0u);
                case CTRL_VOLUME: return reply(setup, id, (uint16_t) dac_state.volume);
                case CTRL_RATE:   return reply(setup, id, dac_state.rate);
                default:          return false;
            }
        case AUDIO_REQ_GET_MIN:
            return id == CTRL_VOLUME && reply(setup, id, (uint16_t) VOLUME_MIN);
        case AUDIO_REQ_GET_MAX:
            return id == CTRL_VOLUME && reply(setup, id, (uint16_t) VOLUME_MAX);
        case AUDIO_REQ_GET_RES:
            return id == CTRL_VOLUME && reply(setup, id, (uint16_t) VOLUME_RES);
        default:
            return false;
    }
}

// SET_CUR carries its value in a data stage that arrives after the setup
// packet, so what the setup asked for is remembered until then.
static struct {
    enum control_id id;
    uint8_t len;
} pending_set;

// Defined in the vendored stack but not declared in its header. The stack
// supports raising it from an on_packet handler (the transfer may already be
// complete afterwards); the argument is unused.
void usb_stall_control_pipe(struct usb_setup_packet *setup);

static void set_cur_packet(struct usb_endpoint *ep){
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    enum control_id id = pending_set.id;
    pending_set.id = CTRL_NONE;
    if (id == CTRL_NONE || buffer->data_len < pending_set.len){
        // no request pending, or a data stage too short to hold the value
        usb_stall_control_pipe(NULL);
        return;
    }
    const uint8_t *d = buffer->data;
    switch (id){
        case CTRL_MUTE:
            dac_state.mute = d[0] != 0;
            break;
        case CTRL_VOLUME: {
            // Byte by byte: the buffer is not promised to be 2 byte aligned.
            int16_t v = (int16_t) ((uint16_t) d[0] | (uint16_t) d[1] << 8);
            // 0x8000 means -infinity; clamp that and anything else out of range.
            if (v < VOLUME_MIN) v = VOLUME_MIN;
            if (v > VOLUME_MAX) v = VOLUME_MAX;
            dac_state.volume = v;
            break;
        }
        case CTRL_RATE: {
            uint32_t rate = (uint32_t) d[0] | (uint32_t) d[1] << 8 | (uint32_t) d[2] << 16;
            // Anything but the four advertised rates is refused, not
            // mapped to some default as upstream does: the value only
            // arrives in this data stage, so the status stage is stalled
            // here and the host sees the failure.
            if (!rate_supported(rate)){
                usb_stall_control_pipe(NULL);
                return;
            }
            if (rate != dac_state.rate){
                dac_state.rate = rate;
                // a repeat of the current rate must not restart playback
                stream_generation = stream_generation + 1;
            }
            break;
        }
        default:
            break;
    }
    usb_start_empty_control_in_transfer_null_completion();
}

static const struct usb_transfer_type set_cur_transfer_type = {
    .on_packet = set_cur_packet,
    .initial_packet_count = 1,
};

static bool do_set_current(const struct usb_setup_packet *setup, enum control_id id){
    // Exactly the size of the control: a short stage cannot hold the value and
    // a long one is not ours.
    if (id == CTRL_NONE || setup->wLength != control_length(id)) return false;
    pending_set.id = id;
    pending_set.len = (uint8_t) setup->wLength;
    usb_start_control_out_transfer(&set_cur_transfer_type);
    return true;
}

// Returning false makes the stack stall the control pipe.
static bool audio_class_request(const struct usb_setup_packet *setup){
    if ((setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK) != USB_REQ_TYPE_TYPE_CLASS) return false;
    enum control_id id = decode_control(setup);
    if (id == CTRL_NONE) return false;
    bool in = (setup->bmRequestType & USB_DIR_IN) != 0;
    if (setup->bRequest == AUDIO_REQ_SET_CUR) return !in && do_set_current(setup, id);
    return in && do_get_request(setup, id);
}

static bool ac_setup_request_handler(__unused struct usb_interface *interface,
                                     struct usb_setup_packet *setup){
    return audio_class_request(setup);
}

// The sampling frequency control is addressed to the OUT endpoint.
static bool ep_setup_request_handler(__unused struct usb_endpoint *ep,
                                     struct usb_setup_packet *setup){
    return audio_class_request(setup);
}

static bool as_set_alternate(__unused struct usb_interface *interface, uint alt){
    if (alt >= ALT_COUNT) return false;
    dac_state.alt = (uint8_t) alt;
    // A packet from the previous selection must not count as a live stream.
    have_packet = false;
    stream_generation = stream_generation + 1;
    return true;
}

// ---- endpoints (USB interrupt) ----------------------------------------------

// Unpack the packet straight into the ring, or drop all of it: half a packet
// would leave the stream a few frames short in the middle of a sample run.
static void audio_out_packet(struct usb_endpoint *ep){
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    uint8_t alt = dac_state.alt;
    if (alt != 0 && buffer->data_len > 0){
        last_packet_us = time_us_32();
        have_packet = true;
        if (accepting){
            uint32_t subframe = (uint32_t) alt + 1u;        // alt 1 = 16 bit, alt 2 = 24 bit
            uint32_t frame_bytes = 2 * subframe;
            uint32_t frames = buffer->data_len / frame_bytes;
            uint32_t w = write_index;
            if (frames > usb_dac_ring_free(w, read_index, USB_DAC_RING_FRAMES)){
                ring_overruns++;
            } else if (frames){
                uint32_t slot = usb_dac_ring_slot(w, USB_DAC_RING_FRAMES);
                uint32_t first = USB_DAC_RING_FRAMES - slot;    // frames up to the end of the array
                if (first > frames) first = frames;
                usb_dac_unpack(buffer->data, first * frame_bytes, subframe, &ring[2 * slot]);
                if (frames > first){
                    usb_dac_unpack(buffer->data + first * frame_bytes, (frames - first) * frame_bytes,
                                   subframe, &ring[0]);
                }
                // volatile alone does not keep the plain stores above from
                // sinking below the index; the barrier does, at no cost
                __compiler_memory_barrier();
                write_index = w + frames;       // after the frames are in place
            }
        } else {
            packets_discarded++;
        }
    }
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static void feedback_packet(struct usb_endpoint *ep){
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    uint32_t fb = feedback_nominal(dac_state.rate);
    buffer->data[0] = (uint8_t) fb;
    buffer->data[1] = (uint8_t) (fb >> 8);
    buffer->data[2] = (uint8_t) (fb >> 16);
    buffer->data_len = 3;
    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
}

static const struct usb_transfer_type audio_out_transfer_type = {
    .on_packet = audio_out_packet,
    .initial_packet_count = 1,
};

static const struct usb_transfer_type feedback_transfer_type = {
    .on_packet = feedback_packet,
    .initial_packet_count = 1,
};

static struct usb_interface ac_interface;
static struct usb_interface as_interface;
static struct usb_endpoint ep_audio_out, ep_feedback;
static struct usb_transfer audio_out_transfer, feedback_transfer;

static void usb_dac_start_device(void){
    pico_get_unique_board_id_string(serial_string, sizeof(serial_string));

    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true);
    ac_interface.setup_request_handler = ac_setup_request_handler;

    static struct usb_endpoint *const as_endpoints[] = {&ep_audio_out, &ep_feedback};
    usb_interface_init(&as_interface, &audio_device_config.alt1.interface,
                       as_endpoints, count_of(as_endpoints), true);
    as_interface.set_alternate_handler = as_set_alternate;
    ep_audio_out.setup_request_handler = ep_setup_request_handler;

    audio_out_transfer.type = &audio_out_transfer_type;
    usb_set_default_transfer(&ep_audio_out, &audio_out_transfer);
    feedback_transfer.type = &feedback_transfer_type;
    usb_set_default_transfer(&ep_feedback, &feedback_transfer);

    static struct usb_interface *const interfaces[] = {&ac_interface, &as_interface};
    struct usb_device *device = usb_device_init(&device_descriptor, &audio_device_config.descriptor,
                                                interfaces, count_of(interfaces),
                                                get_descriptor_string);
    if (!device){
        printf("USB DAC         : cannot set up the USB device\n");
        return;
    }
    usb_device_start();
}

// ---- playback (main loop) ---------------------------------------------------

static bool gate_open;      // main loop only, see dac_fill

// Called by audio_out_service() from the main loop. Always writes num_frames:
// audio_out reuses its buffers, so anything left unwritten would play twice.
static void dac_fill(int32_t * dst, uint32_t num_frames, void * context){
    (void) context;
    uint32_t r = read_index;
    uint32_t level = usb_dac_ring_level(write_index, r);

    // audio_out_start() fills every buffer at once. Handing it audio only as
    // it trickles in would play audio, silence, audio. So whole buffers of
    // silence until a buffer and a margin are queued, then audio from there on.
    if (!gate_open){
        uint32_t margin = audio_out_sample_rate() * USB_DAC_START_MARGIN_MS / 1000u;
        if (level < audio_out_frames_per_buffer() + margin){
            memset(dst, 0, (size_t) num_frames * 2 * sizeof(int32_t));
            return;
        }
        gate_open = true;
    }

    uint32_t n = level < num_frames ? level : num_frames;
    uint32_t slot = usb_dac_ring_slot(r, USB_DAC_RING_FRAMES);
    uint32_t first = USB_DAC_RING_FRAMES - slot;
    if (first > n) first = n;
    memcpy(dst, &ring[2 * slot], (size_t) first * 2 * sizeof(int32_t));
    if (n > first) memcpy(dst + 2 * first, &ring[0], (size_t) (n - first) * 2 * sizeof(int32_t));
    __compiler_memory_barrier();    // the copies above stay above, as for write_index
    read_index = r + n;             // after the frames are copied out

    if (n < num_frames){
        // A dry spell: pad, then wait for the ring to refill rather than
        // stutter through it one packet at a time.
        memset(dst + 2 * n, 0, (size_t) (num_frames - n) * 2 * sizeof(int32_t));
        ring_underruns++;
        gate_open = false;
    }
}

// Hands the host's volume and mute to the output when either changed. Main
// loop only: dB to gain is a table lookup, but audio_out is not for the
// interrupt, and a change never restarts the stream.
static void apply_volume(void){
    static bool applied;
    static int16_t applied_volume;
    static bool applied_mute;

    int16_t volume = dac_state.volume;
    bool mute = dac_state.mute;
    if (applied && volume == applied_volume && mute == applied_mute) return;
    applied = true;
    applied_volume = volume;
    applied_mute = mute;
    // Muting keeps dac_state.volume, so unmuting comes back to where it was.
    audio_out_set_volume(mute ? 0 : (int32_t) usb_dac_volume_gain(volume));
}

// Decides from the main loop, not the interrupt, whether the output plays.
// audio_out_stop() and audio_out_start() are not for interrupt context.
static void stream_update(void){
    static bool playing;
    static uint8_t played_alt;
    static uint32_t played_rate;
    static uint32_t played_generation;
    // set when the output refused a rate; not retried until rate or generation moves
    static bool refused;
    static uint32_t refused_rate;
    static uint32_t refused_generation;

    // The interrupt may change these between the reads; read again until the
    // generation holds still so alt and rate belong to it.
    // The packet stamp is read in the same pass, so a SET_INTERFACE between the
    // reads cannot pair the old alt with a packet of the new one.
    uint32_t generation;
    uint8_t alt;
    uint32_t rate;
    bool any_packet;
    uint32_t last;
    do {
        generation = stream_generation;
        alt = dac_state.alt;
        rate = dac_state.rate;
        any_packet = have_packet;
        last = last_packet_us;
    } while (generation != stream_generation);
    // The packet time is read before the clock: the interrupt may stamp a
    // newer packet in between, and a stamp later than "now" would wrap the
    // unsigned difference to a huge age and stop a healthy stream. Unsigned
    // subtraction also survives the 32 bit microsecond counter wrapping.
    bool recent = any_packet && (uint32_t) (time_us_32() - last) < USB_DAC_STREAM_TIMEOUT_US;
    bool want = alt != 0 && recent;

    if (playing && (!want || alt != played_alt || rate != played_rate || generation != played_generation)){
        audio_out_stop();
        accepting = false;
        playing = false;
        gpio_put(CONN_PIN, 0);
        printf("USB DAC         : stopped (%s)\n",
               alt == 0 ? "idle" : rate != played_rate ? "rate changed" :
               alt != played_alt ? "format changed" :
               generation != played_generation ? "restarted" : "no packets");
    }

    if (refused && (rate != refused_rate || generation != refused_generation)) refused = false;

    if (!playing && want && !refused){
        // Packets may have arrived since the alt was selected, and from an
        // earlier stream: start from an empty ring and a closed gate. Packets
        // are dropped (and counted) until then. The output is stopped here, so
        // it accepts a new rate; it prints its own "I2S : N Hz" line.
        accepting = false;
        read_index = write_index;
        bool ok = rate == audio_out_sample_rate() || audio_out_set_sample_rate(rate);
        if (!ok){
            refused = true;
            refused_rate = rate;
            refused_generation = generation;
            printf("USB DAC         : cannot play %lu Hz\n", (unsigned long) rate);
        } else {
            gate_open = false;
            accepting = true;
            played_alt = alt;
            played_rate = rate;
            played_generation = generation;
            audio_out_start(dac_fill, NULL);
            playing = true;
            gpio_put(CONN_PIN, 1);
            printf("USB DAC         : playing %d bit, %lu Hz\n", alt == 2 ? 24 : 16, (unsigned long) rate);
        }
    }
}

// ---- main loop --------------------------------------------------------------

// Prints what the host last set, once per change. Runs here because the
// interrupt that records it must not print.
static void report_changes(void){
    static bool first = true;
    static uint8_t last_alt;
    static uint32_t last_rate;
    static int16_t last_volume;
    static bool last_mute;

    uint8_t alt = dac_state.alt;
    uint32_t rate = dac_state.rate;
    int16_t volume = dac_state.volume;
    bool mute = dac_state.mute;
    if (!first && alt == last_alt && rate == last_rate && volume == last_volume && mute == last_mute) return;
    first = false;
    last_alt = alt;
    last_rate = rate;
    last_volume = volume;
    last_mute = mute;

    int tenths = ((int) volume * 10) / 256;
    const char *format = alt == 1 ? " (16 bit)" : alt == 2 ? " (24 bit)" : " (idle)";
    printf("USB DAC         : alt %u%s, %lu Hz, %s%d.%d dB%s\n",
           (unsigned) alt, format, (unsigned long) rate,
           tenths < 0 ? "-" : "", tenths < 0 ? -tenths / 10 : tenths / 10,
           tenths < 0 ? -tenths % 10 : tenths % 10, mute ? ", muted" : "");
}

// Once a second, and only when something changed, like the Bluetooth sink.
static void report_counters(void){
    static uint32_t reported_underruns, reported_overruns, reported_discarded, reported_i2s, reported_late;
    static uint32_t last_report_us;
    uint32_t now = time_us_32();
    if ((uint32_t) (now - last_report_us) < 1000000u) return;
    last_report_us = now;

    uint32_t underruns = ring_underruns;
    uint32_t overruns = ring_overruns;
    uint32_t discarded = packets_discarded;
    if (underruns != reported_underruns || overruns != reported_overruns
            || discarded != reported_discarded){
        reported_underruns = underruns;
        reported_overruns = overruns;
        reported_discarded = discarded;
        printf("USB DAC         : %lu underruns, %lu overruns, %lu discarded\n",
               (unsigned long) underruns, (unsigned long) overruns, (unsigned long) discarded);
    }

    uint32_t i2s = audio_out_underruns();
    uint32_t late = audio_out_late_irqs();
    if (i2s != reported_i2s || late != reported_late){
        reported_i2s = i2s;
        reported_late = late;
        printf("I2S             : %lu underruns, %lu late refills\n",
               (unsigned long) i2s, (unsigned long) late);
    }
}

void usb_dac_run(void){
    gpio_init(CONN_PIN);
    gpio_set_dir(CONN_PIN, GPIO_OUT);
    gpio_put(CONN_PIN, 0);  // set to 1 while a stream is playing

    // Clocks come up anyway, so the DAC stays locked and the wiring can be
    // checked with a scope in this mode.
    if (!audio_out_init(USB_DAC_DEFAULT_RATE)){
        printf("USB DAC         : cannot play %d Hz\n", USB_DAC_DEFAULT_RATE);
    }

    apply_volume();     // the host's volume is in force before the first sample
    usb_dac_start_device();
    printf("USB DAC         : enumerating, config descriptor %u bytes, %u mA\n",
           (unsigned) sizeof(audio_device_config), (unsigned) USB_DAC_MAX_POWER_MA);

    while (true){
        apply_volume();
        stream_update();
        audio_out_service();    // without a fill callback this plays silence
        report_changes();
        report_counters();
        mode_button_poll();     // rate limits itself
        watchdog_update();
        sleep_ms(audio_out_service_interval_ms());
    }
}
