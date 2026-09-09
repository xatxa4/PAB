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

/*
 * usb_dac.c - USB sound card mode.
 *
 * The output half is done: audio_out is source-agnostic, so a UAC device only
 * has to unpack incoming packets into 32 bit frames and call audio_out_service()
 * from this loop. What is missing is the USB device itself - descriptors, the
 * isochronous OUT endpoint, an explicit feedback endpoint so the host tracks
 * our clock, and the sample rate and volume controls.
 *
 * That is a few hundred lines with no way to test it from here, so it is not
 * written yet rather than written blind. The mode exists so the switching and
 * persistence around it can be exercised with the Bluetooth side working.
 *
 * References worth following, in order of closeness to this design:
 *   BambooMaster/usb_sound_card_hires - same i2s.pio, async feedback, hi-res
 *   pico-playground apps/usb_sound_card - the simpler original
 *   TinyUSB uac2_headset example       - if going via the SDK's TinyUSB
 */

#include "usb_dac.h"

#include <stdio.h>

#include "pico/stdlib.h"

#include "audio_out.h"
#include "app_mode.h"
#include "mode_button.h"

#define USB_DAC_SAMPLE_RATE  48000

void usb_dac_run(void){
    printf("USB DAC         : mode selected, device not implemented yet\n");

    // Clocks come up anyway, so the DAC stays locked and the wiring can be
    // checked with a scope in this mode.
    audio_out_init(USB_DAC_SAMPLE_RATE);

    while (true){
        audio_out_service();    // no fill callback registered: plays silence
        mode_button_poll();     // rate limits itself, so 5ms here is fine
        sleep_ms(5);
    }
}
