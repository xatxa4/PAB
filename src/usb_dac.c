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

#define USB_DAC_SAMPLE_RATE  48000

void usb_dac_run(void){
    printf("USB DAC         : mode selected, device not implemented yet\n");

    // Clocks come up anyway, so the DAC stays locked and the wiring can be
    // checked with a scope in this mode.
    audio_out_init(USB_DAC_SAMPLE_RATE);

    while (true){
        audio_out_service();    // no fill callback registered: plays silence
        sleep_ms(5);
    }
}
