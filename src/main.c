/*
 * Copyright (C) 2023 BlueKitchen GmbH
 * Copyright (c) 2024 joba-1 <https://github.com/joba-1/PicoW_A2DP>
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * Derived from joba-1/PicoW_A2DP, itself built on BTstack's examples.
 *
 * SPDX-License-Identifier: LicenseRef-BlueKitchen-NonCommercial
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"

#include "btstack_run_loop.h"

#include "app_mode.h"
#include "bt.h"
#include "mode_button.h"
#include "usb_dac.h"
#include "pab_version.h"

// `picotool info` reads this off a .uf2 before it is flashed, which settles
// whether a binary contains a fix without having to boot it.
bi_decl(bi_program_version_string(PAB_GIT_VERSION));

// Without this, a panic or hard fault ends in a breakpoint and the core sits
// locked up until power-cycled, and so does a Bluetooth controller that never
// comes up. Long enough for a firmware download and a 400ms flash erase, short
// of the RP2040's ~8.3s maximum.
#ifndef PAB_WATCHDOG_MS
#define PAB_WATCHDOG_MS         8000
#endif
#define PAB_WATCHDOG_FEED_MS    250
#define PAB_BT_UP_DEADLINE_MS   20000   // after boot, for the controller to come up

static volatile bool _fatal_entered;


// Unrecoverable error happened. Reboot by setting watchdog.
// Blink led until watchdog fires
// If RUN_PIN is defined then try reset via run pin after 5 blinks
void fatal() {
    _fatal_entered = true;        // so the feeder cannot keep this alive
    watchdog_enable(1000, true);  // reboot in 1s
    #ifdef RUN_PIN
        unsigned count = 0;
    #endif
    while(true) {  // blink until reboot
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
        sleep_ms(20);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        sleep_ms(80);
        #ifdef RUN_PIN
            if (++count >= 5) {
                // Pull run pin low, to reset the pico
                gpio_init(RUN_PIN);
                gpio_set_dir(RUN_PIN, GPIO_OUT);
                gpio_put(RUN_PIN, (count & 1) ? false : true);
            }
        #endif
    }
}


void on_bt_up( void * ) {
    printf("Bluetooth stack is up\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
}


// Fed from a BTstack timer, so a wedged BTstack context stops it as surely as a
// fault does, and only while the controller is up or still coming up.
static btstack_timer_source_t _watchdog_timer;

static void watchdog_timer_handler(btstack_timer_source_t * ts) {
    if (!_fatal_entered && bt_healthy(PAB_BT_UP_DEADLINE_MS)) {
        watchdog_update();
    }
    btstack_run_loop_set_timer(ts, PAB_WATCHDOG_FEED_MS);
    btstack_run_loop_add_timer(ts);
}


static void bt_sink_run(void) {
    // before the radio: a controller bring-up that hangs is caught as well
    watchdog_enable(PAB_WATCHDOG_MS, true);

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("Failed to init cyw43_arch\n");
        fatal();
    }

    // led on during setup until bt is up
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);

    bt_begin(BT_NAME, BT_PIN, on_bt_up, NULL);

    // BTstack owns the loop from here on, so the mode button rides along on a
    // run loop timer rather than being polled by us.
    mode_button_start();

    btstack_run_loop_set_timer_handler(&_watchdog_timer, &watchdog_timer_handler);
    btstack_run_loop_set_timer(&_watchdog_timer, PAB_WATCHDOG_FEED_MS);
    btstack_run_loop_add_timer(&_watchdog_timer);

    printf("Setup done\n");
    bt_run();
}


int main() {
    stdio_init_all();

    printf("PAB             : %s\n", PAB_GIT_VERSION);

    app_mode_t mode = app_mode_current();
    printf("MODE            : %s\n", app_mode_name(mode));

    switch (mode) {
        case APP_MODE_USB_DAC:
            watchdog_enable(PAB_WATCHDOG_MS, true);     // fed from its loop
            usb_dac_run();
            break;
        case APP_MODE_BT_SINK:
        default:
            bt_sink_run();
            break;
    }

    fatal();
    return -2;
}
