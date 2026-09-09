/*
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * Original to this project. The BOOTSEL read follows the technique in the Pico
 * SDK's picoboard/button example (Raspberry Pi (Trading) Ltd., BSD-3-Clause).
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

#include "mode_button.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"

#include "btstack_run_loop.h"

#include "app_mode.h"

// How often the button is sampled. Each sample stops the world for ~100us, so
// this is a compromise between feeling responsive and staying out of the way.
#ifndef MODE_BUTTON_POLL_MS
#define MODE_BUTTON_POLL_MS             50
#endif

// Consecutive samples the button must be down for before we believe it.
#ifndef MODE_BUTTON_DEBOUNCE_POLLS
#define MODE_BUTTON_DEBOUNCE_POLLS      3
#endif

#ifndef MODE_BUTTON_BLINKS
#define MODE_BUTTON_BLINKS              3
#endif
#ifndef MODE_BUTTON_BLINK_ON_MS
#define MODE_BUTTON_BLINK_ON_MS         300
#endif
#ifndef MODE_BUTTON_BLINK_OFF_MS
#define MODE_BUTTON_BLINK_OFF_MS        300
#endif

// Fast blink while we are waiting for you to let go of the button.
#ifndef MODE_BUTTON_RELEASE_BLINK_MS
#define MODE_BUTTON_RELEASE_BLINK_MS    60
#endif

// A button that never reads released is more likely to be a wiring fault than
// a patient finger, so do not wait forever.
#ifndef MODE_BUTTON_RELEASE_TIMEOUT_MS
#define MODE_BUTTON_RELEASE_TIMEOUT_MS  5000
#endif

// GPIO_QSPI_SS: index 1 in the QSPI bank, and bit 1 of gpio_hi_in.
#define QSPI_SS_INDEX                   1

// Time for the line to settle once we stop driving it, and for the flash to see
// a clean deselect. ~1000 iterations of a volatile loop, per the SDK example.
#define SETTLE_ITERATIONS               1000

static btstack_timer_source_t poll_timer;


// Must not run from flash: the chip select being borrowed here is the one XIP
// fetches through, so for the length of this function there is no code to fetch
// and no interrupt may be taken.
bool __no_inline_not_in_flash_func(mode_button_pressed)(void){
    uint32_t saved = save_and_disable_interrupts();

    io_rw_32 * ctrl_reg = &ioqspi_hw->io[QSPI_SS_INDEX].ctrl;
    uint32_t   ctrl     = *ctrl_reg;

    // Stop driving chip select. The board's pull up holds it high; the button
    // shorts it to ground.
    *ctrl_reg = (ctrl & ~IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS)
              | ((uint32_t) GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB);

    for (volatile int i = 0; i < SETTLE_ITERATIONS; i++) tight_loop_contents();

    bool pressed = (sio_hw->gpio_hi_in & (1u << QSPI_SS_INDEX)) == 0;

    *ctrl_reg = ctrl;   // hand the line back exactly as we found it

    restore_interrupts(saved);
    return pressed;
}


static void led_set(bool on){
#ifdef CYW43_WL_GPIO_LED_PIN
    // The onboard LED hangs off the radio chip, so there is an LED to drive
    // only once cyw43_arch_init() has run. True in Bluetooth mode; not in USB
    // sound card mode, which never brings the radio up.
    if (cyw43_is_initialized(&cyw43_state)){
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
    }
#else
    (void) on;
#endif
}


// Blink, wait for the release, reboot. Blocks whatever loop called us, which is
// fine: nothing after this matters.
static void announce_and_switch(void){
    printf("MODE            : BOOTSEL held, switching to %s\n",
           app_mode_name(app_mode_next()));

    for (unsigned i = 0; i < MODE_BUTTON_BLINKS; i++){
        led_set(true);
        sleep_ms(MODE_BUTTON_BLINK_ON_MS);
        led_set(false);
        sleep_ms(MODE_BUTTON_BLINK_OFF_MS);
    }

    // The bootrom samples this same button after the watchdog reset. Rebooting
    // with it still held would come up as a USB drive instead of in the new
    // mode, so wait it out, blinking fast to say so.
    uint64_t give_up_us = time_us_64() + MODE_BUTTON_RELEASE_TIMEOUT_MS * 1000ull;
    while (mode_button_pressed()){
        if (time_us_64() >= give_up_us){
            printf("MODE            : BOOTSEL still held after %d ms, rebooting "
                   "anyway - keep holding and you get the bootloader\n",
                   MODE_BUTTON_RELEASE_TIMEOUT_MS);
            break;
        }
        led_set(true);
        sleep_ms(MODE_BUTTON_RELEASE_BLINK_MS);
        led_set(false);
        sleep_ms(MODE_BUTTON_RELEASE_BLINK_MS);
    }
    led_set(false);

    app_mode_switch_next();     // does not return
}


void mode_button_poll(void){
    static uint64_t next_poll_us;
    static unsigned held_polls;

    uint64_t now = time_us_64();
    if (now < next_poll_us) return;
    next_poll_us = now + MODE_BUTTON_POLL_MS * 1000ull;

    if (!mode_button_pressed()){
        held_polls = 0;
        return;
    }

    // Act once on the way down, not repeatedly while it is held.
    if (++held_polls == MODE_BUTTON_DEBOUNCE_POLLS){
        announce_and_switch();  // does not return
    }
}


static void poll_timer_handler(btstack_timer_source_t * ts){
    mode_button_poll();
    btstack_run_loop_set_timer(ts, MODE_BUTTON_POLL_MS);
    btstack_run_loop_add_timer(ts);
}


void mode_button_start(void){
    btstack_run_loop_set_timer_handler(&poll_timer, &poll_timer_handler);
    btstack_run_loop_set_timer(&poll_timer, MODE_BUTTON_POLL_MS);
    btstack_run_loop_add_timer(&poll_timer);
}
