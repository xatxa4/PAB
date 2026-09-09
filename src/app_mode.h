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

#ifndef APP_MODE_H
#define APP_MODE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Which personality the box boots into.
 *
 * Switching is a reboot, deliberately: Bluetooth and USB audio each want the
 * radio, the USB device and a set of DMA channels to themselves, and tearing
 * one down cleanly at runtime is far more ways to go wrong than starting from
 * reset. The requested mode is left in a watchdog scratch register, which
 * survives the reset, and written to flash so a cold start comes back the same
 * way. Flash is only touched when the mode actually changes.
 */

typedef enum {
    APP_MODE_BT_SINK = 0,
    APP_MODE_USB_DAC = 1,
    APP_MODE_COUNT
} app_mode_t;

/// Mode for this boot: the one just requested if we got here via
/// app_mode_switch_to(), otherwise the last one stored in flash, otherwise
/// APP_MODE_BT_SINK. Safe to call more than once.
app_mode_t app_mode_current(void);

/// The mode a single button press moves to: the next one in the cycle.
app_mode_t app_mode_next(void);

/// Persist mode if it differs from what is stored, then reboot into it.
/// Does not return.
void app_mode_switch_to(app_mode_t mode);

/// Convenience for a single button: switch to the next mode in the cycle.
/// Does not return.
void app_mode_switch_next(void);

const char * app_mode_name(app_mode_t mode);

#endif
