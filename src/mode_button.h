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

#ifndef MODE_BUTTON_H
#define MODE_BUTTON_H

#include <stdbool.h>

/*
 * BOOTSEL as the mode button.
 *
 * The Pico W has no user button, but BOOTSEL can be read at runtime. It sits on
 * the flash chip select line, so taking QSPI_SS's output driver off and reading
 * the pin back says whether it is held: the pull up on the line gives a 1, a
 * finger on the button gives a 0. For that window there is no chip select for
 * XIP to use, which is why the read runs from RAM with interrupts off, and why
 * we sample twenty times a second rather than continuously.
 *
 * Holding it flashes the onboard LED three times and reboots into the next
 * mode. The flashes are not only feedback: the bootrom looks at this same
 * button after the reset, so being still on it would bring the Pico up as a USB
 * drive instead of in the new mode. The blink is your cue to let go, and we
 * wait for the release before rebooting.
 */

/// True while BOOTSEL is held. Costs about 100us with interrupts disabled, so
/// this is not something to sit in a loop on.
bool mode_button_pressed(void);

/// Take one debounced sample and, once the button has been held long enough,
/// switch modes - in which case this does not return. Rate limits itself, so
/// calling it more often than it polls costs nothing.
void mode_button_poll(void);

/// Register a BTstack run loop timer that calls mode_button_poll(). For modes
/// that hand their loop to BTstack; call it after the stack is set up and
/// before btstack_run_loop_execute(). Modes that own their loop call
/// mode_button_poll() from it instead.
void mode_button_start(void);

#endif
