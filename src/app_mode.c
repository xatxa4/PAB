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

#include "app_mode.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

// One sector for the stored mode. BTstack keeps its link keys in the top two
// sectors (PICO_FLASH_BANK_TOTAL_SIZE), so sit immediately below those.
#ifndef APP_MODE_FLASH_OFFSET
#define APP_MODE_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - (3 * FLASH_SECTOR_SIZE))
#endif

// Survives a watchdog reset but is undefined at power on, hence the tag.
#define SCRATCH_TAG         0x0DE50000u
#define SCRATCH_TAG_MASK    0xFFFF0000u
#define SCRATCH_INDEX       0          // 4..7 belong to the SDK's reboot vector

#define FLASH_TAG           0xA5

static bool       resolved;
static app_mode_t current = APP_MODE_BT_SINK;

static app_mode_t mode_from_flash(void){
    const uint8_t * stored = (const uint8_t *) (XIP_BASE + APP_MODE_FLASH_OFFSET);
    if (stored[0] != FLASH_TAG) return APP_MODE_BT_SINK;   // never written
    if (stored[1] >= APP_MODE_COUNT) return APP_MODE_BT_SINK;
    return (app_mode_t) stored[1];
}

static void mode_write_flash(void * param){
    uint8_t mode = *(const uint8_t *) param;

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xff, sizeof(page));
    page[0] = FLASH_TAG;
    page[1] = mode;

    flash_range_erase(APP_MODE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(APP_MODE_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
}

app_mode_t app_mode_current(void){
    if (resolved) return current;

    uint32_t scratch = watchdog_hw->scratch[SCRATCH_INDEX];
    if ((scratch & SCRATCH_TAG_MASK) == SCRATCH_TAG){
        // consume it, so a later crash-and-watchdog-reboot does not repeat it
        watchdog_hw->scratch[SCRATCH_INDEX] = 0;
        uint8_t requested = (uint8_t) (scratch & 0xff);
        if (requested < APP_MODE_COUNT){
            current  = (app_mode_t) requested;
            resolved = true;
            return current;
        }
    }

    current  = mode_from_flash();
    resolved = true;
    return current;
}

void app_mode_switch_to(app_mode_t mode){
    if (mode >= APP_MODE_COUNT) mode = APP_MODE_BT_SINK;

    if (mode_from_flash() != mode){
        uint8_t stored = (uint8_t) mode;
        int rc = flash_safe_execute(mode_write_flash, &stored, UINT32_MAX);
        if (rc != PICO_OK){
            // the scratch register still carries us into the mode this time,
            // it just will not survive a power cycle
            printf("MODE            : could not store mode (%d)\n", rc);
        }
    }

    printf("MODE            : switching to %s\n", app_mode_name(mode));

    watchdog_hw->scratch[SCRATCH_INDEX] = SCRATCH_TAG | (uint32_t) mode;
    watchdog_reboot(0, 0, 0);
    while (true) tight_loop_contents();
}

app_mode_t app_mode_next(void){
    return (app_mode_t) ((app_mode_current() + 1) % APP_MODE_COUNT);
}


void app_mode_switch_next(void){
    app_mode_switch_to(app_mode_next());
}

const char * app_mode_name(app_mode_t mode){
    switch (mode){
        case APP_MODE_BT_SINK: return "Bluetooth sink";
        case APP_MODE_USB_DAC: return "USB sound card";
        default:               return "unknown";
    }
}
