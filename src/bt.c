/*
 * Copyright (C) 2023 BlueKitchen GmbH
 * Copyright (c) 2024 joba-1 <https://github.com/joba-1/PicoW_A2DP>
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * Derived from BTstack's a2dp_sink_demo, by way of joba-1/PicoW_A2DP.
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

#include "bt.h"

#include "sdp.h"
#include "a2dp.h"
#include "avrcp.h"

#include <hci.h>
#include "l2cap.h"
#include <btstack_event.h>
#include <btstack_run_loop.h>

#include <memory.h>


static bool _is_up = false;
static bd_addr_t _local_addr = {0};
static bt_on_up_cb_t _cb = 0;
static void *_data = 0;
static const char *_name = 0;
static const char *_pin = 0;
static btstack_packet_callback_registration_t _hci_registration;


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);

    bd_addr_t address;

    if (packet_type != HCI_EVENT_PACKET) return;

    switch(hci_event_packet_get_type(packet)) {

        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            gap_local_bd_addr(_local_addr);
            _is_up = true;
            if (_cb) (*_cb)(_data);
            break;

        case HCI_EVENT_PIN_CODE_REQUEST:
            hci_event_pin_code_request_get_bd_addr(packet, address);
            gap_pin_code_response(address, _pin);
            break;

        default:
            break;
    }
}


void bt_begin( const char *name, const char *pin, bt_on_up_cb_t cb, void *data ) {
    _name = name ? name : "Pico 00:00:00:00:00:00";
    _pin = pin ? pin : "0000";
    _data = data;
    _cb = cb;

    l2cap_init();
    sdp_begin();

    a2dp_sink_begin();
    avrcp_begin();

    gap_set_local_name(_name);
    gap_discoverable_control(1); 
    gap_set_class_of_device(0x200414);  // Service Class: Audio, Major Device Class: Audio, Minor: Loudspeaker
    // Role switch stays on so a phone can become master after re-connect. Sniff
    // does not: it lets the source park the link mid stream, and the power it
    // saves is irrelevant to a mains powered speaker.
    gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH );
    gap_set_allow_role_switch(true);  // A2DP Source, e.g. smartphone, can become master after re-connect.

    _hci_registration.callback = &packet_handler;
    hci_add_event_handler(&_hci_registration);
}


void bt_run() {
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
}


bool bt_up() {
    return _is_up;
}


void bt_addr( bd_addr_t local_addr ) {
    memcpy(local_addr, _local_addr, sizeof(bd_addr_t));
}
