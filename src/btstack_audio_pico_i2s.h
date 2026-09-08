/*
 * Copyright (C) 2023 BlueKitchen GmbH
 * Copyright (c) 2024 joba-1 <https://github.com/joba-1/PicoW_A2DP>
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * Derived from BTstack's btstack_audio_pico.c, by way of joba-1/PicoW_A2DP.
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

#ifndef BTSTACK_AUDIO_PICO_I2S_H
#define BTSTACK_AUDIO_PICO_I2S_H

#include "btstack_audio.h"

const btstack_audio_sink_t * btstack_audio_pico_sink_get_instance(void);

/// Audio held between the sink accepting a frame and the DAC clocking it out,
/// for AVDTP delay reporting. Tracks the buffer sizing, so it stays honest if
/// PICO_AUDIO_I2S_NUM_BUFFERS or _BUFFER_FRAMES change.
uint32_t btstack_audio_pico_sink_latency_us(uint32_t sample_rate);

#endif
