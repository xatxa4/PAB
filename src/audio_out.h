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

#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include <stdint.h>

/*
 * Source-agnostic 32 bit I2S output.
 *
 * Drives the i2s_data (and optionally i2s_mclk) programs from
 * pico_i2s_pio/i2s.pio with a pair of chained DMA channels, so the hardware
 * moves from one buffer to the next without the CPU and a late interrupt costs
 * nothing until a whole buffer has played.
 *
 * Nothing here knows about Bluetooth. A source registers a fill callback and
 * calls audio_out_service() regularly from whatever loop it owns.
 */

#define AUDIO_OUT_UNITY_GAIN  65536

/// Fill dst with num_frames interleaved stereo frames, left aligned in 32 bits.
/// Called from whatever context calls audio_out_service(), never from the DMA
/// IRQ. Fill the whole buffer - it is reused between calls, so anything left
/// unwritten is the previous block's audio played a second time.
typedef void (*audio_out_fill_fn)(int32_t * dst, uint32_t num_frames, void * context);

/// Claims the PIO state machines and DMA channels and starts the clocks. From
/// here on the DAC is fed continuously, silence included, so it never has to
/// re-acquire. Idempotent.
void audio_out_init(uint32_t sample_rate);

void audio_out_set_sample_rate(uint32_t sample_rate);

/// 0 .. AUDIO_OUT_UNITY_GAIN. Applied to the 32 bit frames, so attenuating
/// does not cost resolution the way scaling 16 bit samples first would.
void audio_out_set_volume(int32_t gain);

void audio_out_start(audio_out_fill_fn fill, void * context);
void audio_out_stop(void);

/// Refills whatever the DMA has consumed. Call at least twice per buffer
/// period; audio_out_frames_per_buffer() says how long that is.
void audio_out_service(void);

uint32_t audio_out_frames_per_buffer(void);

/// Audio held between accepting a frame and the DAC clocking it out.
uint32_t audio_out_latency_us(uint32_t sample_rate);

/// Times the DMA found nothing queued and played silence. Non-zero means the
/// source is not keeping up; zero means a gap came from somewhere else.
uint32_t audio_out_underruns(void);

#endif
