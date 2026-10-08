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

#include <stdbool.h>
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
/// re-acquire. Idempotent: once running, sample_rate is ignored and
/// audio_out_set_sample_rate() is the way to change it. False, claiming
/// nothing, if audio_out_set_sample_rate() would refuse sample_rate.
bool audio_out_init(uint32_t sample_rate);

/// Reprograms the clocks for a new rate. Only between streams - the contract
/// is audio_out_stop(), then this, then audio_out_start() - because audio
/// already handed over was made for the old rate. Whatever is still queued is
/// discarded; up to two buffers already loaded into the DMA play out at the
/// new rate.
///
/// The PIO divider is 16.8 fixed point, so the achieved rate is close to, not
/// exactly, the one asked for; it is printed with its error in ppm. True
/// without touching anything if the rate is already the current one. False,
/// changing nothing, if a stream is running, audio_out_init() has not run, the
/// rate is above PICO_AUDIO_I2S_MAX_SAMPLE_RATE, or the divider is out of
/// range.
bool audio_out_set_sample_rate(uint32_t sample_rate);

/// The rate last programmed, 0 before audio_out_init().
uint32_t audio_out_sample_rate(void);

/// 0 .. AUDIO_OUT_UNITY_GAIN. Applied to the 32 bit frames, so attenuating
/// does not cost resolution the way scaling 16 bit samples first would.
void audio_out_set_volume(int32_t gain);

void audio_out_start(audio_out_fill_fn fill, void * context);
void audio_out_stop(void);

/// Refills whatever the DMA has consumed. Call at least twice per buffer
/// period; audio_out_service_interval_ms() says how often that is.
void audio_out_service(void);

/// Frames in each buffer at the current rate. Buffers are sized in time
/// (PICO_AUDIO_I2S_BUFFER_US), so this follows the rate while the buffer
/// period stays put.
uint32_t audio_out_frames_per_buffer(void);

/// The longest a caller may leave between audio_out_service() calls at the
/// current rate: half a buffer period, at least 1ms.
uint32_t audio_out_service_interval_ms(void);

/// How long one buffer plays at sample_rate. Buffers are sized in time, so this
/// is PICO_AUDIO_I2S_BUFFER_US to within a frame at any rate.
uint32_t audio_out_buffer_us(uint32_t sample_rate);

/// Frames the DAC will play before anything handed over now: what is left of
/// the buffer playing, plus every buffer queued behind it, silence included.
/// The output's share of the latency, as it stands.
uint32_t audio_out_queued_frames(void);

/// Times the DMA found nothing queued and played silence. Non-zero means the
/// source is not keeping up; zero means a gap came from somewhere else.
uint32_t audio_out_underruns(void);

/// Times the refill IRQ came a whole buffer late, so a buffer of silence played
/// instead of the next one queued. Interrupts held off that long - a flash
/// write, typically - and not the source.
uint32_t audio_out_late_irqs(void);

#endif
