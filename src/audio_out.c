/*
 * Copyright (c) 2025 BambooMaster <https://github.com/BambooMaster/pico-i2s-pio>
 * Copyright (c) 2026 xatxa4 <https://github.com/xatxa4/PAB>
 *
 * PIO setup, clock dividers and pin mapping follow BambooMaster's
 * pico-i2s-pio i2s_core.c; the state machine programs are its i2s.pio. The
 * chained DMA and buffer handling are new.
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

/*
 * audio_out.c - 32 bit I2S output, no knowledge of what produces the audio.
 *
 * Signal generation is what pico-i2s-pio does in CLOCK_MODE_DEFAULT and
 * MODE_I2S: the same i2s_data and i2s_mclk programs from pico_i2s_pio/i2s.pio,
 * the same pin order, the same clock dividers. Only the way the PIO is fed
 * differs - a pair of chained DMA channels walks a ring of buffers, and their
 * completion IRQ reloads whichever has just finished.
 */

#include "audio_out.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/bus_ctrl.h"   // the name SDK 1.5 and 2.x share

#include "i2s.pio.h"

#ifndef PICO_AUDIO_I2S_DATA_PIN
#define PICO_AUDIO_I2S_DATA_PIN         18
#endif

// LRCLK/WS, BCLK is PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1
#ifndef PICO_AUDIO_I2S_CLOCK_PIN_BASE
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE   20
#endif

// Off by default. usb_sound_card_hires drives MCLK, but it has no radio; joba-1
// has a radio, but pico-extras never generates MCLK. This build is the only one
// that would do both, and a 22.5792MHz square wave switching IOVDD next to the
// CYW43's SPI pins (GP23-25) is a poor neighbour for the Bluetooth link.
// Set to a pin number only if the DAC actually needs a master clock.
#ifndef PICO_AUDIO_I2S_MCLK_PIN
#define PICO_AUDIO_I2S_MCLK_PIN         -1
#endif

#define I2S_HAVE_MCLK   (PICO_AUDIO_I2S_MCLK_PIN >= 0)

// The CYW43 driver claims a state machine on the first PIO with room, so leave
// pio0 to it and keep I2S out of the way.
#ifndef PICO_AUDIO_I2S_PIO
#define PICO_AUDIO_I2S_PIO              pio1
#endif

// Audio queued ahead of the DAC. 4 x 512 frames is ~46ms at 44.1kHz, of which
// one buffer is playing and one is loaded in the chained channel, so the run
// loop has two to refill. Shrinking this cuts latency.
#ifndef PICO_AUDIO_I2S_NUM_BUFFERS
#define PICO_AUDIO_I2S_NUM_BUFFERS      4
#endif

#ifndef PICO_AUDIO_I2S_BUFFER_FRAMES
#define PICO_AUDIO_I2S_BUFFER_FRAMES    512
#endif

#define BUFFER_WORDS              (PICO_AUDIO_I2S_BUFFER_FRAMES * 2)

static audio_out_fill_fn fill_callback;
static void *            fill_context;
static bool              i2s_started;
static uint32_t          current_rate;

static uint i2s_sm;
#if I2S_HAVE_MCLK
static uint i2s_mclk_sm;
#endif

// Two channels chained to each other. When one finishes the other is started by
// the hardware, so a late IRQ costs nothing until a whole buffer has played -
// with a single channel the CPU had to re-arm it within the 8 word PIO FIFO,
// about 90us at 44.1kHz, and anything slower than that punched a hole in the
// output no matter how much audio was buffered behind it.
static int i2s_dma_chan[2];
static volatile int8_t chan_buffer[2] = { -1, -1 };   // -1 while playing silence

static volatile uint32_t underrun_count;

// 32 bit stereo frames, interleaved left/right, as the PIO pulls them
static int32_t audio_buffer[PICO_AUDIO_I2S_NUM_BUFFERS][BUFFER_WORDS];
static int32_t silence_buffer[BUFFER_WORDS];

// buffer_ready means "holds audio", and stays set for as long as the DMA is
// reading it - it is cleared only once the transfer has finished. The run loop
// refills anything not marked ready, so it can never write a buffer that is
// queued or in flight, and the IRQ never picks one that is being written.
static volatile bool    buffer_ready[PICO_AUDIO_I2S_NUM_BUFFERS];
static volatile uint8_t next_buffer;

static volatile int32_t volume_gain = AUDIO_OUT_UNITY_GAIN;

static void __time_critical_func(i2s_dma_handler)(void){
    for (uint8_t c = 0; c < 2; c++){
        if ((dma_hw->ints0 & (1u << i2s_dma_chan[c])) == 0) continue;
        dma_hw->ints0 = 1u << i2s_dma_chan[c];

        // this channel is done, so the buffer it just played is free again.
        // The other channel is already playing, started by the chain.
        if (chan_buffer[c] >= 0){
            buffer_ready[chan_buffer[c]] = false;
        }

        // load the buffer that comes after the one now playing, ready for the
        // chain to pick up. On underrun leave next_buffer alone so the queued
        // buffers still play in order.
        uint8_t i = next_buffer;
        const int32_t * src;
        if (buffer_ready[i]){
            chan_buffer[c] = (int8_t) i;
            src = audio_buffer[i];
            if (++i == PICO_AUDIO_I2S_NUM_BUFFERS) i = 0;
            next_buffer = i;
        } else {
            chan_buffer[c] = -1;
            src = silence_buffer;
            underrun_count++;
        }

        dma_channel_set_read_addr(i2s_dma_chan[c], src, false);
        dma_channel_set_trans_count(i2s_dma_chan[c], BUFFER_WORDS, false);
    }
}

// i2s_data spends 128 cycles per stereo frame, so BCLK comes out at 64fs and
// the divider is clk_sys / (128 * rate). The PIO takes it in 16.8 fixed point,
// which in 1/256ths is 2 * clk_sys / rate. Rounded to nearest: the float
// pio_sm_set_clkdiv() truncates, which put 44.1kHz at +165ppm instead of -12.
// Returns 0 if the divider is out of range - below 1, or past the 16 bit
// integer part.
static uint32_t data_clkdiv_fp8(uint32_t sample_rate){
    if (sample_rate == 0) return 0;
    uint64_t div = (2ull * clock_get_hz(clk_sys) + sample_rate / 2) / sample_rate;
    return (div >= 0x100 && div <= 0xffffff) ? (uint32_t) div : 0;
}

static void program_sample_rate(uint32_t sample_rate, uint32_t div_fp8){
    uint32_t sys = clock_get_hz(clk_sys);

    pio_sm_set_clkdiv_int_frac(PICO_AUDIO_I2S_PIO, i2s_sm, (uint16_t) (div_fp8 >> 8), (uint8_t) div_fp8);

#if I2S_HAVE_MCLK
    // MCLK is fixed at 22.5792 / 24.576 MHz; i2s_mclk halves its state machine clock
    float mclk_sm_hz = (sample_rate % 48000 == 0) ? 49.152e6f : 45.1584e6f;
    pio_sm_set_clkdiv(PICO_AUDIO_I2S_PIO, i2s_mclk_sm, (float) sys / mclk_sm_hz);
#endif

    current_rate = sample_rate;

    // worth knowing rather than discovering by ear
    uint64_t achieved_mhz = (2000ull * sys + div_fp8 / 2) / div_fp8;
    int32_t  error_ppm    = (int32_t) (((int64_t) achieved_mhz - (int64_t) sample_rate * 1000) * 1000 / sample_rate);
    printf("I2S             : %lu Hz, divider %lu+%lu/256, %+ld ppm\n",
           (unsigned long) sample_rate, (unsigned long) (div_fp8 >> 8),
           (unsigned long) (div_fp8 & 0xff), (long) error_ppm);
}

// Drops audio queued for the DMA but not yet loaded into a channel, so none of
// it plays at a rate it was not made for. What is loaded or playing - at most
// two buffers - finishes on its own.
static void discard_queued(void){
    uint32_t saved = save_and_disable_interrupts();
    for (uint8_t i = 0; i < PICO_AUDIO_I2S_NUM_BUFFERS; i++){
        if (i != chan_buffer[0] && i != chan_buffer[1]) buffer_ready[i] = false;
    }
    restore_interrupts(saved);
}

bool audio_out_set_sample_rate(uint32_t sample_rate){
    if (!i2s_started) return false;

    uint32_t div_fp8 = data_clkdiv_fp8(sample_rate);
    if (div_fp8 == 0) return false;
    if (sample_rate == current_rate) return true;
    if (fill_callback != NULL) return false;    // mid-stream: stop first

    discard_queued();
    program_sample_rate(sample_rate, div_fp8);
    return true;
}

uint32_t audio_out_sample_rate(void){
    return current_rate;
}

static void i2s_start(uint32_t sample_rate, uint32_t div_fp8){
    PIO pio = PICO_AUDIO_I2S_PIO;

    i2s_sm = pio_claim_unused_sm(pio, true);

    pio_gpio_init(pio, PICO_AUDIO_I2S_DATA_PIN);
    pio_gpio_init(pio, PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    pio_gpio_init(pio, PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1);

#if I2S_HAVE_MCLK
    i2s_mclk_sm = pio_claim_unused_sm(pio, true);
    pio_gpio_init(pio, PICO_AUDIO_I2S_MCLK_PIN);
    pio_sm_set_consecutive_pindirs(pio, i2s_mclk_sm, PICO_AUDIO_I2S_MCLK_PIN, 1, true);
    uint offset_mclk = pio_add_program(pio, &i2s_mclk_program);
    pio_sm_config cm = i2s_mclk_program_get_default_config(offset_mclk);
    sm_config_set_set_pins(&cm, PICO_AUDIO_I2S_MCLK_PIN, 1);
    pio_sm_init(pio, i2s_mclk_sm, offset_mclk, &cm);
    pio_sm_set_enabled(pio, i2s_mclk_sm, true);
#endif

    uint offset = pio_add_program(pio, &i2s_data_program);
    pio_sm_config c = i2s_data_program_get_default_config(offset);
    sm_config_set_out_pins(&c, PICO_AUDIO_I2S_DATA_PIN, 1);
    sm_config_set_sideset_pins(&c, PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    sm_config_set_out_shift(&c, false, false, 32);   // shift left, MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_sm, offset, &c);

    uint32_t pin_mask = (1u << PICO_AUDIO_I2S_DATA_PIN) | (3u << PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    pio_sm_set_pindirs_with_mask(pio, i2s_sm, pin_mask, pin_mask);
    pio_sm_exec(pio, i2s_sm, pio_encode_jmp(offset));
    pio_sm_set_pins(pio, i2s_sm, 0);
    pio_sm_clear_fifos(pio, i2s_sm);

    program_sample_rate(sample_rate, div_fp8);

    i2s_dma_chan[0] = dma_claim_unused_channel(true);
    i2s_dma_chan[1] = dma_claim_unused_channel(true);

    for (uint8_t c = 0; c < 2; c++){
        dma_channel_config dc = dma_channel_get_default_config(i2s_dma_chan[c]);
        channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
        channel_config_set_read_increment(&dc, true);
        channel_config_set_write_increment(&dc, false);
        channel_config_set_dreq(&dc, pio_get_dreq(pio, i2s_sm, true));
        channel_config_set_chain_to(&dc, i2s_dma_chan[c ^ 1]);
        dma_channel_configure(i2s_dma_chan[c], &dc, &pio->txf[i2s_sm],
                              silence_buffer, BUFFER_WORDS, false);
        dma_channel_set_irq0_enabled(i2s_dma_chan[c], true);
    }

    irq_add_shared_handler(DMA_IRQ_0, i2s_dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    // At the default priority the refill ranks equal with the CYW43 GPIO IRQ
    // and waits behind it at a buffer boundary. Nothing else uses DMA_IRQ_0 -
    // the CYW43 bus polls its DMA - so anything sharing it later must be as
    // short as this handler.
    irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    // DMA ahead of the processors on the bus, so a busy core cannot hold off
    // the transfers feeding the PIO FIFO
    hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS);

    // clocks run from here on, feeding silence whenever nothing is streaming,
    // so the DAC never has to re-acquire between tracks
    pio_sm_set_enabled(pio, i2s_sm, true);
    dma_channel_start(i2s_dma_chan[0]);
}


bool audio_out_init(uint32_t sample_rate){
    if (i2s_started) return true;

    uint32_t div_fp8 = data_clkdiv_fp8(sample_rate);
    if (div_fp8 == 0) return false;

    i2s_start(sample_rate, div_fp8);
    i2s_started = true;
    return true;
}

void audio_out_set_volume(int32_t gain){
    if (gain < 0) gain = 0;
    if (gain > AUDIO_OUT_UNITY_GAIN) gain = AUDIO_OUT_UNITY_GAIN;
    volume_gain = gain;
}

void audio_out_start(audio_out_fill_fn fill, void * context){
    fill_context  = context;
    fill_callback = fill;
    audio_out_service();
}

void audio_out_stop(void){
    fill_callback = NULL;
    // the DMA drains what is queued and falls back to silence on its own
}

void audio_out_service(void){
    audio_out_fill_fn fill = fill_callback;
    if (fill == NULL) return;

    // start at the buffer the IRQ wants next, so it is never left behind
    uint8_t i = next_buffer;

    for (uint8_t n = 0; n < PICO_AUDIO_I2S_NUM_BUFFERS;
         n++, i = (i + 1 == PICO_AUDIO_I2S_NUM_BUFFERS) ? 0 : i + 1){

        if (buffer_ready[i]) continue;

        int32_t * dst = audio_buffer[i];
        fill(dst, PICO_AUDIO_I2S_BUFFER_FRAMES, fill_context);

        int32_t gain = volume_gain;
        if (gain != AUDIO_OUT_UNITY_GAIN){
            for (int w = 0; w < BUFFER_WORDS; w++){
                dst[w] = (int32_t) (((int64_t) dst[w] * gain) >> 16);
            }
        }

        buffer_ready[i] = true;
    }
}

uint32_t audio_out_frames_per_buffer(void){
    return PICO_AUDIO_I2S_BUFFER_FRAMES;
}

uint32_t audio_out_latency_us(uint32_t sample_rate){
    if (sample_rate == 0) return 0;
    uint32_t frames = PICO_AUDIO_I2S_NUM_BUFFERS * PICO_AUDIO_I2S_BUFFER_FRAMES;
    return (uint32_t) (((uint64_t) frames * 1000000u) / sample_rate);
}

uint32_t audio_out_underruns(void){
    return underrun_count;
}
