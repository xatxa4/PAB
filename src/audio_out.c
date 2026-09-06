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
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

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

void audio_out_set_sample_rate(uint32_t sample_rate){
    uint32_t sys = clock_get_hz(clk_sys);

    // i2s_data spends 128 cycles per stereo frame, so BCLK comes out at 64fs
    pio_sm_set_clkdiv(PICO_AUDIO_I2S_PIO, i2s_sm, (float) sys / (float) (sample_rate * 128));

#if I2S_HAVE_MCLK
    // MCLK is fixed at 22.5792 / 24.576 MHz; i2s_mclk halves its state machine clock
    float mclk_sm_hz = (sample_rate % 48000 == 0) ? 49.152e6f : 45.1584e6f;
    pio_sm_set_clkdiv(PICO_AUDIO_I2S_PIO, i2s_mclk_sm, (float) sys / mclk_sm_hz);
#endif
}

static void i2s_start(uint32_t sample_rate){
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

    audio_out_set_sample_rate(sample_rate);

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
    irq_set_enabled(DMA_IRQ_0, true);

    // clocks run from here on, feeding silence whenever nothing is streaming,
    // so the DAC never has to re-acquire between tracks
    pio_sm_set_enabled(pio, i2s_sm, true);
    dma_channel_start(i2s_dma_chan[0]);
}


void audio_out_init(uint32_t sample_rate){
    if (i2s_started) return;
    i2s_start(sample_rate);
    i2s_started = true;
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
