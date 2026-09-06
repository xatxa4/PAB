/*
 * btstack_audio_pico_i2s.c
 *
 * Implementation of btstack_audio.h driving I2S from PIO directly.
 *
 * The A2DP pipeline hands us 16 bit stereo PCM, which goes out as 32 bit I2S
 * frames with BCLK = 64fs. DACs that ignore the lower bits (the Pimoroni line
 * out pack) are unaffected; DACs that expect a full 64 BCLK frame, such as the
 * ES9038Q2M, need it.
 *
 * Everything runs on core0: a DMA channel feeds the PIO from a small ring of
 * buffers, its completion IRQ hands the next buffer over, and the BTstack run
 * loop refills whatever has been consumed. No second core, so nothing gets in
 * the way of the flash writes BTstack uses to store link keys.
 */

#define BTSTACK_FILE__ "btstack_audio_pico_i2s.c"

#include "btstack_audio_pico_i2s.h"

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_run_loop.h"

#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "i2s_out.pio.h"

#ifndef PICO_AUDIO_I2S_DATA_PIN
#define PICO_AUDIO_I2S_DATA_PIN         9
#endif

// BCLK, LRCLK is PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1
#ifndef PICO_AUDIO_I2S_CLOCK_PIN_BASE
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE   10
#endif

// The CYW43 driver claims a state machine on the first PIO with room, so leave
// pio0 to it and keep I2S out of the way.
#ifndef PICO_AUDIO_I2S_PIO
#define PICO_AUDIO_I2S_PIO              pio1
#endif

#define DRIVER_POLL_INTERVAL_MS   5

// 3 x 512 frames is ~35ms of audio at 44.1kHz, refilled every 5ms. Same depth
// the pico-extras driver used, so the A2DP resampler sees what it used to.
#define NUM_BUFFERS               3
#define BUFFER_FRAMES             512
#define BUFFER_WORDS              (BUFFER_FRAMES * 2)

static void (*playback_callback)(int16_t * buffer, uint16_t num_frames);

static btstack_timer_source_t driver_timer_sink;
static bool     sink_active;
static bool     i2s_started;
static uint8_t  sink_channel_count;
static uint32_t sink_sample_rate;

static uint i2s_sm;
static int  i2s_dma_chan;

// 32 bit stereo frames, interleaved left/right, as the PIO consumes them
static int32_t audio_buffer[NUM_BUFFERS][BUFFER_WORDS];
static int32_t silence_buffer[BUFFER_WORDS];

// buffer_ready is set only by the run loop and cleared only by the DMA IRQ, so
// a buffer is never written while the DMA is reading it
static volatile bool    buffer_ready[NUM_BUFFERS];
static volatile uint8_t next_buffer;

// scratch for the 16 bit samples coming out of the A2DP pipeline
static int16_t render_pcm[BUFFER_FRAMES * 2];

// writing the read address trigger alias also reloads the transfer count
static void __time_critical_func(i2s_dma_handler)(void){
    dma_hw->ints0 = 1u << i2s_dma_chan;

    // on underrun keep next_buffer where it is, so buffers still play in order
    const int32_t * next = silence_buffer;
    uint8_t i = next_buffer;
    if (buffer_ready[i]){
        next = audio_buffer[i];
        buffer_ready[i] = false;
        if (++i == NUM_BUFFERS) i = 0;
        next_buffer = i;
    }
    dma_channel_set_read_addr(i2s_dma_chan, next, true);
}

static void i2s_set_sample_rate(uint32_t sample_rate){
    // the PIO program spends 128 cycles per stereo frame, so BCLK is 64fs
    float div = (float) clock_get_hz(clk_sys) / (float) (sample_rate * 128);
    pio_sm_set_clkdiv(PICO_AUDIO_I2S_PIO, i2s_sm, div);
    pio_sm_clkdiv_restart(PICO_AUDIO_I2S_PIO, i2s_sm);
}

static void i2s_start(uint32_t sample_rate){
    PIO pio = PICO_AUDIO_I2S_PIO;

    i2s_sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &i2s_out_32_program);

    pio_gpio_init(pio, PICO_AUDIO_I2S_DATA_PIN);
    pio_gpio_init(pio, PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    pio_gpio_init(pio, PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1);

    pio_sm_config c = i2s_out_32_program_get_default_config(offset);
    sm_config_set_out_pins(&c, PICO_AUDIO_I2S_DATA_PIN, 1);
    sm_config_set_sideset_pins(&c, PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    sm_config_set_out_shift(&c, false, false, 32);   // shift left, MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, i2s_sm, offset, &c);

    uint32_t pin_mask = (1u << PICO_AUDIO_I2S_DATA_PIN) | (3u << PICO_AUDIO_I2S_CLOCK_PIN_BASE);
    pio_sm_set_pindirs_with_mask(pio, i2s_sm, pin_mask, pin_mask);
    pio_sm_set_pins(pio, i2s_sm, 0);

    i2s_set_sample_rate(sample_rate);

    i2s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dc = dma_channel_get_default_config(i2s_dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, i2s_sm, true));
    dma_channel_configure(i2s_dma_chan, &dc, &pio->txf[i2s_sm],
                          silence_buffer, BUFFER_WORDS, false);

    dma_channel_set_irq0_enabled(i2s_dma_chan, true);
    irq_add_shared_handler(DMA_IRQ_0, i2s_dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    // clocks run from here on, feeding silence whenever nothing is streaming,
    // so the DAC never has to re-acquire between tracks
    pio_sm_set_enabled(pio, i2s_sm, true);
    dma_channel_start(i2s_dma_chan);
}

static void btstack_audio_pico_sink_fill_buffers(void){
    // start at the buffer the IRQ wants next, so it is never left behind
    uint8_t i = next_buffer;

    for (uint8_t n = 0; n < NUM_BUFFERS; n++, i = (i + 1 == NUM_BUFFERS) ? 0 : i + 1){
        if (buffer_ready[i]) continue;

        (*playback_callback)(render_pcm, BUFFER_FRAMES);

        // 16 bit PCM sits in the top half of the 32 bit I2S frame; shift as
        // unsigned, negative operands make the signed shift undefined
        int32_t * dst = audio_buffer[i];
        if (sink_channel_count == 1){
            for (int f = 0; f < BUFFER_FRAMES; f++){
                int32_t sample = (int32_t) ((uint32_t) render_pcm[f] << 16);
                dst[2 * f    ] = sample;
                dst[2 * f + 1] = sample;
            }
        } else {
            for (int w = 0; w < BUFFER_WORDS; w++){
                dst[w] = (int32_t) ((uint32_t) render_pcm[w] << 16);
            }
        }

        buffer_ready[i] = true;
    }
}

static void driver_timer_handler_sink(btstack_timer_source_t * ts){

    btstack_audio_pico_sink_fill_buffers();

    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

static int btstack_audio_pico_sink_init(
    uint8_t channels,
    uint32_t samplerate,
    void (*playback)(int16_t * buffer, uint16_t num_samples)
){
    btstack_assert(playback != NULL);
    btstack_assert(channels != 0);

    playback_callback  = playback;
    sink_channel_count = channels;

    if (!i2s_started){
        i2s_start(samplerate);
        sink_sample_rate = samplerate;
        i2s_started = true;
    } else if (samplerate != sink_sample_rate){
        i2s_set_sample_rate(samplerate);
        sink_sample_rate = samplerate;
    }

    return 0;
}

static void btstack_audio_pico_sink_set_volume(uint8_t volume){
    // AVRCP volume is applied to the decoded PCM in a2dp.c
    UNUSED(volume);
}

static void btstack_audio_pico_sink_start_stream(void){

    sink_active = true;

    btstack_audio_pico_sink_fill_buffers();

    btstack_run_loop_set_timer_handler(&driver_timer_sink, &driver_timer_handler_sink);
    btstack_run_loop_set_timer(&driver_timer_sink, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_sink);
}

static void btstack_audio_pico_sink_stop_stream(void){

    btstack_run_loop_remove_timer(&driver_timer_sink);
    sink_active = false;

    // the DMA drains what is queued and falls back to silence on its own
}

static void btstack_audio_pico_sink_close(void){
    if (sink_active){
        btstack_audio_pico_sink_stop_stream();
    }
}

static const btstack_audio_sink_t btstack_audio_pico_sink = {
    .init = &btstack_audio_pico_sink_init,
    .set_volume = &btstack_audio_pico_sink_set_volume,
    .start_stream = &btstack_audio_pico_sink_start_stream,
    .stop_stream = &btstack_audio_pico_sink_stop_stream,
    .close = &btstack_audio_pico_sink_close,
};

const btstack_audio_sink_t * btstack_audio_pico_sink_get_instance(void){
    return &btstack_audio_pico_sink;
}
