/*
 * btstack_audio_pico_i2s.c
 *
 * Implementation of btstack_audio.h on top of pico-i2s-pio.
 *
 * The A2DP pipeline hands us 16 bit stereo PCM. pico-i2s-pio sends 32 bit
 * frames with BCLK = 64fs and a free running MCLK (22.5792 / 24.576 MHz),
 * which is what DACs such as the ES9038Q2M expect.
 *
 * Core0 (the BTstack run loop) renders PCM into the pico-i2s-pio sample queue
 * on a timer; core1 drains that queue into the PIO via DMA. The DMA handoff is
 * blocking, so it needs a core of its own.
 */

#define BTSTACK_FILE__ "btstack_audio_pico_i2s.c"

#include "btstack_audio_pico_i2s.h"

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_run_loop.h"

#include <stddef.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "i2s.h"

#ifndef PICO_AUDIO_I2S_DATA_PIN
#define PICO_AUDIO_I2S_DATA_PIN         9
#endif

// LRCLK/WS, BCLK is PICO_AUDIO_I2S_CLOCK_PIN_BASE + 1
#ifndef PICO_AUDIO_I2S_CLOCK_PIN_BASE
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE   10
#endif

#ifndef PICO_AUDIO_I2S_MCLK_PIN
#define PICO_AUDIO_I2S_MCLK_PIN         12
#endif

// The CYW43 driver claims a state machine on the first PIO with room, so leave
// pio0 to it and keep I2S out of the way.
#ifndef PICO_AUDIO_I2S_PIO
#define PICO_AUDIO_I2S_PIO              pio1
#endif

#ifndef PICO_AUDIO_I2S_CLOCK_MODE
#define PICO_AUDIO_I2S_CLOCK_MODE       CLOCK_MODE_DEFAULT
#endif

#ifndef PICO_AUDIO_I2S_FORMAT
#define PICO_AUDIO_I2S_FORMAT           MODE_I2S
#endif

// Clocks start at boot, before A2DP negotiates a rate, so the DAC can lock early
#ifndef PICO_AUDIO_I2S_INITIAL_SAMPLE_RATE
#define PICO_AUDIO_I2S_INITIAL_SAMPLE_RATE  44100
#endif

#define DRIVER_POLL_INTERVAL_MS   5

// how much audio to keep queued ahead of the DAC
#define TARGET_LATENCY_MS         20

// frames rendered per playback_callback() call
#define RENDER_FRAMES_MAX         128

// frames per DMA transfer: 0.5ms at 48kHz, rounded up
#define PUMP_FRAMES_MAX           32

static void (*playback_callback)(int16_t * buffer, uint16_t num_frames);

static btstack_timer_source_t driver_timer_sink;
static bool     sink_active;
static bool     i2s_running;
static uint8_t  sink_channel_count;
static uint32_t sink_sample_rate = PICO_AUDIO_I2S_INITIAL_SAMPLE_RATE;

// touched only from the run loop on core0
static int16_t render_pcm[RENDER_FRAMES_MAX * 2];
static int32_t render_left[RENDER_FRAMES_MAX];
static int32_t render_right[RENDER_FRAMES_MAX];

// touched only from core1
static int32_t pump_dma_a[2][PUMP_FRAMES_MAX * 2];
static int32_t pump_dma_b[2][PUMP_FRAMES_MAX * 2];

static void i2s_pump_core1(void){
    int32_t buf_l[PUMP_FRAMES_MAX];
    int32_t buf_r[PUMP_FRAMES_MAX];
    uint8_t page = 0;
    bool    muted = true;

    while (true){
        int chunk = (int) (i2s_get_sample_rate_hz() / 2000);
        if (chunk > PUMP_FRAMES_MAX) chunk = PUMP_FRAMES_MAX;

        // wait for a small cushion before unmuting, so a slow start does not
        // turn into a stutter of alternating audio and silence
        int queued = i2s_get_queue_length();
        if (muted){
            if (queued >= chunk * 3) muted = false;
        } else if (queued == 0){
            muted = true;
        }

        int frames = 0;
        if (!muted){
            frames = i2s_dequeue(buf_l, buf_r, chunk);
            if (frames < chunk) muted = true;
        }

        // pad with silence rather than shortening the transfer: MCLK/BCLK/LRCLK
        // stay running between tracks and the DAC keeps its lock
        memset(&buf_l[frames], 0, (chunk - frames) * sizeof(int32_t));
        memset(&buf_r[frames], 0, (chunk - frames) * sizeof(int32_t));

        int words = i2s_format_piodata(buf_l, buf_r, chunk,
                                       (uint32_t *) pump_dma_a[page],
                                       (uint32_t *) pump_dma_b[page]);
        i2s_dma_transfer_blocking(pump_dma_a[page], pump_dma_b[page], words);
        page ^= 1;
    }
}

void btstack_audio_pico_i2s_begin(void){
    if (i2s_running) return;

    i2s_set_config(PICO_AUDIO_I2S_PIO, PICO_AUDIO_I2S_CLOCK_MODE, PICO_AUDIO_I2S_FORMAT);
    i2s_set_pin(PICO_AUDIO_I2S_DATA_PIN, PICO_AUDIO_I2S_CLOCK_PIN_BASE, PICO_AUDIO_I2S_MCLK_PIN);
    i2s_init(sink_sample_rate);

    multicore_launch_core1(i2s_pump_core1);
    i2s_running = true;
}

static void btstack_audio_pico_sink_fill_buffers(void){
    int target = (int) (sink_sample_rate / 1000 * TARGET_LATENCY_MS);
    if (target > I2S_QUEUE_MAX - 1) target = I2S_QUEUE_MAX - 1;

    while (true){
        int frames = target - i2s_get_queue_length();
        if (frames <= 0) break;
        if (frames > RENDER_FRAMES_MAX) frames = RENDER_FRAMES_MAX;

        (*playback_callback)(render_pcm, frames);

        // 16 bit PCM sits in the top half of the 32 bit I2S frame; shift as
        // unsigned, negative operands make the signed shift undefined
        if (sink_channel_count == 1){
            for (int i = 0; i < frames; i++){
                render_left[i]  = (int32_t) ((uint32_t) render_pcm[i] << 16);
                render_right[i] = render_left[i];
            }
        } else {
            for (int i = 0; i < frames; i++){
                render_left[i]  = (int32_t) ((uint32_t) render_pcm[2 * i    ] << 16);
                render_right[i] = (int32_t) ((uint32_t) render_pcm[2 * i + 1] << 16);
            }
        }

        if (!i2s_enqueue(render_left, render_right, frames)) break;
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

    btstack_audio_pico_i2s_begin();

    if (samplerate != sink_sample_rate){
        sink_sample_rate = samplerate;
        i2s_change_clock(samplerate);
    }

    return 0;
}

static void btstack_audio_pico_sink_set_volume(uint8_t volume){
    // AVRCP volume is applied to the decoded PCM in a2dp.c
    UNUSED(volume);
}

static void btstack_audio_pico_sink_start_stream(void){

    sink_active = true;

    // pre-fill the I2S queue
    btstack_audio_pico_sink_fill_buffers();

    btstack_run_loop_set_timer_handler(&driver_timer_sink, &driver_timer_handler_sink);
    btstack_run_loop_set_timer(&driver_timer_sink, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_sink);
}

static void btstack_audio_pico_sink_stop_stream(void){

    btstack_run_loop_remove_timer(&driver_timer_sink);
    sink_active = false;

    // core1 drains what is left and falls back to silence, keeping the clocks up
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
