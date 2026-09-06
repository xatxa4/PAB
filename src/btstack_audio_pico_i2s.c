/*
 * btstack_audio_pico_i2s.c
 *
 * btstack_audio_sink_t on top of audio_out. All this does is own the run loop
 * timer that services the output, and widen the 16 bit PCM the A2DP pipeline
 * produces into the 32 bit frames the DAC wants. The hardware lives in
 * audio_out.c, which knows nothing about Bluetooth.
 */

#define BTSTACK_FILE__ "btstack_audio_pico_i2s.c"

#include "btstack_audio_pico_i2s.h"
#include "audio_out.h"

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_run_loop.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define DRIVER_POLL_INTERVAL_MS   5

static void (*playback_callback)(int16_t * buffer, uint16_t num_frames);

static btstack_timer_source_t driver_timer_sink;
static bool     sink_active;
static uint8_t  sink_channel_count;
static uint32_t sink_sample_rate;

// scratch for the 16 bit samples coming out of the A2DP pipeline
static int16_t render_pcm[1024 * 2];

static void sink_fill(int32_t * dst, uint32_t num_frames, void * context){
    UNUSED(context);
    btstack_assert(num_frames <= (sizeof(render_pcm) / sizeof(render_pcm[0])) / 2);

    (*playback_callback)(render_pcm, (uint16_t) num_frames);

    // shift as unsigned; negative operands make the signed shift undefined
    if (sink_channel_count == 1){
        for (uint32_t f = 0; f < num_frames; f++){
            int32_t sample = (int32_t) ((uint32_t) render_pcm[f] << 16);
            dst[2 * f    ] = sample;
            dst[2 * f + 1] = sample;
        }
    } else {
        for (uint32_t w = 0; w < num_frames * 2; w++){
            dst[w] = (int32_t) ((uint32_t) render_pcm[w] << 16);
        }
    }
}

static void driver_timer_handler_sink(btstack_timer_source_t * ts){

    audio_out_service();

    // says whether a gap was ours: the DMA ran out of audio and played silence.
    // Throttled to once a second so reporting cannot make the problem worse.
    static uint32_t reported;
    static uint16_t ticks;
    if (++ticks >= 1000 / DRIVER_POLL_INTERVAL_MS){
        ticks = 0;
        uint32_t underruns = audio_out_underruns();
        if (underruns != reported){
            reported = underruns;
            printf("I2S             : %lu underruns\n", (unsigned long) reported);
        }
    }

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

    audio_out_init(samplerate);
    if (samplerate != sink_sample_rate){
        audio_out_set_sample_rate(samplerate);
        sink_sample_rate = samplerate;
    }

    return 0;
}

static void btstack_audio_pico_sink_set_volume(uint8_t volume){
    if (volume > 127) volume = 127;
    // AVRCP 0..127 mapped so 127 is unity and 0 lands where it always has
    audio_out_set_volume(((int32_t) volume + 1) * 512);
}

static void btstack_audio_pico_sink_start_stream(void){

    sink_active = true;
    audio_out_start(&sink_fill, NULL);

    btstack_run_loop_set_timer_handler(&driver_timer_sink, &driver_timer_handler_sink);
    btstack_run_loop_set_timer(&driver_timer_sink, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_sink);
}

static void btstack_audio_pico_sink_stop_stream(void){

    btstack_run_loop_remove_timer(&driver_timer_sink);
    audio_out_stop();
    sink_active = false;
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

uint32_t btstack_audio_pico_sink_latency_us(uint32_t sample_rate){
    return audio_out_latency_us(sample_rate);
}
