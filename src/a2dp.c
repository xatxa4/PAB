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

#include "a2dp.h"

#include <stdio.h>

#include <btstack.h>
#include <btstack_resample.h>
#include <classic/a2dp_sink.h>
#include "hardware/watchdog.h"

// for connection led 
#include <pico/cyw43_arch.h>

#include "btstack_audio_pico_i2s.h"


// Dominates end to end latency: each SBC frame is 128 samples, so 20..40 frames
// is ~60..120ms at 44.1kHz. joba-1 uses 60/120/30, which is safer but puts
// lip sync out by a third of a second.
#define OPTIMAL_FRAMES_MIN 20
#define OPTIMAL_FRAMES_MAX 40

// Playback starts once this many SBC frames are queued.
#define START_FRAMES       ((OPTIMAL_FRAMES_MIN + OPTIMAL_FRAMES_MAX) / 2)

// Room in the SBC ring above OPTIMAL_FRAMES_MAX for packets that arrive in a
// burst. A full ring refuses a whole packet, and a packet carries up to 15
// frames, so this must hold at least one maximal packet on top of the target
// depth. Costs RAM only - the resampler, not the ring size, sets the latency.
#define ADDITIONAL_FRAMES  30
#define MAX_FRAMES_PER_PACKET 15    // 4 bit field in the SBC media payload header
#define NUM_CHANNELS       2
#define BYTES_PER_FRAME    (2*NUM_CHANNELS)

// Highest bitpool _sbc_capabilities offers a source.
#define ADVERTISED_MAX_BITPOOL 53

// SBC frame length in bytes (A2DP spec 12.9): header and CRC, scale factors,
// then the audio bits rounded up to a whole byte.
#define SBC_FRAME_BYTES(channels, subbands, audio_bits) \
    (4 + (4 * (subbands) * (channels)) / 8 + ((audio_bits) + 7) / 8)

// The largest frame _sbc_capabilities lets a source send: dual channel, 8
// subbands, 16 blocks, where each channel spends its own bitpool. 224 bytes at
// bitpool 53. The 120 inherited from a2dp_sink_demo only covers joint stereo
// (119), so a dual channel stream - SBC XQ - overran the stack buffer in
// playback_handler. Derived, so raising ADVERTISED_MAX_BITPOOL grows it to match.
#define MAX_SBC_FRAME_SIZE SBC_FRAME_BYTES(2, 8, 16 * 2 * ADVERTISED_MAX_BITPOOL)

// AVDTP delay reports are in units of 100us. Android 9 discards anything not
// above 100ms and falls back to its own guess of about 200ms, which would put
// the sound a tenth of a second ahead of the picture. Our real latency often
// sits below 101ms, so the floor costs other sources up to ~20ms (44.1kHz) or
// ~25ms (48kHz) of sound-early - inside the 45ms where viewers notice - and
// ~50ms for the seconds it takes to rebuild after the SBC ring has run dry.
// Apple asks for at most 1000ms, and Android ignores more than that.
#define DELAY_REPORT_MIN_100US 1010
#define DELAY_REPORT_MAX_100US 10000

// A typical packet. Before any audio has arrived there is nothing better, and
// it moves the estimate by a frame or two at most.
#define ASSUMED_FRAMES_PER_PACKET 5

// A delay report is an AVDTP command of our own, so it waits for BTstack: the
// endpoint configured, and no other command of ours in flight. Apple asks for
// no more than one a second.
#define DELAY_REPORT_INTERVAL_MS  1000
#define DELAY_REPORT_RETRY_MS     10
#define DELAY_REPORT_ATTEMPTS     50

// While playing, the measured latency is averaged over DELAY_REPORT_INTERVAL_MS
// and re-reported when it has moved by this much (100us units). The resampler
// lets it wander across ~50ms with clock drift and after dropouts.
#define DELAY_REPORT_STEP_100US   50


typedef struct {
    uint8_t  reconfigure;
    uint8_t  num_channels;
    uint16_t sampling_frequency;
    uint8_t  block_length;
    uint8_t  subbands;
    uint8_t  min_bitpool_value;
    uint8_t  max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} sbc_configuration_t;


typedef enum {
    STREAM_STATE_CLOSED,
    STREAM_STATE_OPEN,
    STREAM_STATE_PLAYING,
    STREAM_STATE_PAUSED,
} stream_state_t;


// all configurations with bitpool 2-53 are supported
static const uint8_t _sbc_capabilities[] = {
    0xFF,  // (AVDTP_SBC_44100 << 4) | AVDTP_SBC_STEREO,
    0xFF,  // (AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS
    2, ADVERTISED_MAX_BITPOOL
};
uint8_t _seid = 0;
uint16_t _cid = 0;
static avdtp_stream_endpoint_t * _endpoint;
stream_state_t _stream_state = STREAM_STATE_CLOSED;
sbc_configuration_t _sbc_configuration = {0};
btstack_sbc_decoder_state_t _state = {0};
bool _media_initialized = false;
bool _audio_stream_started = false;
unsigned _sbc_frame_size = 0;
btstack_resample_t _resample_instance = {0};
btstack_ring_buffer_t _sbc_frame_ring_buffer = {0};
btstack_ring_buffer_t _decoded_audio_ring_buffer = {0};
uint8_t _sbc_frame_storage[(OPTIMAL_FRAMES_MAX + ADDITIONAL_FRAMES) * MAX_SBC_FRAME_SIZE] = {0};
_Static_assert(sizeof(_sbc_frame_storage) / MAX_SBC_FRAME_SIZE >= OPTIMAL_FRAMES_MAX + MAX_FRAMES_PER_PACKET,
               "SBC ring cannot take a full packet of the largest frames on top of the target depth");
// Room for two decoded frames. Reads from the SBC ring are sized by the newest
// packet's frame size, so after the source raises its bitpool one read can
// complete two of the older, smaller frames, and the second one's output has
// to wait here.
uint8_t _decoded_audio_storage[(2*128+16) * BYTES_PER_FRAME] = {0};
int16_t * _request_buffer = 0;
int _request_frames = 0;

// A ring buffer write that does not fit is refused whole, so each of these is
// audio thrown away. Together with audio_out_underruns() they tell "source too
// slow" apart from "source too fast, or buffer too small".
static uint32_t _sbc_frames_dropped;    // SBC ring full
static uint32_t _pcm_frames_dropped;    // decoded PCM ring full
static uint32_t _sbc_frames_rejected;   // larger than MAX_SBC_FRAME_SIZE

static btstack_timer_source_t _delay_timer;
static uint16_t _delay_pending;         // value waiting to go out, 0 if none
static uint16_t _delay_sent;            // last value sent for this configuration, 0 if none
static uint32_t _delay_sent_ms;
static bool     _delay_sent_once;       // on this connection, for the rate limit
static bool     _delay_rejected;        // the source refused one: send no more
static bool     _delay_reporting;       // the source switched Delay Reporting on

// latency measured while playing, in frames, summed over one window
static uint64_t _latency_sum;
static uint32_t _latency_count;
static uint32_t _latency_window_ms;
static bool     _latency_reported;      // since playback last started
static uint8_t  _delay_attempts;


// send decoded frames to the i2s buffer or ringbuffer. Volume is applied by the
// sink, in the 32 bit domain - attenuating here first would throw away
// resolution the DAC can actually render.
static void handle_pcm_data(int16_t * data, int num_audio_frames, int num_channels, int sample_rate, void * context) {
    UNUSED(sample_rate);
    UNUSED(context);
    UNUSED(num_channels);   // the count in the stream; the data is always stereo

    const btstack_audio_sink_t * audio_sink = btstack_audio_sink_get_instance();
    if (!audio_sink){
        return;
    }

    // resample into request buffer - add some additional space for resampling
    int16_t  output_buffer[(128+16) * NUM_CHANNELS]; // 16 * 8 * 2
    uint32_t resampled_frames = btstack_resample_block(&_resample_instance, data, num_audio_frames, output_buffer);

    // store data in btstack_audio buffer first
    int frames_to_copy = btstack_min(resampled_frames, _request_frames);
    memcpy(_request_buffer, output_buffer, frames_to_copy * BYTES_PER_FRAME);
    _request_frames -= frames_to_copy;
    _request_buffer += frames_to_copy * NUM_CHANNELS;

    // and rest in ring buffer
    int frames_to_store = resampled_frames - frames_to_copy;
    if (frames_to_store) {
        int status = btstack_ring_buffer_write(&_decoded_audio_ring_buffer, (uint8_t *)&output_buffer[frames_to_copy * NUM_CHANNELS], frames_to_store * BYTES_PER_FRAME);
        if (status != ERROR_CODE_SUCCESS){
            _pcm_frames_dropped += frames_to_store;
        }
    }
}


// Throttled to once a second, like the underrun report, so reporting cannot
// make the problem worse.
static void report_drops(void) {
    static uint32_t last_ms;
    static uint32_t reported_dropped, reported_pcm, reported_rejected;

    if (_sbc_frames_dropped == reported_dropped &&
        _pcm_frames_dropped == reported_pcm &&
        _sbc_frames_rejected == reported_rejected) return;

    uint32_t now = btstack_run_loop_get_time_ms();
    if ((uint32_t) (now - last_ms) < 1000) return;
    last_ms = now;

    reported_dropped  = _sbc_frames_dropped;
    reported_pcm      = _pcm_frames_dropped;
    reported_rejected = _sbc_frames_rejected;
    printf("A2DP  Sink      : frames dropped: %lu SBC ring full, %lu PCM ring full, %lu SBC too large\n",
           (unsigned long) reported_dropped, (unsigned long) reported_pcm, (unsigned long) reported_rejected);
}


/// provide pcm frames to i2s sink
static void playback_handler(int16_t * buffer, uint16_t num_audio_frames) {

    // called from lower-layer but guaranteed to be on main thread
    if (_sbc_frame_size == 0){
        memset(buffer, 0, num_audio_frames * BYTES_PER_FRAME);
        return;
    }

    // first fill from resampled audio
    uint32_t bytes_read;
    btstack_ring_buffer_read(&_decoded_audio_ring_buffer, (uint8_t *) buffer, num_audio_frames * BYTES_PER_FRAME, &bytes_read);
    buffer += bytes_read / sizeof(int16_t);
    num_audio_frames -= bytes_read / BYTES_PER_FRAME;

    // then start decoding sbc frames using request_* globals
    _request_buffer = buffer;
    _request_frames = num_audio_frames;
    while (_request_frames && btstack_ring_buffer_bytes_available(&_sbc_frame_ring_buffer) >= _sbc_frame_size) {
        // decode frame
        uint8_t sbc_frame[MAX_SBC_FRAME_SIZE];
        btstack_ring_buffer_read(&_sbc_frame_ring_buffer, sbc_frame, _sbc_frame_size, &bytes_read);
        btstack_sbc_decoder_process_data(&_state, 0, sbc_frame, _sbc_frame_size);
    }

    // ran dry: the caller's buffer is reused between calls, so anything we do
    // not write here would be played again as a chunk of the previous block
    if (_request_frames) {
        memset(_request_buffer, 0, _request_frames * BYTES_PER_FRAME);
        _request_frames = 0;
    }
}


static void media_processing_init(sbc_configuration_t * configuration) {
    if (_media_initialized) return;

    btstack_sbc_decoder_init(&_state, SBC_MODE_STANDARD, handle_pcm_data, NULL);

    btstack_ring_buffer_init(&_sbc_frame_ring_buffer, _sbc_frame_storage, sizeof(_sbc_frame_storage));
    btstack_ring_buffer_init(&_decoded_audio_ring_buffer, _decoded_audio_storage, sizeof(_decoded_audio_storage));
    // BTstack's decoder is set up with two channels and a stride of two, so it
    // always hands over interleaved stereo - a mono stream comes out with each
    // sample in both slots, though it reports one channel. Set to that one
    // channel, the resampler read half of each frame and the rest of the
    // output was whatever lay on the stack: noise in every frame.
    btstack_resample_init(&_resample_instance, NUM_CHANNELS);

    // setup audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->init(NUM_CHANNELS, configuration->sampling_frequency, &playback_handler);
    }

    _audio_stream_started = false;
    _media_initialized = true;
    return;
}


// How long the first sample of a packet waits before it plays, estimated
// before any audio has arrived. Playback starts with START_FRAMES queued, and
// the output buffers are filled from those frames, not on top of them - the
// old figure counted them twice and came out ~45ms high, enough to put sound
// noticeably ahead of the picture. What lies ahead of the newest packet is the
// rest of those frames, plus the silence the DMA was already playing: one
// buffer queued and, on average, half of the one in flight.
static uint32_t estimated_delay_us(void) {
    uint32_t rate              = _sbc_configuration.sampling_frequency;
    uint32_t samples_per_frame = _sbc_configuration.block_length * _sbc_configuration.subbands;
    if ((rate == 0) || (samples_per_frame == 0)) return 0;

    uint32_t frames_ahead = START_FRAMES - ASSUMED_FRAMES_PER_PACKET;
    uint32_t ahead_us     = (uint32_t) (((uint64_t) frames_ahead * samples_per_frame * 1000000u) / rate);
    return ahead_us + 3 * btstack_audio_pico_sink_buffer_us(rate) / 2;
}


static uint16_t delay_report_value(uint32_t delay_us) {
    uint32_t value = delay_us / 100;
    if (value < DELAY_REPORT_MIN_100US) value = DELAY_REPORT_MIN_100US;
    if (value > DELAY_REPORT_MAX_100US) value = DELAY_REPORT_MAX_100US;
    return (uint16_t) value;
}


// The source switches Delay Reporting on in SET_CONFIGURATION if it wants
// reports. Sending one it did not ask for is out of spec, and a source that
// rejects it leaves BTstack's A2DP layer thinking the stream is gone.
static bool delay_reporting_enabled(void) {
    return _delay_reporting && !_delay_rejected;
}


static void delay_report_timer_handler(btstack_timer_source_t * ts);

static void delay_report_schedule(uint32_t delay_ms) {
    btstack_run_loop_remove_timer(&_delay_timer);
    btstack_run_loop_set_timer_handler(&_delay_timer, &delay_report_timer_handler);
    btstack_run_loop_set_timer(&_delay_timer, delay_ms);
    btstack_run_loop_add_timer(&_delay_timer);
}


// Tell the source how far behind we are so it can delay the video to match,
// rather than us shrinking buffers and hoping. Goes out as soon as BTstack
// and the rate limit allow.
static void delay_report_request(uint16_t value) {
    if (value == _delay_sent) {
        _delay_pending = 0;         // the source already has it; drop anything older
        return;
    }
    _delay_pending  = value;
    _delay_attempts = 0;
    // not from here: this runs inside BTstack's event dispatch, where the
    // accept that makes the endpoint CONFIGURED may not have gone out yet
    delay_report_schedule(1);
}


static void delay_report_timer_handler(btstack_timer_source_t * ts) {
    UNUSED(ts);
    if (_delay_pending == 0) return;
    if ((_cid == 0) || !delay_reporting_enabled()) {
        _delay_pending = 0;
        return;
    }

    uint32_t now   = btstack_run_loop_get_time_ms();
    uint32_t since = now - _delay_sent_ms;
    if (_delay_sent_once && (since < DELAY_REPORT_INTERVAL_MS)) {
        delay_report_schedule(DELAY_REPORT_INTERVAL_MS - since);
        return;
    }

    uint8_t status = avdtp_sink_delay_report(_cid, _seid, _delay_pending);
    if ((status == ERROR_CODE_COMMAND_DISALLOWED) && (++_delay_attempts < DELAY_REPORT_ATTEMPTS)) {
        // endpoint not CONFIGURED yet, or another command of ours in flight
        delay_report_schedule(DELAY_REPORT_RETRY_MS);
        return;
    }
    if (status != ERROR_CODE_SUCCESS) {
        printf("A2DP  Sink      : delay report not sent, status 0x%02x\n", status);
        _delay_pending = 0;
        return;
    }

    printf("A2DP  Sink      : delay report %u.%u ms\n",
           (unsigned) (_delay_pending / 10), (unsigned) (_delay_pending % 10));
    _delay_sent      = _delay_pending;
    _delay_sent_ms   = now;
    _delay_sent_once = true;
    _delay_pending   = 0;
}


// A new configuration: whatever the source was told before no longer holds.
static void delay_report_reset(void) {
    btstack_run_loop_remove_timer(&_delay_timer);
    _delay_pending = 0;
    _delay_sent    = 0;
}


// A source that switched Delay Reporting on and then rejects a report is
// broken, but the damage lands here: BTstack's A2DP layer answers any reject
// of a command of ours by dropping its stream state to CONNECTED, and from then
// on never forwards START or SUSPEND - and on SDK 1.5.1 never STREAM_ESTABLISHED
// either - so the speaker goes quiet until the next connection. Put back the
// state it was in, and stop reporting to this source.
static void delay_report_rejected(uint16_t cid) {
    printf("A2DP  Sink      : source rejected the delay report, sending no more\n");
    _delay_rejected = true;
    delay_report_reset();

    avdtp_connection_t * connection = avdtp_get_connection_for_avdtp_cid(cid);
    if ((connection == NULL) || (connection->a2dp_sink_config_process.state != A2DP_CONNECTED)) return;
    connection->a2dp_sink_config_process.state =
        (_stream_state == STREAM_STATE_CLOSED) ? A2DP_W4_OPEN_STREAM_WITH_SEID : A2DP_STREAMING_OPENED;
}


// The first report of a configuration, before there is audio to measure.
static void delay_report_initial(void) {
    if (!delay_reporting_enabled()) return;
    if ((_delay_sent != 0) || (_delay_pending != 0)) return;

    uint32_t estimate_us = estimated_delay_us();
    if (estimate_us == 0) return;

    printf("A2DP  Sink      : latency estimate %lu.%lu ms\n",
           (unsigned long) (estimate_us / 1000), (unsigned long) (estimate_us % 1000 / 100));
    delay_report_request(delay_report_value(estimate_us));
}


static void measure_delay_restart(void) {
    _latency_sum      = 0;
    _latency_count    = 0;
    _latency_reported = false;
}


// What the first sample of the packet just written will wait before it plays:
// every SBC frame ahead of it in the ring, the decoded audio not yet handed
// over, and everything the output has queued, silence included. Decoding is
// pulled by the output, so this is the whole of our latency. Averaged over a
// second and re-reported when it has moved far enough to matter.
static void measure_delay(unsigned packet_frames) {
    uint32_t rate              = _sbc_configuration.sampling_frequency;
    uint32_t samples_per_frame = _sbc_configuration.block_length * _sbc_configuration.subbands;
    if (!_audio_stream_started || (rate == 0) || (samples_per_frame == 0) || (_sbc_frame_size == 0)) return;
    if ((_cid == 0) || !delay_reporting_enabled()) return;

    // rounded up: after a bitpool change the reads no longer line up with
    // frames, and the decoder holds a partly consumed one outside the ring
    uint32_t sbc_bytes  = btstack_ring_buffer_bytes_available(&_sbc_frame_ring_buffer);
    uint32_t sbc_frames = (sbc_bytes + _sbc_frame_size - 1) / _sbc_frame_size;
    uint32_t ahead      = (sbc_frames > packet_frames) ? (sbc_frames - packet_frames) : 0;
    uint32_t frames     = ahead * samples_per_frame
                        + btstack_ring_buffer_bytes_available(&_decoded_audio_ring_buffer) / BYTES_PER_FRAME
                        + btstack_audio_pico_sink_queued_frames();

    uint32_t now = btstack_run_loop_get_time_ms();
    if (_latency_count == 0) _latency_window_ms = now;
    _latency_sum += frames;
    _latency_count++;
    if ((uint32_t) (now - _latency_window_ms) < DELAY_REPORT_INTERVAL_MS) return;

    uint32_t mean_us = (uint32_t) ((_latency_sum * 1000000u) / ((uint64_t) _latency_count * rate));
    _latency_sum   = 0;
    _latency_count = 0;

    uint16_t value  = delay_report_value(mean_us);
    int      change = (int) value - (int) _delay_sent;
    bool     moved  = (_delay_sent == 0) || (change >= DELAY_REPORT_STEP_100US) || (change <= -DELAY_REPORT_STEP_100US);

    // once per start regardless, so a log shows what the report rests on
    if (!moved && _latency_reported) return;
    _latency_reported = true;

    printf("A2DP  Sink      : latency measured %lu.%lu ms\n",
           (unsigned long) (mean_us / 1000), (unsigned long) (mean_us % 1000 / 100));
    if (moved) delay_report_request(value);
}


static void media_processing_start(void) {
    if (!_media_initialized) return;

    // setup audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        audio->start_stream();
    }
    _audio_stream_started = true;
}


static void media_processing_pause(void) {
    if (!_media_initialized) return;

    // stop audio playback
    _audio_stream_started = false;

    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio) {
        audio->stop_stream();
    }
    // discard pending data
    btstack_ring_buffer_reset(&_decoded_audio_ring_buffer);
    btstack_ring_buffer_reset(&_sbc_frame_ring_buffer);
}


static void media_processing_close(void) {
    if (!_media_initialized) return;

    _media_initialized = false;
    _audio_stream_started = false;
    _sbc_frame_size = 0;

    // stop audio playback
    const btstack_audio_sink_t * audio = btstack_audio_sink_get_instance();
    if (audio){
        // printf("close stream\n");
        audio->close();
    }
}


// Largest frame a source may send under configuration c, i.e. at its maximum
// bitpool. Mono and dual channel spend a bitpool per channel; joint stereo
// adds a join bit per subband.
static unsigned sbc_max_frame_bytes(const sbc_configuration_t * c) {
    unsigned channels = (c->channel_mode == SBC_CHANNEL_MODE_MONO) ? 1 : 2;
    unsigned bits;

    switch (c->channel_mode){
        case SBC_CHANNEL_MODE_JOINT_STEREO:
            bits = c->subbands + c->block_length * c->max_bitpool_value;
            break;
        case SBC_CHANNEL_MODE_STEREO:
            bits = c->block_length * c->max_bitpool_value;
            break;
        default:
            bits = c->block_length * channels * c->max_bitpool_value;
            break;
    }
    return SBC_FRAME_BYTES(channels, c->subbands, bits);
}


static void event_handler(uint8_t event, uint8_t *packet) {
    uint8_t status;
    uint8_t allocation_method;

    switch (event){
        // case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
        //     printf("A2DP  Sink      : Received non SBC codec - not implemented\n");
        //     break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:{
            // printf("A2DP  Sink      : Received SBC codec configuration\n");
            _sbc_configuration.reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            _sbc_configuration.num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            _sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            _sbc_configuration.block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            _sbc_configuration.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            _sbc_configuration.min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            _sbc_configuration.max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            
            allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            
            // Adapt Bluetooth spec definition to SBC Encoder expected input
            _sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
           
            switch (a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet)) {
                case AVDTP_CHANNEL_MODE_JOINT_STEREO:
                    _sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_STEREO:
                    _sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
                    _sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
                    break;
                case AVDTP_CHANNEL_MODE_MONO:
                    _sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
                    break;
                default:
                    btstack_assert(false);
                    break;
            }
            // dump_sbc_configuration(&_a2dp->_sbc_configuration);

            // MAX_SBC_FRAME_SIZE covers everything we advertise, so this only
            // fires if the capabilities and the limit drift apart. Say so now
            // rather than leave media_handler dropping the stream in silence.
            unsigned frame_bytes = sbc_max_frame_bytes(&_sbc_configuration);
            if (frame_bytes > MAX_SBC_FRAME_SIZE){
                printf("A2DP  Sink      : SBC frames of up to %u bytes negotiated, only %u fit\n",
                       frame_bytes, (unsigned) MAX_SBC_FRAME_SIZE);
            }

            // The first delay report belongs in the configured state, before
            // OPEN: some sources hold OPEN until they hear it (Android up to
            // 2s, ESP-IDF 5s). When the source is configuring us, the endpoint
            // is still mid-configuration here and the report follows the
            // accept. When we configured the source, BTstack sends OPEN next
            // on its own and the report would collide with it, so it waits for
            // STREAM_ESTABLISHED.
            _cid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_a2dp_cid(packet);
            delay_report_reset();

            // Decided by SET_CONFIGURATION, already parsed into remote_sep in
            // both flows. A RECONFIGURE may only carry the codec, and BTstack
            // overwrites the field with that, so it is read once here.
            if (!_sbc_configuration.reconfigure){
                _delay_reporting = (_endpoint->remote_sep.configured_service_categories & (1 << AVDTP_DELAY_REPORTING)) != 0;
            }

            if (_endpoint->state == AVDTP_STREAM_ENDPOINT_CONFIGURATION_SUBSTATEMACHINE){
                delay_report_initial();
            }
            break;
        }

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
            // A new source: start it clean. BTstack only resets the endpoint
            // when a configured stream is released, so a configuration of ours
            // that failed before the accept would leave the Delay Reporting
            // bit set below, and that source's SEP seid, for the next source -
            // which would then be configured with Delay Reporting it may never
            // have offered, and a strict one rejects that. One connection at a
            // time, so this endpoint is all there is.
            if (a2dp_subevent_signaling_connection_established_get_status(packet) != ERROR_CODE_SUCCESS) break;
            if (_endpoint != NULL){
                _endpoint->remote_configuration_bitmap &= (uint16_t) ~(1u << AVDTP_DELAY_REPORTING);
                _endpoint->set_config_remote_seid = 0;
            }
            delay_report_reset();
            _delay_sent_once = false;
            _delay_rejected  = false;
            _delay_reporting = false;
            break;

        case A2DP_SUBEVENT_SIGNALING_DELAY_REPORTING_CAPABILITY:
            // When BTstack configures the source itself, it decides on Delay
            // Reporting at the SBC capability, which arrives before this one,
            // so it never switches it on. Do it here, for the SEP it picked:
            // SET_CONFIGURATION only goes out once all capabilities are in.
            if ((_endpoint != NULL) && (_endpoint->set_config_remote_seid != 0) &&
                (a2dp_subevent_signaling_delay_reporting_capability_get_remote_seid(packet) == _endpoint->set_config_remote_seid)){
                _endpoint->remote_configuration_bitmap |= (uint16_t) (1u << AVDTP_DELAY_REPORTING);
            }
            break;

        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                // printf("A2DP  Sink      : Streaming connection failed, status 0x%02x\n", status);
                break;
            }

            // a2dp_subevent_stream_established_get_bd_addr(packet, _addr);
            // _cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            _seid = a2dp_subevent_stream_established_get_local_seid(packet);
            _cid  = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            _stream_state = STREAM_STATE_OPEN;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            gpio_put(CONN_PIN, 1);
            delay_report_initial();

            // inquiry scan keeps interrupting the ACL link on its own 1.28s
            // cycle, and nobody needs to discover a speaker that is in use
            gap_discoverable_control(0);

            // printf("A2DP  Sink      : Streaming connection is established, address %s, cid 0x%02x, local seid %d\n",
            //        bd_addr_to_str(_a2dp->addr), _a2dp->a2dp_cid, _a2dp->a2dp_local_seid);
            break;
        
        case A2DP_SUBEVENT_STREAM_STARTED:
            // printf("A2DP  Sink      : Stream started\n");
            _stream_state = STREAM_STATE_PLAYING;
            if (_sbc_configuration.reconfigure){
                media_processing_close();
            }
            // prepare media processing
            media_processing_init(&_sbc_configuration);
            delay_report_initial();     // after a reconfigure, nothing else sent one
            // audio stream is started when buffer reaches minimal level
            break;
        
        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            // printf("A2DP  Sink      : Stream paused\n");
            _stream_state = STREAM_STATE_PAUSED;
            media_processing_pause();
            break;
        
        case A2DP_SUBEVENT_STREAM_RELEASED:
            // printf("A2DP  Sink      : Stream released\n");
            _stream_state = STREAM_STATE_CLOSED;
            _cid = 0;
            delay_report_reset();
            _delay_sent_once = false;
            _delay_rejected  = false;
            _delay_reporting = false;
            media_processing_close();
            gap_discoverable_control(1);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            gpio_put(CONN_PIN, 0);
            watchdog_enable(100, true);  // reboot in 0.1s, since reconnect is buggy
            break;
        
        case A2DP_SUBEVENT_COMMAND_REJECTED:
            // BTstack only forwards rejects of commands we sent
            if (a2dp_subevent_command_rejected_get_signal_identifier(packet) == AVDTP_SI_DELAYREPORT){
                delay_report_rejected(a2dp_subevent_command_rejected_get_a2dp_cid(packet));
            }
            break;

        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            // printf("A2DP  Sink      : Signaling connection released\n");
            // _cid = 0;
            // _stream_state = STREAM_STATE_CLOSED;
            // media_processing_close();
            // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            // gpio_put(CONN_PIN, 0);
            break;
        
        default:
            break;
    }
}


static void data_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    event_handler(packet[2], packet);
}


int read_media_header(uint8_t *packet, int size, int *offset, avdtp_media_packet_header_t *media_header) {
    int media_header_len = 12; // without crc
    int pos = *offset;
    
    if (size - pos < media_header_len) {
        return 0;
    }

    media_header->version = packet[pos] & 0x03;
    media_header->padding = get_bit16(packet[pos],2);
    media_header->extension = get_bit16(packet[pos],3);
    media_header->csrc_count = (packet[pos] >> 4) & 0x0F;
    pos++;

    media_header->marker = get_bit16(packet[pos],0);
    media_header->payload_type  = (packet[pos] >> 1) & 0x7F;
    pos++;

    media_header->sequence_number = big_endian_read_16(packet, pos);
    pos+=2;

    media_header->timestamp = big_endian_read_32(packet, pos);
    pos+=4;

    media_header->synchronization_source = big_endian_read_32(packet, pos);
    pos+=4;
    *offset = pos;

    return 1;
}


int read_sbc_header(uint8_t * packet, int size, int * offset, avdtp_sbc_codec_header_t * sbc_header) {
    int sbc_header_len = 12; // without crc
    int pos = *offset;
    
    if (size - pos < sbc_header_len) {
        return 0;
    }

    sbc_header->fragmentation = get_bit16(packet[pos], 7);
    sbc_header->starting_packet = get_bit16(packet[pos], 6);
    sbc_header->last_packet = get_bit16(packet[pos], 5);
    sbc_header->num_frames = packet[pos] & 0x0f;
    pos++;
    *offset = pos;

    return 1;
}


static void media_handler(uint8_t seid, uint8_t *packet, uint16_t size) {
    UNUSED(seid);

    int pos = 0;
     
    avdtp_media_packet_header_t media_header;
    if (!read_media_header(packet, size, &pos, &media_header)) return;
    
    avdtp_sbc_codec_header_t sbc_header;
    if (!read_sbc_header(packet, size, &pos, &sbc_header)) return;

    int packet_length = size-pos;
    uint8_t *packet_begin = packet + pos;

    // store sbc frame size for buffer management. It comes off the wire, so it
    // is a bound to check rather than a number to trust: playback_handler reads
    // one frame of this size into a MAX_SBC_FRAME_SIZE stack buffer.
    if (sbc_header.num_frames == 0) return;
    report_drops();
    unsigned frame_size = packet_length / sbc_header.num_frames;
    if (frame_size == 0 || frame_size > MAX_SBC_FRAME_SIZE) {
        _sbc_frames_rejected += sbc_header.num_frames;
        return;
    }
    _sbc_frame_size = frame_size;
    int status = btstack_ring_buffer_write(&_sbc_frame_ring_buffer, packet_begin, packet_length);
    if (status != ERROR_CODE_SUCCESS){
        _sbc_frames_dropped += sbc_header.num_frames;
    }

    // decide on audio sync drift based on number of sbc frames in queue
    int sbc_frames_in_buffer = btstack_ring_buffer_bytes_available(&_sbc_frame_ring_buffer) / _sbc_frame_size;

    uint32_t resampling_factor;

    // nominal factor (fixed-point 2^16) and compensation offset
    uint32_t nominal_factor = 0x10000;
    uint32_t compensation   = 0x00100;

    if (sbc_frames_in_buffer < OPTIMAL_FRAMES_MIN){
    	resampling_factor = nominal_factor - compensation;    // stretch samples
    } else if (sbc_frames_in_buffer <= OPTIMAL_FRAMES_MAX){
    	resampling_factor = nominal_factor;                   // nothing to do
    } else {
    	resampling_factor = nominal_factor + compensation;    // compress samples
    }

    btstack_resample_set_factor(&_resample_instance, resampling_factor);

    // start stream if enough frames buffered
    if (!_audio_stream_started && sbc_frames_in_buffer >= START_FRAMES){
        media_processing_start();
        measure_delay_restart();
    }

    if (status == ERROR_CODE_SUCCESS){
        measure_delay(sbc_header.num_frames);
    }
}


void a2dp_sink_begin() {
    // Init I2S interface
    btstack_audio_sink_set_instance(btstack_audio_pico_sink_get_instance());

    // Init connection indicator pin (high if bt a2dp stream connected)
    gpio_init(CONN_PIN);
    gpio_set_dir(CONN_PIN, GPIO_OUT);
    gpio_put(CONN_PIN, 0);  // set to 1 while a bt connection is active

    a2dp_sink_init();

    a2dp_sink_register_packet_handler(&data_handler);
    a2dp_sink_register_media_handler(&media_handler);

    // BTstack keeps this pointer and copies every configuration the source
    // sets into it, long after this function has returned - so not the stack.
    static uint8_t sbc_configuration[4];
    _endpoint = a2dp_sink_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC,
        _sbc_capabilities, sizeof(_sbc_capabilities),
        sbc_configuration, sizeof(sbc_configuration));
    _seid = avdtp_local_seid(_endpoint);

    // BTstack gives the endpoint the Delay Reporting capability itself, which
    // is what lets a source turn reports on and sync its video to them.
}


uint32_t a2dp_sink_sbc_frames_dropped(void) {
    return _sbc_frames_dropped;
}


uint32_t a2dp_sink_pcm_frames_dropped(void) {
    return _pcm_frames_dropped;
}


uint32_t a2dp_sink_sbc_frames_rejected(void) {
    return _sbc_frames_rejected;
}
