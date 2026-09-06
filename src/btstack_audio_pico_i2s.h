#ifndef BTSTACK_AUDIO_PICO_I2S_H
#define BTSTACK_AUDIO_PICO_I2S_H

#include "btstack_audio.h"

const btstack_audio_sink_t * btstack_audio_pico_sink_get_instance(void);

/// Audio held between the sink accepting a frame and the DAC clocking it out,
/// for AVDTP delay reporting. Tracks the buffer sizing, so it stays honest if
/// PICO_AUDIO_I2S_NUM_BUFFERS or _BUFFER_FRAMES change.
uint32_t btstack_audio_pico_sink_latency_us(uint32_t sample_rate);

#endif
