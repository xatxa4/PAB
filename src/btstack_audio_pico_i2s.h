#ifndef BTSTACK_AUDIO_PICO_I2S_H
#define BTSTACK_AUDIO_PICO_I2S_H

#include "btstack_audio.h"

/// Bring up MCLK/BCLK/LRCLK and the core1 DMA pump. Idempotent.
/// Must run before cyw43_arch_init() and stdio_init_all(): with
/// PICO_AUDIO_I2S_CLOCK_MODE=CLOCK_MODE_LOW_JITTER this re-runs the system PLL,
/// and drivers started earlier would keep dividers derived from the old clock.
void btstack_audio_pico_i2s_begin(void);

const btstack_audio_sink_t * btstack_audio_pico_sink_get_instance(void);

#endif
