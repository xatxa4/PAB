# What to adopt, and in what order

Everything identified across the debugging and research so far, sorted by
whether it is a bug, an enabler, or a nice-to-have — and filtered through one
question, because the box is going to grow more personalities (USB, WiFi radio,
S/PDIF) with different sample rates and bit depths:

> **Does this improvement make the next source harder to add?**

Three ways an improvement can lay a rock in the way, and each item below is
checked against them:

1. **It hardcodes a rate or a depth.** 44.1 kHz is what Bluetooth gives us.
   S/PDIF brings 48 and 96, USB brings whatever the host picks. Anything sized
   in *frames* rather than *time* silently changes meaning when the rate does.
2. **It assumes Bluetooth.** Anything that reaches into the run loop, the radio,
   or BTstack from code a future source will also need.
3. **It turns a peripheral off globally.** Every power measure below is a
   statement about what *this* mode needs. As a global `#define` it is a trap
   the fourth personality springs.

Tiers are ordered by what blocks what, not by size.

---

## Tier 0 — These are bugs

### 0.1 SBC frames can overflow the stack (`src/a2dp.c`)

The live one. `_sbc_frame_size` is computed from the received packet:

```c
// a2dp.c:449
_sbc_frame_size = packet_length / sbc_header.num_frames;
```

and then used, unbounded, to fill a fixed stack buffer:

```c
// a2dp.c:64,165-166
#define MAX_SBC_FRAME_SIZE 120
uint8_t sbc_frame[MAX_SBC_FRAME_SIZE];
btstack_ring_buffer_read(&_sbc_frame_ring_buffer, sbc_frame, _sbc_frame_size, &bytes_read);
```

120 bytes is only enough for **joint stereo**. The SBC frame length for 8
subbands and 16 blocks is `4 + 8 + ceil((8 + 16*bitpool)/8)` joint, but
`4 + 8 + ceil(16*2*bitpool/8)` dual-channel:

| Configuration | bitpool | Frame bytes |
|---|---|---|
| Joint stereo | 53 | 119 — fits |
| Dual channel (SBC XQ) | 38 | 164 — **overflows** |
| Dual channel | 53 | 224 — **overflows** |

`_sbc_capabilities` advertises `0xFF, 0xFF, 2, 53` (`a2dp.c:88`), which offers
every channel mode including dual channel, so a compliant source may negotiate
any of these. This is exactly the SBC XQ failure — right channel only, then the
controller crashes. The same bug is in BTstack's own `a2dp_sink_demo.c`, which
is where it came from.

**Fix, in three parts** — the constant alone is not enough, because the size
comes off the wire:

```c
#define MAX_SBC_FRAME_SIZE 224          // dual channel, 16 blocks, bitpool 53

// at a2dp.c:449, before anything uses it
unsigned frame_size = packet_length / sbc_header.num_frames;
if (frame_size == 0 || frame_size > MAX_SBC_FRAME_SIZE) return;   // drop it
_sbc_frame_size = frame_size;
```

and re-derive the bound from the negotiated configuration at
`AVDTP_SUBEVENT_..._SBC_CONFIGURATION` time rather than trusting the constant,
so raising the advertised bitpool later cannot quietly re-break it.

Fixing this **also unlocks SBC XQ**, which is the cheapest audio-quality
improvement available and needs no protocol work.

*Future-proofing:* none of this is Bluetooth-specific in spirit — it is the
general rule that a frame size taken from an input is a bound to check, not a
number to trust. The same applies to USB isochronous packet lengths and S/PDIF
block sizes.

### 0.2 Two ring buffer overflows are discarded silently

```c
// a2dp.c:137 and a2dp.c:450
int status = btstack_ring_buffer_write(...);
// if (status) { printf(...); }     <- commented out
```

Both are the `-Wextra` warnings the build reports. They are not cosmetic: when
the ring buffer is full the write **fails and the audio is dropped**, with
nothing said. That is a candidate stutter source that has been invisible the
whole time we were chasing stutter.

Handle the status: count the drops, expose the counter alongside
`audio_out_underruns()`, and print it throttled. Two counters that distinguish
*source too slow* (underrun) from *source too fast / buffer too small* (overflow)
turn the next stutter report into a five-minute diagnosis instead of a week.

### 0.3 The SBC frame store has almost no headroom

```c
// a2dp.c:105
uint8_t _sbc_frame_storage[(OPTIMAL_FRAMES_MAX + ADDITIONAL_FRAMES) * MAX_SBC_FRAME_SIZE];
```

50 frames of storage with `OPTIMAL_FRAMES_MAX` at 40 — a burst past the target
depth hits 0.2 immediately. And note the sizing multiplies by
`MAX_SBC_FRAME_SIZE`: raising that to 224 grows this from 6000 to 11200 bytes
(fine, there is 264 KB), but the *frame count* stays 50 only if the arithmetic
is kept honest. Assert it rather than hoping.

---

## Tier 1 — Structural. Do these before the next personality, not after

These are the items that decide whether Tier 2 is safe.

### 1.1 Give each mode a capability descriptor

**This is the single most important item in the document,** because six of the
remaining improvements are only safe once it exists.

Every power measure below is a claim about which peripherals a mode needs. As a
global setting, each one is a landmine for a mode that does not exist yet: gate
`CLK_SYS_USBCTRL` for the Bluetooth sink and the USB sound card boots dead;
hold SPI in reset and a future S/PDIF front end using SPI silently fails.

So `app_mode` should carry, per mode, what that mode needs:

```c
typedef struct {
    const char * name;
    uint32_t     wake_en0, wake_en1;    // peripherals to keep clocked
    uint32_t     sleep_en0, sleep_en1;
    uint32_t     keep_out_of_reset;     // RESETS bits
    bool         needs_usb, needs_radio, needs_adc;
    uint32_t     preferred_sys_clk_hz;  // 0 = SDK default
} app_mode_config_t;
```

Then one function applies the profile at boot, and adding a personality means
adding a row — not auditing every power measure in the tree. Without this,
Tier 2 makes further development *more* fragile, which is precisely what must
not happen.

### 1.2 `audio_out`: size buffers in time, not frames

```c
// audio_out.c:87,90,112
#define PICO_AUDIO_I2S_BUFFER_FRAMES 512
#define BUFFER_WORDS (PICO_AUDIO_I2S_BUFFER_FRAMES * 2)
static int32_t audio_buffer[PICO_AUDIO_I2S_NUM_BUFFERS][BUFFER_WORDS];
```

`audio_out.h` asks callers to service "at least twice per buffer period", so the
budget is half a buffer — and 512 frames is a different length of time at every
rate:

| Sample rate | One buffer | Service budget | vs. the 5 ms tick |
|---|---|---|---|
| 44 100 Hz | 11.61 ms | 5.80 ms | holds, 14% margin |
| 48 000 Hz | 10.67 ms | 5.33 ms | holds, 6% margin |
| 96 000 Hz | 5.33 ms | 2.67 ms | **misses** |
| 192 000 Hz | 2.67 ms | 1.33 ms | **misses badly** |

The buffer depth you tuned for movie latency changes meaning the moment a source
picks a different rate. That is a dropout that will appear the first time S/PDIF
or USB runs hi-res, and it will look exactly like the stutter we already spent a
week on.

Size the allocation for the worst case (highest supported rate × target latency)
and set the *used* length per rate in `audio_out_set_sample_rate()`. Report the
service deadline through the existing `audio_out_frames_per_buffer()` so callers
can pick their own timer interval instead of hardcoding 5 ms.

This is rock number one, and it is already in the tree.

### 1.3 `audio_out`: validate and document the rate-change contract

`audio_out_set_sample_rate()` accepts anything and returns nothing. It is called
from `btstack_audio_pico_i2s.c:131` — currently before the stream starts, which
is safe, but nothing in the API says so, and a USB host that changes rate
mid-stream will call it while the DMA is running.

Give it a return value, a documented "stop → reprogram → start" contract, and a
check that the PIO divider is achievable and how far off it lands. The divider
is `clk_sys / (rate * 128)` in 16.8 fixed point, so the error is worth reporting
rather than discovering by ear.

*Note:* `_sbc_capabilities` already advertises 16 and 32 kHz. Those paths have
never been exercised.

### 1.4 DMA IRQ priority and bus priority

Two lines, no design implications, and they harden every future source that uses
`audio_out`:

```c
irq_set_priority(DMA_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);   // audio_out.c, near :221
bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS;
```

The refill IRQ currently ranks equal with the CYW43 background IRQ, and DMA
ranks below the processors on the bus. Neither is what you want for an audio
path that must not miss a buffer boundary.

### 1.5 Hard fault handler and a real watchdog

Today a fault hangs silently and the only watchdog use is the mode-switch
reboot. With four personalities, a fault handler that records where it died in a
scratch register and reboots into the last-known-good mode is worth more than
any single feature.

Two interactions to respect: the watchdog must be fed across the BOOTSEL blink
(~1.8 s of blocking in `mode_button.c`), and the fault handler must not clobber
`watchdog_hw->scratch[0]`, which carries the mode.

---

## Tier 2 — Power. Safe only once 1.1 exists

Ordered by return per unit of risk. Nothing here should be adopted as a global
setting; each is a row in the mode descriptor.

### 2.1 Peripheral clock gating — best value, no timing risk

The RP2040 implements exactly the clock gating the ARM low-power guidance asks
for, in hardware, and the SDK never touches it:

```
CLOCKS_WAKE_EN0/1_RESET  = 0xffffffff     // gates while running
CLOCKS_SLEEP_EN0/1_RESET = 0xffffffff     // takes over when all cores sleep
```

Everything is on: `CLK_ADC_ADC`, `CLK_SYS_ADC`, `CLK_RTC_RTC`, `CLK_SYS_RTC`,
`CLK_SYS_I2C0/1`, `CLK_SYS_SPI0/1`, `CLK_PERI_SPI0/1`, `CLK_SYS_PWM`,
`CLK_SYS_JTAG`, `CLK_SYS_UART0/1`, `CLK_PERI_UART0/1`, `CLK_SYS_USBCTRL`,
`CLK_USB_USBCTRL`. The Bluetooth sink uses none of them.

`SLEEP_EN` is live for us because core1 is parked in the bootrom's WFE loop and
therefore counts as asleep, and the run loop genuinely reaches `__wfe`
(`btstack_run_loop_async_context` → `sem_acquire_block_until` →
`lock_core.h:177` → `best_effort_wfe_or_timeout`).

It cannot change a single timing relationship in the audio path, which is why it
goes first. Honest bound: RP2040 has no per-peripheral *power* gating, so this
saves dynamic clock-tree power — expect hundreds of µA, not milliamps.

**Keep clocked regardless:** PIO0 (the CYW43 bus), PIO1 (I2S), DMA, XIP, TIMER,
WATCHDOG, ROM, SIO, PADS, IO, BUSFABRIC, BUSCTRL, both PLLs, XOSC, RESETS, PSM.

### 2.2 Stop un-resetting every peripheral at boot

`runtime_init_post_clock_resets()` (`pico_runtime_init/runtime_init.c:148-152`)
is `__weak` and calls `unreset_block_mask_wait_blocking(RESETS_RESET_BITS)` —
bringing every peripheral out of reset, including several that early init
deliberately left in reset at `runtime_init.c:73-86`. Overridable via
`PICO_RUNTIME_SKIP_POST_CLOCK_RESETS`.

Composes with 2.1: reset stops a block being *used*, gating stops its clock tree
toggling. Per-mode, from the descriptor.

### 2.3 Stop the clocks nothing uses

`runtime_init_clocks.c:108-133` configures, unconditionally:

- `clk_usb` = 48 MHz
- `clk_adc` = PLL_USB / 1 = **48 MHz** — for an ADC this project never reads
- `clk_rtc` = 46.875 kHz — for an RTC this project never reads

The temperature sensor itself is off (`ADC.CS.TS_EN` resets to 0), but the ADC
block is clocked at 48 MHz for nothing.

`clk_usb` and PLL_USB can only go when `PAB_STDIO != usb` *and* the mode is not
the USB sound card — squarely a descriptor decision. Keep PLL_USB if 2.5 is ever
revisited, since it is the glitchless park source.

### 2.4 Pad hygiene on the I2S pins — and the rate trap in it

GP18/20/21 switch continuously with their input receivers still enabled
(PADS_BANK0 resets to `0x56`: IE=1, PDE=1, 4 mA drive).

```c
gpio_set_input_enabled(pin, false);                          // always right
gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);       // current and EMI
gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);                // rate-dependent!
```

The first two are unconditionally good. **Slow slew is not.** BCLK is 2.8 MHz at
44.1 kHz but 6.1 MHz at 96 kHz and 12.3 MHz at 192 kHz; slow slew that cleans up
edges at 44.1 will round them off at hi-res. Either make it conditional on the
configured rate inside `audio_out_set_sample_rate()`, or leave slew alone. Do
not set it once at init and forget — that is rock number two.

Unused pins are already held by their pull-downs, so there is no shoot-through
to fix there.

### 2.5 Dynamic / hybrid `clk_sys` scaling — **defer, and here is why**

Technically possible, and the mechanism is clean: switch `clk_sys` between the
already-locked PLL_SYS (125 MHz) and PLL_USB (48 MHz) through the glitchless
mux, avoiding the relock window that makes `set_sys_clock_pll()` unusable here
(it parks on PLL_USB and calls `pll_init`, so BCLK runs at 48/125 of nominal for
tens of µs — audible). `CYW43_PIO_CLOCK_DIV_DYNAMIC` and
`cyw43_set_pio_clock_divisor()` exist for exactly this and are off by default.

**But this is the item that most directly violates the brief.** On RP2040 the
PIO block has no clock mux — it runs from `clk_sys`, full stop. So `clk_sys` is
shared by the I2S output, the CYW43 bus, and *every future PIO-based source*: a
S/PDIF receiver's recovery PIO, a USB feedback path's timing. Making `clk_sys`
mutable means every one of those has to re-derive its dividers correctly, at the
right moment, forever. It converts a constant into an invariant that four
subsystems must jointly maintain.

Against unmeasured payoff on the smaller half of the power budget. **Do not
adopt.** If it is ever revisited, the useful split is not fast-vs-slow while
streaming — the load is constant there — but *idle vs streaming*: park at 48 MHz
and 0.95 V when no stream is open. Revisit only after 2.6 says the core is worth
attacking at all.

---

## Tier 3 — Cheap, do them while touching the files anyway

- **`-Wall -Wextra`.** Currently off; turning it on surfaces exactly the two
  real bugs in 0.2 and nothing else. That is a good ratio.
- **Git hash in the build.** Two rounds of debugging in this project were spent
  testing a binary that did not contain the fix. A hash printed at boot ends
  that class of problem permanently.
- **`PAB_MODES` build option** — compile out personalities you are not shipping.
  Matters more with four modes than with two.

---

## Measure before spending more on power

Two numbers, both cheap, and they decide whether Tier 2 is worth the effort at
all. I have been giving you estimates:

1. **Core duty cycle while streaming.** Accumulate `time_us_64()` deltas around
   the WFE in the run loop. If the core already idles 80% of the time, clock
   gating is the whole prize and scaling is worth nothing.
2. **The RP2040 / CYW43 split.** Board current with the radio never brought up
   vs. streaming. If the radio dominates — which I expect — everything in Tier 2
   is fighting for the smaller half, and the leverage is on the radio side:
   making sure the source actually AVDTP-suspends when paused instead of
   streaming silence, which costs continuous radio time and is invisible from
   here.

For reference, `nhasbun/pico_power_saving` measures ~5.5 mA underclocked to
15.3 MHz at 0.90 V and ~2 mA in deep sleep — on a plain Pico with no radio and
nothing to do. Those are not our numbers.

---

## Considered and rejected

| | Why not |
|---|---|
| Deep sleep / DORMANT | Stops XOSC. Kills the CYW43 bus, the service timer and the I2S clock. Not usable while streaming. |
| Sleep-on-exit (`SCR.SLEEPONEXIT`) | The design is a run loop, not interrupt-only. SBC decode and `audio_out_service()` run in thread context. |
| "Race to sleep" | Assumes leakage comparable to dynamic power. At 40 nm LP and these clocks, dynamic dominates. The load is constant anyway — no burstiness to race through. |
| Force SMPS PWM (`WL_GPIO1`) | Trades power *away* for lower supply ripple. Worth knowing it exists — the buck defaults to PFM, the efficient mode — but it is the wrong direction here. |
| LE Audio / LC3 | CYW43439 is BT 5.2 without Isochronous Channels. The hardware cannot. |
| AAC | Licensing, and BTstack ships no AAC decoder. Fixing 0.1 to unlock SBC XQ gets most of the quality for none of the trouble. |
| Pico 2 W | Same CYW43439 radio, so no Bluetooth gain. The M33 and 520 KB SRAM would help decode headroom, but nothing here is currently CPU-bound. |

---

## Suggested order

1. **0.1, 0.2, 0.3** — bugs, independent of everything else, and 0.1 gets you
   SBC XQ. One commit each so a regression bisects cleanly.
2. **Tier 3 `-Wall -Wextra` and the build stamp** — same sitting, they support
   everything after.
3. **1.4** — two lines, hardens the audio path for every future source.
4. **1.2 and 1.3** — the rate-agility work. Before the next personality, not
   after; retrofitting time-based buffering across four sources is far worse
   than doing it across one.
5. **1.1** — the mode descriptor. Nothing in Tier 2 lands safely before this.
6. **1.5** — fault handler and watchdog, once the descriptor exists to say what
   a safe mode is.
7. **Measure** (both numbers).
8. **2.1**, then **2.2**, **2.3**, **2.4** — each behind its own flag, so board
   current can be bisected against them one at a time.
9. **2.5** — only if the measurement says the core is where the power is.

Steps 1–3 are worth doing whatever else happens. Step 4 is the one with a
deadline: it gets more expensive with every personality added.
