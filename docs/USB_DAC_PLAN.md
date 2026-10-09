# USB sound card mode: implementation plan

Goal: replace the stub in `src/usb_dac.c` with a working USB Audio Class 1
(UAC1) sound card that feeds the existing `audio_out` (I2S, 32-bit frames,
BCLK = 64fs), as the second personality behind the BOOTSEL mode switch.

Reference: [BambooMaster/usb_sound_card_hires](https://github.com/BambooMaster/usb_sound_card_hires),
itself derived from pico-playground `apps/usb_sound_card`. **This revision was
checked against the real code** (upstream `03e5eef`, 2026-03-08; pico-extras
tag `sdk-2.1.1`; Pico SDK 2.1.1), and against binaries built from this tree.
The first draft was written from a summary of upstream, and several of its
assumptions did not hold. They are corrected below and marked **(changed)**.

## Status

All steps are implemented and on `main`; none has run on hardware yet.

| Step | Commit |
|---|---|
| 0 USB stack vendored, console chosen after the mode | `c3c2bbe` |
| A Enumerate | `98f8b3d` |
| B 48 kHz playback | `1fcb13c` |
| C Rates, depths, volume, mute | `a71a1ea` |
| D Feedback | `3c9a716` |
| E Charger fallback | `1ba1021` |

Where the code differs from the text below, the code and its commit messages
are right:

- **Ring:** 4096 frames (a power of two for the index math, 32 KB), not 3345.
- **Start gate:** one buffer plus 4 ms **plus 6 ms** for the main loop's period
  and jitter. Without the extra 6 ms, 2 to 5 percent of simulated starts
  underran once.
- **Feedback (D10):** proportional only, Kp 8 (10.14 units per frame), tau
  about 2 s, filter 1/16 per pass. The setpoint is the queue on the first pass
  after the gate opens. Nominal is sent before that, after every stop or
  underrun, and whenever the stored value would not fit the current rate.
- **Unsupported rates (Step C)** are refused by stalling the status stage of
  the SET_CUR, which the vendored stack supports from a packet handler.
- **A stream generation** counter, bumped on every SET_INTERFACE and every rate
  change, restarts playback from an empty ring. It catches an idle-and-back
  between two loop passes.
- **Step E** does not use a "no enumeration within 2 s" rule; that would drop
  TVs and TV boxes, which power their ports long before they enumerate. It
  recognises chargers electrically instead: D+ shorted to D- reads as line
  state SE1 with our pull-up, which no host holds. A plain timeout exists as
  `-DUSB_DAC_NO_HOST_FALLBACK_MS`, off by default.
- **Host-testable code** lives in `src/usb_dac_pcm.h`: unpack, ring math, gain
  table, feedback filter and controller, charger decision.

---

## 1. Will it work with my devices?

UAC1 is the most widely supported USB audio class there is, so whether a host
works depends on the host, not on this implementation. Where a host has no USB
audio output at all, no firmware can change that. The Bluetooth mode covers
those hosts, which is the point of having two modes.

| Host | Expectation | Evidence |
|---|---|---|
| Linux (ALSA, PipeWire, PulseAudio) | **Works.** `snd-usb-audio` handles UAC1 async with a 3-byte feedback endpoint. | Long-standing kernel support; Rockbox's UAC1 async device works on Linux. |
| Android phones/tablets (OTG) | **Works.** Android uses the same Linux kernel driver. Needs an OTG adapter. | Upstream author tested a Pixel 6a on Android 15. |
| macOS | **Works.** | Rockbox reports its async feedback works on macOS. |
| Windows 10/11 | **Probably works. Test the feedback specifically.** The UAC1 driver loads with no install. Whether it follows an async feedback endpoint is less certain. | Upstream author tested Windows 11. One wdmaudiodev report says UAC1 feedback works on Windows 10. Rockbox says Windows does not work for them, but their device runs UAC1 over high speed, which is outside the spec. Microsoft's docs do not settle it. |
| NVIDIA Shield TV | **Should work.** | NVIDIA's user guide lists USB DACs as supported (up to 24/192). |
| Chromecast with Google TV, Google TV Streamer | **Unlikely.** | USB audio routing reportedly disappeared after the Android 12 update. The Streamer's documented USB uses do not include audio. |
| Sony Bravia (Android/Google TV) | **Unreliable.** | Sony support named HDMI ARC, optical and analog as the only supported outputs. One owner reported clicks over a USB DAC. |
| Fire TV Stick (all models) | **No.** | The port behaves as power-only in practice. No confirmed USB audio report. |
| Samsung (Tizen), LG (webOS) TVs | **No.** | Neither documents USB audio output. Use Bluetooth mode, or the TV's optical out into another DAC. |

Three things in this plan exist specifically to widen that list:

- **16-bit is alt setting 1, and every alt offers 48 kHz.** TV boxes and
  phones are 48 kHz-centric, and some hosts take the first format offered.
- **`bMaxPower` matches what the board actually draws.** Upstream declares
  500 mA. iPhones and iPads with Lightning refuse accessories that ask for more
  than about 100 mA, and so do some bus-powered hubs and OTG Y-cables. Measure
  the Pico W plus the DAC board in this mode, then declare that plus a margin.
- **Robust when feedback is ignored.** If a host does not follow the feedback
  endpoint, the ring buffer drifts. It must then drop or pad audio and count
  it, never corrupt it or wedge (Step B). Step D's log tells you whether a host
  follows the feedback.

Sources for the TV rows are listed at the end. They are mostly vendor support
pages and forum reports. The upstream author's tests and the Linux/Android
behaviour are the solid part.

---

## 2. How it fits the mode-switch design

The reboot-per-mode design is what makes this tractable. Each boot owns the USB
controller outright, so the Bluetooth sink and the sound card never share it.

- **Mode first, then stdio.** `main()` calls `stdio_init_all()` before the
  mode is known. With `PAB_STDIO=usb`, that brings up TinyUSB, which claims
  the USB controller and its interrupt. In the new flow, `app_mode_current()`
  runs first (it only reads a scratch register and flash, and prints nothing).
  In USB DAC mode only the UART console is initialised (if linked); every other
  mode calls `stdio_init_all()` as today.
- **One USB stack per boot.** Bluetooth mode may use TinyUSB (USB console).
  USB DAC mode uses pico-extras' `usb_device`. Both are linked into one image.
  That only works with the interrupt-vector fix in decision D3. **Without it, a
  default build panics at boot in Bluetooth mode, before the watchdog is
  armed.**
- **Persistence and switching are unchanged.** `app_mode.c` and the
  BOOTSEL logic are not touched. The USB DAC loop already calls
  `mode_button_poll()`, so holding BOOTSEL there reboots into Bluetooth mode.
  The watchdog reset drops the USB pull-up, so the host sees an unplug.
- **`CONN_PIN` (GP26) means the same thing in both modes:** high while a
  stream is playing. In USB mode that means an alt setting other than 0 is
  selected and packets are arriving. Anything wired to GP26, such as an LED or
  an amplifier trigger, keeps working.
- **No onboard LED in USB mode** (it is on the radio chip, which stays off).
  This is already documented in the README.
- **Watchdog:** armed in `main()` and fed from the USB DAC loop, as the
  stub does today. USB work is interrupt-driven, so a USB interrupt storm
  starves the loop and the watchdog catches it.
- **Flashing during development:** in USB DAC mode there is no USB console, so
  `picotool` cannot reboot the board into BOOTSEL. Either hold BOOTSEL while
  plugging in, or switch to Bluetooth mode first.

---

## 3. What upstream actually is

Verified against `usb_sound_card_hires` `03e5eef`:

| Item | Fact | Consequence for PAB |
|---|---|---|
| `usb_sound_card.c` licence | Header: **Copyright (c) 2020 Raspberry Pi (Trading) Ltd., BSD-3-Clause**. The repo's MIT LICENSE covers BambooMaster's changes. | The first draft said "keep upstream's MIT header". Wrong. See D12. |
| `lufa/*.h` | LUFA licence (Dean Camera, permissive, notice required). | Vendor with its licence, or write the structs. See D12. |
| pico-extras | A **fork**, `BambooMaster/pico-extras` branch `usb_sound_card_hires`, with one change (`993442c`): `struct usb_buffer` `data_len`/`data_max` go from `uint8_t` to `uint16_t`. | Stock pico-extras truncates any packet over 255 bytes. Only 16-bit at 44.1/48 kHz fits; 24-bit at 48 kHz (294 bytes) does not. **The patch is mandatory.** |
| Stack size config | `PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE=3` (1024-byte iso buffers), `PICO_USBDEV_MAX_DESCRIPTOR_SIZE=256`, `PICO_USBDEV_USE_ZERO_BASED_INTERFACES=1`. | Keep all three. |
| Config descriptor | **244 bytes** (measured with this toolchain) against the 256-byte buffer. | Above 256, Release builds `memcpy` past a static buffer, because the only guard is an `assert`. Add a `static_assert`. |
| DPRAM layout | Two double-buffered iso endpoints at stride 1024 reserve 0x180 + 0x800 + 0x800 = 0x1180 bytes, more than the 4 KB DPRAM. | Works in Release only because the feedback endpoint uses 3 bytes of its second buffer (at 0xD80). **A Debug build trips `assert(next_buffer_offset <= USB_DPRAM_MAX)`.** Build Release, or patch per-endpoint strides (optional, Step 0). |
| Endpoint sizing | `usb_interface_init()` sizes the endpoints from the alt-1 descriptor, for all alts. | **Every alt must use the same `wMaxPacketSize` (582).** Do not "optimise" it per alt. |
| Alt settings | 1 = 16-bit, 2 = 24-bit (both 44.1/48/88.2/96), 3 = 32-bit at 44.1/48 only. | 32-bit at 88.2/96 kHz was removed upstream (`d1ba052`, "feedback fails behind a USB hub"). Its 776-byte packets exceeded the 582-byte `wMaxPacketSize`. Lesson: size `wMaxPacketSize` for (rate/1000 + 1) × bytes per frame. |
| Feedback | `_as_sync_packet`: target = 1.5 ms of queued samples; `adjust = ((target - level) * freq / target) >> 7`; value `((freq + adjust) << 14) / 1000` in 10.14, clamped to ±freq/128, sent as 3 bytes LSB first. `bRefresh = 3` (every 8 ms). | The format and clamp are right. The *level* PAB must measure is different (D10). |
| Controls | Volume GET_CUR/MIN/MAX/RES (−90…0 dB, 1 dB steps). Mute is stored but **never applied**. SET_CUR frequency outside the four rates silently becomes 44.1 kHz. | Apply mute. Reject unknown rates. |
| Rate change | `_audio_reconfigure()` reprograms the I2S clocks **inside the USB interrupt**. | PAB defers it to the main loop (D8). |
| Playback | core1 pump, 0.5 ms chunks, low-jitter `set_sys_clock_khz` per rate family. | Not taken: `audio_out` owns output and clocks. |
| `interpolation` branch | Uses RP2350 DSP. | Not applicable: Pico W is RP2040. |
| Typo | `PICO_USBDEV_ENABLE_DEBUG_TRAgCE` | Harmless. Do not copy. |

---

## 4. Decisions

Items marked **(changed)** or **(new)** differ from the first draft. The rest
stand as written. Do not reopen any of them without hardware evidence.

**D1. UAC1, not UAC2.** Widest support (Windows without a driver install,
Linux, macOS, iOS, Android, Android TV boxes). UAC2 at full speed adds risk on
TVs and older Windows, for no benefit at ≤ 96 kHz/24-bit.

**D2. Stack: pico-extras `usb_device`, vendored into the repo with two patches.
(changed)** The first draft added pico-extras as an external dependency
(`PICO_EXTRAS_PATH`). Stock pico-extras cannot carry 24-bit (D2a). The upstream
fork pins an unrelated tree. The code we need is about 1,900 lines, unchanged
since 2021 (`f5c7be9`) and identical from tag `sdk-2.1.1` through `sdk-2.3.1`.
Vendoring it the way `pico_i2s_pio/` is vendored keeps the build at one
checkout (no new path, no new option) and keeps the patches visible.
- **D2a.** `struct usb_buffer`: `uint8_t data_len, data_max` → `uint16_t`.
- **D2b.** The interrupt handler rename. See D3.
- **TinyUSB status (re-checked).** TinyUSB added UAC1 device support
  ("Add basic UAC1 support", 2025-09-30, first released in 0.20.0). Every Pico
  SDK through 2.3.1 still bundles TinyUSB 0.18.0, which is UAC2-only. Moving
  PAB to TinyUSB would mean overriding the SDK's TinyUSB, a riskier change than
  this whole feature. Revisit only when an SDK release ships TinyUSB ≥ 0.20.
  That would also allow a composite UAC + CDC console.

**D3. The USB interrupt vector belongs to whichever stack the mode starts.
(new, critical)** pico-extras defines `void __isr __used isr_usbctrl(void)`.
The SDK `#define`s `isr_usbctrl` to `isr_irq5`, the strong override of the
vector-table entry. Merely compiling `usb_device.c` into the image therefore
takes over `USBCTRL_IRQ`, even if nothing ever calls it. Confirmed with
`arm-none-eabi-nm`: `isr_irq5` becomes `T` at pico-extras' handler instead of
the weak default. TinyUSB, which the USB console uses in Bluetooth mode,
registers with `irq_add_shared_handler()`, which runs
`hard_assert(vtable_handler == __unhandled_user_irq)`. That assert is active in
Release, so the result is a **panic in `stdio_init_all()`, before the watchdog
is armed: the box hangs until power-cycled.**
Fix, in the vendored source: rename the handler to
`static void __isr usb_device_irq_handler(void)` (no `__used`, not
`isr_usbctrl`). In `usb_device_start()`, call
`irq_set_exclusive_handler(USBCTRL_IRQ, usb_device_irq_handler)` before
`irq_set_enabled(USBCTRL_IRQ, true)`. Bluetooth mode never calls
`usb_device_start()`, so the vector stays at the SDK default until TinyUSB
claims it. Verified: with the handler renamed, `isr_irq5` stays `W`
(→ `__unhandled_user_irq`).
**Regression check for every step:**
`arm-none-eabi-nm build/picow-a2dp.elf | grep -w isr_irq5` must print `W`.

**D4. Console in USB DAC mode: UART or nothing. (changed)** The mode is
resolved before stdio. In USB DAC mode, call `stdio_uart_init()` if
`LIB_PICO_STDIO_UART` is defined, and never `stdio_usb_init()`. Other modes
call `stdio_init_all()` as today. No CMake option is needed, and the default
`PAB_STDIO=usb` build keeps its Bluetooth-mode console. Document in the README
that the sound card mode has a console only with `PAB_STDIO=uart`.

**D5. Data path: own unpack, int32 ring. (changed)** The first draft pointed at
`pico_i2s_pio/i2s_uac.c`. That file is **not compiled today** (CMake uses only
`i2s.pio`). It writes planar L/R buffers where `audio_out` wants interleaved,
pulls in `i2s_core.h`, and shifts signed values in ways the C standard leaves
undefined. Write a small unpack in `usb_dac.c` instead. Shift as unsigned, the
way `btstack_audio_pico_i2s.c` does:
- 16-bit: `(int32_t) ((uint32_t) (uint16_t) s << 16)`
- 24-bit: `(int32_t) ((uint32_t) b0 << 8 | (uint32_t) b1 << 16 | (uint32_t) b2 << 24)`

The USB interrupt unpacks each OUT packet straight into a single-producer,
single-consumer ring of interleaved `int32_t` frames. The `audio_out` fill
callback (main loop) copies out of it. On one core, `volatile` indices suffice:
the producer writes data first and advances the write index last. Keep the
unpack and the feedback math in functions with no SDK includes, so they can be
checked with host `gcc` in a scratch directory.

**D6. Formats: alt 1 = 16-bit, alt 2 = 24-bit, each at 44.1/48/88.2/96 kHz.
No 32-bit alt in v1. (changed)** Integer 32-bit carries nothing over 24-bit
from any real source. It costs a third alt (55 descriptor bytes, one more host
matrix column), and it is the alt upstream had to cut back. Add it later only if
a host demands it. **One `wMaxPacketSize` = 582 for every alt** (24-bit
× 97 frames), because of the endpoint-sizing fact in §3.

**D7. Default rate 48 kHz (changed from upstream's 44.1).** It is what TV
boxes, Android and PipeWire use by default. `GET_CUR` on the frequency control
must return the actual current rate.

**D8. Rate and alt changes are requests from the interrupt, acted on by the
main loop. (new)** USB control requests and SET_INTERFACE arrive in the USB
interrupt. `audio_out_set_sample_rate()` refuses while a stream runs, and it
prints. The interrupt therefore only records `pending_alt` / `pending_rate`
and completes the control transfer. The loop then applies them in this order:
`audio_out_stop()` → flush the ring → `audio_out_set_sample_rate()` → re-arm
the start gate (D9) → `audio_out_start()`.
Hosts typically send SET_INTERFACE (alt ≠ 0) *before* SET_CUR(rate), and
packets may arrive before the loop runs. That is why the ring is flushed on
every applied change.

**D9. Start gate and ring size. (new)** `audio_out_start()` immediately fills
all four free buffers (46 ms). If the ring is short at that moment, audio and
padding interleave, and the stream starts with a stutter. So the fill callback
returns **whole buffers of silence** until the ring holds `start_threshold` =
one `audio_out` buffer + 4 ms. From then on it drains the ring and pads only
on a true underrun, which it counts.
Ring capacity = 3 × `audio_out_frames_per_buffer()` at 96 kHz = 3345 frames
(26.8 KB as `int32_t` stereo). This tree uses about 101 KB of .bss today, so
there is room. Overrun (ring full): drop the incoming packet and count it.
In steady state the ring swings between about 4 ms and one buffer + 4 ms (it
loses a whole buffer at each refill), so the total latency is about 50 to
62 ms.

**D10. Feedback measures the whole queue, from the main loop. (new)** The
ring alone is a sawtooth: `audio_out` takes a whole buffer (11.6 ms) at a
time. `ring + audio_out_queued_frames()` is smooth, because moving frames from
the ring into `audio_out` does not change the sum. Sample it in the main loop
right after `audio_out_service()` returns, with interrupts disabled across the
two reads. Read it there, not in the feedback interrupt: there, a refill caught
mid-copy has frames that are counted nowhere for a moment. Filter it, compute
the 10.14 value, and store it in a `volatile uint32_t` that the feedback
endpoint's handler copies out. Until the stream has filled, send the nominal
value. Linux detects the feedback format from the first packets it receives.
Tuning is Step D's job.

**D11. Volume and mute.** Advertise −90…0 dB in 1 dB steps, as upstream.
Convert dB to the `audio_out_set_volume()` gain (65536 = unity) in the main
loop, from a 91-entry table or `powf`. At −90 dB the 16-bit gain is about 2,
which is coarse but acceptable down there. Mute sets gain 0 and remembers the
volume. Requests addressed to channels other than master (CN ≠ 0) STALL.

**D12. Descriptor identity and licensing. (changed)**
- VID/PID: keep upstream's `0x2E8A:0xFEDD` for now (Raspberry Pi VID,
  pico-playground's sound-card PID). It must differ from the SDK console's
  `0x2E8A:0x000A`. Windows caches descriptors per VID/PID/`bcdDevice`, so
  **bump `bcdDevice` whenever the descriptors change during development**, or
  uninstall the device in Device Manager. Raspberry Pi hands out free PIDs
  under its VID (github.com/raspberrypi/usb-pid) if this ever ships.
- Strings: manufacturer "xatxa4" (or as you prefer), product "PAB", serial from
  `pico_get_unique_board_id_string()` (link `pico_unique_id`).
- `bmAttributes` 0x80 (bus-powered), `bMaxPower` measured (§1), not 0xFA.
- **Licensing:** `src/usb_dac.c` becomes a derived file. Its header carries the
  Raspberry Pi 2020 copyright (BSD-3-Clause, full text), BambooMaster 2025
  (MIT) and xatxa4 2026 (MIT), with `SPDX-License-Identifier: BSD-3-Clause AND
  MIT` and a line saying which parts follow which source. This mirrors how
  `audio_out.c` credits pico-i2s-pio. The vendored `pico_usb_device/` keeps its
  BSD-3-Clause headers and gets pico-extras' `LICENSE.TXT`. If the LUFA headers
  are vendored, they keep their licence file. Add rows to the README licensing
  table and credit list.

**Unchanged from the first draft:** keep `audio_out`'s clocks
(no `set_sys_clock_khz`); never change the rate while a stream runs; reject
unsupported rates; −Wall −Wextra clean on `PAB_SOURCES` (the vendored code is
not added to `PAB_SOURCES`); the radio is never brought up in this mode.

---

## 5. Models

| Step | Model | Why |
|---|---|---|
| 0, A | **Sonnet 5.5** (`claude-sonnet-5-5`) | Mechanical and fully specified: vendoring, CMake, descriptors from a template, a reordering in `main()`. Every claim is checkable by building and running `nm`. |
| B, C | **Sonnet 5.5, high effort** | Concurrency between the USB interrupt and the main loop (ring, start gate, deferred rate change). It is specified here in enough detail that the model implements rather than designs. High effort buys the careful re-reading this needs. |
| D | **Opus 5.5** (`claude-opus-5-5`) | A control loop, tuned against measurements: filter, gain, clamp, stability, reading long-run logs and deciding what they mean. The only step that is design rather than implementation. |
| Hardware triage, any step | **Opus 5.5** | "Does not enumerate on Windows", "clicks every 40 s on the Shield". Diagnosis from usbmon/Wireshark captures and descriptor dumps. Switch to Opus for the diagnosis, then back to Sonnet for the fix. |
| Docs-only edits | Sonnet 5.5 | Haiku 5.5 can do pure wording edits, but the README and ADOPTION.md entries need technical judgement. |

Fable 5.1 is not needed for any step. It costs 2.5× Opus 5.5 per token, and
nothing here is at the frontier.

**Starting a step** (paste into a fresh session on `main`):

> Read `docs/USB_DAC_PLAN.md` (sections 2–7 are binding), then do **Step X only**.
> Follow the build recipe in section 6 and build after every change. Stop and
> report instead of improvising if the code contradicts the plan. Commit once,
> in the repository's message style, and say exactly what was compiled and what
> was not run on hardware.

**For the Sonnet sessions, specifically:**

- **Read first:** `src/usb_dac.c`, `src/main.c`, `src/audio_out.h`,
  `src/btstack_audio_pico_i2s.c` (the reference pattern for a fill callback,
  underrun reporting and volume mapping), `src/app_mode.h`, `src/mode_button.h`,
  and `CMakeLists.txt`.
- **Do not read or touch** `a2dp.c`, `avrcp.c`, `bt.c`, `sdp.c`,
  `audio_out.c` internals, or `app_mode.c`. If `audio_out.h` lacks something,
  stop and report rather than adding to it.
- **Conventions:** console lines are `printf("USB DAC         : ...\n")` (tag
  padded to 16 characters). Comments say *why*, in the house tone. Shift
  signed samples as unsigned. Counters are `volatile uint32_t`, reported at
  most once a second when they change, as `driver_timer_handler_sink` does.
- **Commit style:** imperative, sentence case, no prefix, says what changes for
  the user. Examples from history: "Restart on a hang, a fault or a dead
  controller, with a watchdog"; "Busy-wait in the mode switch, which runs
  inside an interrupt". One commit per step. Update the status table in
  `docs/ADOPTION.md` and the README in the same commit when behaviour changes.
- **Never:** change Bluetooth-mode behaviour, weaken the watchdog, add a CMake
  option this plan does not ask for, or use `assert` as the only guard for
  something that would corrupt memory in Release.

---

## 6. Build and check recipe (verified 2026-10-08 in the cloud container)

The cloud container can build the firmware; the first draft assumed it could
not. Run from the repository root, with `$S` set to the session's scratchpad:

```sh
apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
git clone -q --depth 1 --branch 2.1.1 https://github.com/raspberrypi/pico-sdk.git $S/pico-sdk
git -C $S/pico-sdk submodule update --init --depth 1 lib/btstack lib/cyw43-driver lib/tinyusb lib/lwip
cmake -S . -B $S/build -G Ninja -DPICO_SDK_PATH=$S/pico-sdk -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build $S/build -j8
```

About 350 MB and a few minutes; picotool is fetched automatically. Today's tree
builds with no warnings: text 427,852 B, bss 103,752 B.

Checks after every step:

1. The build finishes with **zero warnings from `src/`** (vendored code may warn;
   do not fix vendored warnings by editing beyond the documented patches).
2. `arm-none-eabi-nm $S/build/picow-a2dp.elf | grep -w isr_irq5` prints `W`.
3. Build once more with `-DPAB_STDIO=uart` and once with `none`. All three
   configure and link.
4. `arm-none-eabi-size` on the ELF: note the .bss change in the commit message.
5. Pure functions (unpack, feedback math, dB table) are exercised with host
   `gcc` in `$S` against hand-computed values (see §9). Do not commit a test
   harness.

What the container cannot do: run it. Every hardware acceptance line below is
for you, the user, to check with the board.

---

## 7. Steps

Each step is one session and one commit. Stop at the end of a step.

### Step 0: USB stack in, nothing uses it (Sonnet 5.5)

- Vendor from pico-extras tag `sdk-2.1.1` (`f05d4f7`) into `pico_usb_device/`:
  `usb_device/usb_device.c`, `usb_device/usb_stream_helper.c`,
  `usb_device/include/pico/{usb_device.h,usb_device_private.h,usb_stream_helper.h}`
  → `pico_usb_device/{*.c,include/pico/*.h}`;
  `usb_common/include/usb/usb_common.h` → `pico_usb_device/include/usb/`;
  `LICENSE.TXT` → `pico_usb_device/LICENSE`. Clone into `$S`, then copy.
- Apply D2a and D3. Mark each change with a `// PAB:` comment. Add a
  `pico_usb_device/README.md`: origin tag and commit, the two patches, and
  that BambooMaster's fork (`993442c`) is the source of D2a.
- Optional, same step: give each iso endpoint its own stride (a power of two,
  at least 128 and at least its `wMaxPacketSize`) and program
  `DOUBLE_BUFFER_ISO_OFFSET` from it instead of from the global, so Debug
  builds stop asserting. Skip if it grows past about 30 lines.
- `CMakeLists.txt`: add the vendored sources and include dirs to the target,
  **not** to `PAB_SOURCES`. Link `pico_fix_rp2040_usb_device_enumeration`,
  `hardware_irq` and `pico_unique_id`. Add the three `PICO_USBDEV_*`
  definitions from §3 **target-wide**: they change struct layouts and array
  sizes, so every translation unit must agree.
- `main.c`: resolve the mode before stdio, per D4.
- Accept (container): §6 checks 1–4 pass, `isr_irq5` is `W` in all three
  `PAB_STDIO` builds, `usb_device_irq_handler` is absent (unreferenced).
- Accept (hardware): Bluetooth mode with `PAB_STDIO=usb` boots and its console
  works exactly as before. USB DAC mode behaves like the old stub.

### Step A: enumerate (Sonnet 5.5)

- `src/usb_dac.c`: device, configuration and string descriptors per D6/D7/D12.
  Base the layout on upstream's `struct audio_device_config`, minus alt 3.
  Either vendor `lufa/AudioClassCommon.h` + `lufa/StdDescriptors.h` with their
  licence (they compile alongside `pico/usb_device.h` upstream), or write the
  ~10 packed structs yourself. Either way, add:
  `static_assert(sizeof(struct audio_device_config) <= PICO_USBDEV_MAX_DESCRIPTOR_SIZE)`.
  Expect about 189 bytes without alt 3.
- AC interface: input terminal (USB streaming, 2 ch) → feature unit (master
  mute + volume) → output terminal (speaker). AS interface: alt 0 empty, alt 1
  16-bit, alt 2 24-bit. Each has: iso OUT EP 0x01 `bmAttributes` 0x05
  (async), `wMaxPacketSize` 582, `bInterval` 1, `bSynchAddress` 0x82; the
  class-specific EP with `bmAttributes` 0x01 (sampling frequency control); and
  the feedback EP 0x82 with `bmAttributes` 0x11, `wMaxPacketSize` 3,
  `bInterval` 1, `bRefresh` 3.
- Control requests: GET_CUR/MIN/MAX/RES for volume, GET_CUR for mute,
  GET_CUR for frequency, SET_CUR for all three. Store the values only; nothing
  is applied yet. Upstream's `ac_setup_request_handler`,
  `_as_setup_request_handler`, `do_set_current` and `audio_cmd_packet` are the
  template. Keep `audio_cmd_packet`'s length check, and check `wIndex` (entity
  or endpoint) as well.
- Feedback endpoint: the handler sends the nominal value for the current rate.
- OUT endpoint: the handler accepts and discards packets (re-arm with
  `usb_grow_transfer` + `usb_packet_done`, as upstream).
- `usb_dac_run()`: `audio_out_init(48000)`, init and start the device, then the
  existing loop.
- Accept (container): §6 checks; `static_assert` holds; `isr_irq5` still `W`.
- Accept (hardware): Linux `aplay -l` lists PAB; `lsusb -v -d 2e8a:fedd`
  shows two alts with four rates each. Windows shows a "PAB" speaker with no
  driver prompt. Holding BOOTSEL switches back to Bluetooth.

### Step B: fixed-rate playback (Sonnet 5.5, high effort)

- The ring, unpack and start gate per D5/D9; 48 kHz only, both depths.
- Alt changes go through the deferred path of D8 (rates come in Step C).
  `CONN_PIN` is high while playing. If no OUT packet arrives for 50 ms with
  alt ≠ 0, treat the stream as stopped: some hosts stop sending without
  selecting alt 0.
- Counters: underruns (fill padded), overruns (packet dropped), packets
  discarded during a change, and `audio_out_underruns()` /
  `audio_out_late_irqs()`, printed as in Bluetooth mode.
- The feedback endpoint still sends nominal. Over minutes the ring will drift
  by the clock mismatch; at 48 kHz and 100 ppm that is about 5 frames per
  second. The counters must show it cleanly (dropped or padded, never noise).
- Accept (hardware): clean audio for several minutes at 48/16 and 48/24 on
  Linux. Starting and stopping playback repeatedly gives no stutter at start
  and no stuck stream. Underrun/overrun counts appear at the expected slow
  rate, with no crackle beyond that.

### Step C: rates, depths, controls (Sonnet 5.5, high effort)

- SET_CUR frequency: accept exactly the four rates; STALL anything else.
  Apply through D8.
- Volume and mute per D11, applied in the main loop.
- Accept (hardware): switching 44.1↔48↔96 and 16↔24 bits from `pavucontrol`
  or Windows' Advanced tab works without a reboot or a stuck stream. The
  console shows `I2S : <rate> Hz, divider …` for each change. Volume and mute
  work from the host's mixer.

### Step D: feedback and drift (Opus 5.5; the only step that needs it)

- Implement D10. Starting point: an IIR filter over 32–64 ms on
  `ring + audio_out_queued_frames()`. Proportional control towards a
  setpoint taken from the total at the moment the start gate opens; with the
  feedback holding it there, the ring keeps its D9 margin. Clamp to ±1/128 as upstream; the real mismatch is
  under about 300 ppm (see §9). Add a small integral term only if the
  proportional error is large.
- Log once a second: the filtered level, its error and the feedback value in
  ppm, so a host that ignores feedback is visible. Its level walks at a
  constant slope regardless of the value sent.
- Contingency if Windows ignores UAC1 feedback (decide from the logs, not in
  advance): make the device follow the host instead. That means trimming the
  PIO divider by its 1/256 step (about 190 ppm at 48 kHz) with a duty-cycle
  dither. It needs a small `audio_out` API, and that is an `audio_out` change
  to design together with the user.
- Test: 1 h+ at 44.1 and 96 kHz on Linux and Windows; ring fill bounded;
  under/overruns zero. On Linux,
  `watch -n1 cat /proc/asound/card*/stream0` shows the momentary frequency the
  host derives from the feedback. Record results in the style ADOPTION.md uses
  for earlier audits.

### Step E (optional): charger detection (Sonnet 5.5)

A box left in USB mode and powered from a phone charger behind a TV is silently
useless. If no bus reset is seen within about 2 s of enabling the pull-up, no
host is present: reboot into Bluetooth for this boot only, without writing
flash. This needs a non-persisting variant of `app_mode_switch_to()`; a
watchdog scratch flag already carries the mode across the reset. Only after
Step D, and only if you want it.

---

## 8. Host test matrix (when hardware is available)

| Host | Check | How |
|---|---|---|
| Linux | enumerate, all rates/depths, 1 h drift | `aplay -l`, `lsusb -v`, `/proc/asound/card*/stream0` |
| Windows 11 | enumerate without driver, feedback followed | Sound settings → Advanced format; USB Tree View for descriptors; Step D log |
| macOS | enumerate, rates | Audio MIDI Setup |
| Android phone | OTG, 48 kHz playback | `adb shell dumpsys media.audio_flinger` shows the USB output |
| NVIDIA Shield | USB audio output | Settings → Display & Sound |
| Chromecast/Google TV, Fire TV | expected not to work | try once, record the result in README |
| iPhone/iPad (if any) | power acceptance | `bMaxPower` matters here |

Record results in the README next to the Bluetooth lip-sync host list, as
*tested* vs *expected*.

---

## 9. Reference numbers

`clk_sys` = 125 MHz, buffers 11,610 µs, as built today.

| Rate | PIO divider (16.8) | Rate error | `audio_out` buffer | Feedback nominal (10.14) | Max packet 16 / 24 / 32 bit |
|---|---|---|---|---|---|
| 44.1 kHz | 22 + 37/256 | −12 ppm | 512 frames | 0x0B0666 | 180 / 270 / 360 B |
| 48 kHz | 20 + 88/256 | +64 ppm | 557 frames | 0x0C0000 | 196 / 294 / 392 B |
| 88.2 kHz | 11 + 18/256 | +165 ppm | 1024 frames | 0x160CCD | 356 / 534 / 712 B |
| 96 kHz | 10 + 44/256 | +64 ppm | 1115 frames | 0x180000 | 388 / 582 / 776 B |

- Stock pico-extras packet limit: 255 B. Upstream `wMaxPacketSize`: 582 B.
  Full-speed iso maximum: 1023 B.
- Feedback bytes go out LSB first: 48 kHz is `00 00 0C`.
- Descriptor: 244 B upstream (3 alts), about 189 B for D6, limit 256 B.
- RAM today: .bss 103,752 B of 264 KB. Ring per D9: +26.8 KB.

---

## 10. Docs to update as the steps land

- `README.md`: the Modes section (the sound card is real; console only with
  `PAB_STDIO=uart`; flashing tip), the tested-host list, the licensing table
  (`pico_usb_device/` BSD-3-Clause; `usb_dac.c` BSD-3-Clause AND MIT; LUFA if
  vendored), and credits (Raspberry Pi pico-extras/pico-playground; BambooMaster
  `usb_sound_card_hires`).
- `docs/ADOPTION.md`: status table rows per step. In 2.1/2.3 (clock gating),
  note that USB DAC mode needs `CLK_SYS_USBCTRL`, `CLK_USB_USBCTRL` and
  PLL_USB, and that 1.1's `preferred_sys_clk_hz` would let this mode alone run
  `clk_sys` at an MCLK multiple later (upstream's low-jitter mode), since it
  has no radio to disturb.

---

## Sources

Code (all read for this revision): `BambooMaster/usb_sound_card_hires`
`03e5eef`; `BambooMaster/pico-extras` `f41ae02` (branch `usb_sound_card_hires`);
`raspberrypi/pico-extras` `sdk-2.1.1`; `raspberrypi/pico-sdk` 2.1.1
(`hardware_irq/irq.c`, `pico_stdio_usb/stdio_usb.c`); `hathach/tinyusb` 0.18.0
`dcd_rp2040.c`, and master `audio_device.c` for UAC1.

Host support:
- NVIDIA, [SHIELD TV USB Audio Setup](https://support-shield.nvidia.com/shield-tv-pro-user-guide/USB_Audio_Setup.htm)
- [USB audio routing on Chromecast not working after Android 12](https://piunikaweb.com/2022/11/25/usb-audio-routing-on-chromecast-not-working-after-android-12/)
- [Google TV Streamer USB peripherals](https://www.aftvnews.com/tag/google-tv-streamer/page/4/)
- [Sony community: USB DAC on Bravia Android TV](https://community.sony.dk/t5/android-tv/usb-audio-support-usb-dac-for-sony-bravia-android-tv-kd-49xh8096/td-p/3881043)
- [Rockbox USB-DAC (UAC1 async; Windows caveat)](https://www.rockbox.org/wiki/UsbDAC)
- [wdmaudiodev: asynchronous sink feedback (UAC1 feedback on Windows 10)](https://www.freelists.org/post/wdmaudiodev/Asynchronous-sink-feedback-question,8)
- [Microsoft Q&A: explicit feedback with usbaudio2.sys](https://learn.microsoft.com/en-us/answers/questions/472332/explicit-feedback-for-asynchronous-audio-with-usba)
- [Android USB digital audio](https://source.android.com/docs/core/audio/usb)
