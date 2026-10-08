# USB sound card mode: implementation plan

Goal: replace the stub in `src/usb_dac.c` with a working UAC1 sound card that
feeds the existing `audio_out` (I2S, 32-bit frames, BCLK = 64fs). Mode switching
(`app_mode`, BOOTSEL, flash/scratch persistence) already exists and is not
touched.

Reference: https://github.com/BambooMaster/usb_sound_card_hires (MIT), itself
derived from pico-playground `apps/usb_sound_card`. Its `usb_sound_card.c` is the
source for descriptors and control handling. Verify details against the real
file; this plan was written from a summary of it.

## Decisions (already made, do not reopen)

- **UAC1, not UAC2.** Widest host support (Windows without extra drivers, Linux,
  macOS, iOS, Android, Android/Google/Fire TV). UAC2 adds risk on TVs and older
  Windows for no benefit at <= 96 kHz.
- **Stack: pico-extras `usb_device`** (`pico/usb_device.h`), as upstream does.
  **Not TinyUSB**: TinyUSB's audio class driver is UAC2 only. A custom TinyUSB
  class driver for UAC1 is possible but more work; only fall back to it if
  pico-extras cannot be added to the build.
- **Take from upstream:** descriptors, alt-setting to bit-depth mapping, rate and
  volume/mute control requests, feedback packet encoding.
- **Do not take:** its i2s queue, its core1 playback loop, `set_sys_clock_khz`.
  `audio_out` already owns output, clocks and the PIO divider.
- **Data path:** USB OUT packet -> `i2s_unpack_uacdata()` (already vendored in
  `pico_i2s_pio/i2s_uac.c`) or an equivalent unpack into 32-bit left-aligned
  frames -> ring buffer -> `audio_out` fill callback.
- **Rates:** 44.1, 48, 88.2, 96 kHz (`PICO_AUDIO_I2S_MAX_SAMPLE_RATE` is 96000).
  Bit depth: 16 and 24 for all rates; 32-bit only at 44.1/48 as upstream, and
  enforce that per alt setting (upstream does not).
- **Rate change contract:** `audio_out_stop()`, `audio_out_set_sample_rate()`,
  `audio_out_start()`. Never change rate while a stream runs.
- **Licensing:** keep upstream's MIT copyright header on any copied code. Mark
  such files accordingly (see how `pico_i2s_pio/` is attributed).

## Constraints to respect

- USB stdio and a UAC device both want the USB peripheral. `main()` calls
  `stdio_init_all()` before the mode is known. In the USB DAC mode the console
  must not claim USB. Build with `-DPAB_STDIO=uart|none`, or (preferred, if
  simple) skip USB stdio init when the mode is `APP_MODE_USB_DAC`. Document the
  choice in the README; do not break the default Bluetooth-mode console.
- The radio is never brought up in this mode (already the case). The LED is on
  the radio chip, so there is no LED feedback in this mode.
- Feed `watchdog_update()` from the main loop. USB work is interrupt-driven, so
  the loop only services `audio_out_service()`, `mode_button_poll()` and the
  watchdog, as the stub does today.
- No Pico SDK is checked out in the dev container, so it cannot be built there.
  Say clearly in each commit/summary what was and was not compiled or run.
  Keep `-Wall -Wextra` clean on `PAB_SOURCES`.
- Update `docs/ADOPTION.md` where it mentions USB (clock gating: this mode
  needs `CLK_SYS_USBCTRL` and PLL_USB) rather than adding new copies of the
  same facts.

## Steps

Each step is one session and one commit. Stop at the end of a step.

### Step A: enumerate (Sonnet)

- Add pico-extras to `CMakeLists.txt` (`PICO_EXTRAS_PATH`, `pico_extras_import.cmake`
  alongside `pico_sdk_import.cmake`) and link `usb_device`. Fail with a clear
  message if it is missing; Bluetooth-only builds must not need it, so make it
  an option (`PAB_USB_DAC`, default ON if found).
- In `src/usb_dac.c`: device/config/string descriptors, UAC1 control interface
  with a feature unit (mute, volume), one streaming interface with alt 0 (idle),
  1 (16-bit), 2 (24-bit), 3 (32-bit), isochronous OUT + feedback IN endpoint.
- Stdio guard described above.
- Accept: host lists "PAB" as an audio output; no audio yet; mode switch back to
  Bluetooth still works; Bluetooth build unchanged in behavior.

### Step B: fixed-rate playback (Sonnet)

- 48 kHz, 24-bit only. Ring buffer sized in time (about 20 ms) and written from
  the OUT endpoint handler; `audio_out` fill callback reads it, writes silence on
  underrun and counts it.
- Start `audio_out` on alt != 0, stop it on alt 0.
- Feedback endpoint may send a constant nominal value for now.
- Accept: audible, clean audio for several minutes; underrun counter printed on
  stop.

### Step C: rates, depths, controls (Sonnet)

- Frequency control SET_CUR/GET_CUR on the endpoint, following the stop,
  set-rate, start contract. Reject unsupported rates instead of silently falling
  back to 44.1 as upstream does.
- Alt 1/2/3 select unpack width. Enforce the 32-bit rate limit.
- Volume (dB to `audio_out_set_volume()` gain) and mute (actually applied).
- Accept: switching rate and depth from the host mixer works without a reboot
  and without a stuck stream.

### Step D: feedback and drift (Opus; the only step that needs it)

- Compute the feedback value from the ring buffer fill level relative to a
  target (or from `audio_out_queued_frames()`), in the 10.14 format UAC1 full
  speed uses (3 bytes). Clamp the correction (upstream: about +/- freq/128) and
  smooth it so it does not hunt.
- Test: long run (1 h+) at 44.1 and 96 kHz on Linux and Windows; ring fill must
  stay bounded and underruns/overruns stay at zero. Record results in the
  ADOPTION.md style used for earlier audits.

## Host matrix to test when hardware is available

Linux (ALSA), Windows 11, macOS, Android phone, Android TV / Fire TV. Samsung
and LG TVs are expected not to offer USB audio output at all; that is not a bug
here, and Bluetooth mode is the fallback for them.
