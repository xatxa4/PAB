# PAB — Pico Audio Bridge

A Bluetooth A2DP audio sink for the Raspberry Pi Pico W, feeding an external I2S
DAC with 32-bit frames at BCLK = 64fs. Built for an ES9038Q2M, which will not
accept the 16-bit / 32fs stream the usual Pico I2S drivers produce.

A mode framework switches between personalities: the Bluetooth sink, and a USB
sound card that is being brought up step by step (see Modes). Internet radio and
an S/PDIF receiver could follow.

## Wiring

| Signal | Pin | Note |
|--------|-----|------|
| DATA / DIN | GP18 | |
| LRCLK / WS | GP20 | `PICO_AUDIO_I2S_CLOCK_PIN_BASE` |
| BCK | GP21 | clock pin base + 1 |
| MCLK | — | off by default, see `CMakeLists.txt` |
| Connection indicator | GP26 | high while a stream is established (Bluetooth) or playing (USB sound card) |

## Modes

The box has two personalities: the Bluetooth sink, and a USB sound card. The
sound card plays 16 and 24 bit stereo at 44.1, 48, 88.2 and 96 kHz, and the
host's volume and mute work; GP26 is high while it plays. The feedback that
keeps the host's clock and the DAC's together is still next, so a long stream
drifts, and the console counts the audio it drops or pads. It has only been
compiled, not yet run on a host.

**Hold BOOTSEL for about a fifth of a second to switch to the other mode.** The
onboard LED flashes three times slowly, and the Pico reboots into the new mode.

The mode is put in a watchdog scratch register, which survives the reset, and
written to flash so a cold start comes back the same way. Flash is only touched
when the mode actually changes.

Two things worth knowing:

- After the three slow flashes, the LED blinks quickly until you let go of
  BOOTSEL. That wait is deliberate: the bootrom reads the same button after the
  reset, so rebooting while it is still held would bring the Pico up as a USB
  drive instead of in the new mode. Held for more than five seconds, it reboots
  anyway — and if you are still on the button at that point, the bootloader is
  probably what you wanted.
- The LED lives on the radio chip, so it only lights in Bluetooth mode. USB
  sound card mode never brings the radio up, and switching out of it is silent.

## Build

```sh
cmake -B build -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Options: `-DPAB_STDIO=usb|uart|none` (default `usb`), which selects exactly
that console and no other. Pin assignments, I2S buffer depth (in time:
`PICO_AUDIO_I2S_BUFFER_US`), the highest supported sample rate and the Bluetooth
name are compile definitions in `CMakeLists.txt`.

In USB sound card mode the USB port belongs to the sound card, so a console
exists there only with `-DPAB_STDIO=uart`. For the same reason `picotool`
cannot reboot the board into BOOTSEL in that mode: hold BOOTSEL while plugging
in, or switch modes first.

Every build is stamped with `git describe --dirty`. It is printed at boot, and
`picotool info build/picow-a2dp.uf2` reads it off the file before flashing — a
`-dirty` suffix means uncommitted changes went in.

What to work on next, and why in that order, is in
[`docs/ADOPTION.md`](docs/ADOPTION.md).

## Lip sync

PAB tells the source how far behind the picture its audio is (AVDTP delay
reporting), measured from what is actually queued and updated while it plays.
Sources that use it hold their video back to match: Android 9 and later,
including Android, Google and Fire TV; iOS 8.2 and later; Linux with PipeWire.
Many TVs — Samsung, LG, Roku, Apple TV among them — may not, and need their
manual audio/video sync setting instead; about 90–100 ms is the place to start.
The console shows `latency measured` and `delay report` lines while streaming.

## Staying up

- **Watchdog.** A crash, a hang or a Bluetooth controller that stops working
  restarts the Pico within about 8 seconds, instead of leaving it dead until
  unplugged.
- **One source at a time.** While a phone has PAB connected for audio, PAB is
  neither discoverable nor connectable, so another phone cannot cut in. It
  opens up again when that phone disconnects or closes its audio connection.
- **A fresh start per session.** Once the last phone has disconnected after
  playing something, PAB restarts and comes back ready for the next one. A
  phone that only pauses, or switches codec, stays connected.
- **The console.** `I2S : N underruns, M late refills` appears whenever either
  count goes up, at most once a second: the times the DAC ran out of audio, and
  the times its refill came too late (during a flash write, for example) and
  it played silence instead. With a USB console, a terminal that holds the port
  open without reading holds up the box for at most 10 ms per message; the
  output is dropped.
- **Pico SDK 2.1.1** is what PAB is built and checked against. It still
  builds with 1.5.1, with a warning: that SDK's radio driver has a known bug.

---

# Licensing

This project combines code under several licences. **Read the summary at the end
before redistributing it or using it commercially.** Nothing here is legal
advice; if you intend to sell a product based on this, take proper counsel.

## Components and their terms

| Component | Copyright | Licence |
|---|---|---|
| `src/a2dp.c`, `avrcp.c`, `bt.c`, `sdp.c`, `main.c`, `btstack_audio_pico_i2s.*` | BlueKitchen GmbH; joba-1; xatxa4 | BlueKitchen BTstack licence (BSD-3-Clause **plus a non-commercial clause**) |
| `src/audio_out.c` | BambooMaster; xatxa4 | MIT |
| `src/audio_out.h`, `app_mode.*`, `mode_button.*`, `usb_dac.h`, `usb_dac_pcm.h` | xatxa4 | MIT |
| `src/usb_dac.c` | Raspberry Pi (Trading) Ltd.; BambooMaster; xatxa4 | BSD-3-Clause AND MIT |
| `pico_i2s_pio/` (vendored, incl. `i2s.pio`) | BambooMaster | MIT — see `pico_i2s_pio/LICENSE` |
| `pico_usb_device/` (vendored) | Raspberry Pi (Trading) Ltd.; one patch from BambooMaster's fork, one from xatxa4 | BSD-3-Clause — see `pico_usb_device/LICENSE` |
| Raspberry Pi Pico SDK (linked) | Raspberry Pi (Trading) Ltd. | BSD-3-Clause |
| BTstack (linked, via the Pico SDK) | BlueKitchen GmbH | BlueKitchen BTstack licence |

Each source file carries its full licence text in its header, with an SPDX
identifier. Those headers are the authoritative statement; this table is a
summary.

## Credit

- **BlueKitchen GmbH** — BTstack, and the `a2dp_sink_demo` and
  `btstack_audio_pico.c` from which the Bluetooth and sink code derive.
  <https://github.com/bluekitchen/btstack>
- **joba-1** — `PicoW_A2DP`, the working Pico W A2DP sink this project started
  from: the modular split into `a2dp` / `avrcp` / `bt` / `sdp`, the AVRCP volume
  control, the LED and reboot-on-disconnect behaviour.
  <https://github.com/joba-1/PicoW_A2DP>
- **BambooMaster** — `pico-i2s-pio`: the PIO programs (`i2s.pio`), the pin
  mapping and the clock divider scheme that make 32-bit I2S with MCLK work on
  the RP2040; and `usb_sound_card_hires`, which established the working DAC
  configuration. <https://github.com/BambooMaster/pico-i2s-pio>
- **Raspberry Pi (Trading) Ltd.** — the Pico SDK; `pico-extras`' `audio_i2s`,
  whose DMA-and-IRQ structure the output stage follows; `pico-extras`'
  `usb_device`, the USB device stack vendored in `pico_usb_device/`; and the
  `picoboard/button` example from `pico-examples`, which is how `mode_button.c`
  reads BOOTSEL at runtime.

## What this means in practice

The original work in this project (`audio_out.h`, `app_mode.*`, `mode_button.*`,
`usb_dac.h`, `usb_dac_pcm.h`, and this project's own changes throughout) is offered under the **MIT Licence** —
the most permissive of the licences involved. See `LICENSE`. The exception is
`usb_dac.c`, which follows Raspberry Pi's BSD-3-Clause `usb_sound_card` for its
descriptor layout and control requests, so it carries both licences (its header
says which part is which).

That permission applies only to this project's own contributions. It cannot and
does not relicense anyone else's code, and **the terms of the combined work are
set by its most restrictive component, not its most permissive one.**

Concretely, clause 4 of the BlueKitchen BTstack licence reads:

> 4. Any redistribution, use, or modification is done solely for personal
>    benefit and not for any commercial purpose or for monetary gain.

So as it stands, this project as a whole — source or binary — may be used and
redistributed for personal, non-commercial purposes only. BlueKitchen sell
commercial BTstack licences; the header directs enquiries to
<contact@bluekitchen-gmbh.com>.

One loose end worth knowing about: **`joba-1/PicoW_A2DP` ships no LICENCE file.**
Its BTstack-derived parts carry BlueKitchen's terms regardless, but joba-1's own
original contributions are, strictly read, all rights reserved. They have been
credited above and in the file headers; asking joba-1 to state a licence would
settle it properly.
