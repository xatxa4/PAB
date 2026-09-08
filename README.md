# PAB — Pico Audio Bridge

A Bluetooth A2DP audio sink for the Raspberry Pi Pico W, feeding an external I2S
DAC with 32-bit frames at BCLK = 64fs. Built for an ES9038Q2M, which will not
accept the 16-bit / 32fs stream the usual Pico I2S drivers produce.

A mode framework is in place for further personalities (USB sound card, internet
radio, S/PDIF receiver); only the Bluetooth sink is implemented today.

## Wiring

| Signal | Pin | Note |
|--------|-----|------|
| DATA / DIN | GP18 | |
| LRCLK / WS | GP20 | `PICO_AUDIO_I2S_CLOCK_PIN_BASE` |
| BCK | GP21 | clock pin base + 1 |
| MCLK | — | off by default, see `CMakeLists.txt` |
| Connection indicator | GP26 | high while a stream is established |

## Build

```sh
cmake -B build -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Options: `-DPAB_STDIO=usb|uart|none` (default `usb`). Pin assignments, I2S
buffer depth and the Bluetooth name are compile definitions in `CMakeLists.txt`.

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
| `src/audio_out.h`, `app_mode.*`, `usb_dac.*` | xatxa4 | MIT |
| `pico_i2s_pio/` (vendored, incl. `i2s.pio`) | BambooMaster | MIT — see `pico_i2s_pio/LICENSE` |
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
- **Raspberry Pi (Trading) Ltd.** — the Pico SDK, and `pico-extras`'
  `audio_i2s`, whose DMA-and-IRQ structure the output stage follows.

## What this means in practice

The original work in this project (`audio_out.h`, `app_mode.*`, `usb_dac.*`, and
this project's own changes throughout) is offered under the **MIT Licence** —
the most permissive of the licences involved. See `LICENSE`.

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
