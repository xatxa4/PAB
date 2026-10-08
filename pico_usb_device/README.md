# pico_usb_device
The low-level USB device stack from Raspberry Pi's pico-extras, vendored for PAB's USB sound card mode. It is not TinyUSB.

## Origin
- raspberrypi/pico-extras, tag `sdk-2.1.1`, commit `f05d4f7371802440cadd36744789a26944d950ac`
- `src/rp2_common/usb_device` and `src/rp2_common/usb_common`
- The code has not changed since f5c7be9 (2021-01-20) and is identical through `sdk-2.3.1`.
- Only the files PAB needs were copied: `usb_device.c`, `usb_stream_helper.c`, their headers and `usb/usb_common.h`.

## Licence
BSD-3-Clause, Raspberry Pi (Trading) Ltd. See `LICENSE`.

## Changes
Both are marked `// PAB:` in the source.

- `include/pico/usb_device.h`: `struct usb_buffer` `data_len` and `data_max` are `uint16_t` instead of `uint8_t`, so packets over 255 bytes work (24-bit 48 kHz stereo is 294). Same change as commit `993442c` in BambooMaster's pico-extras fork.
- `usb_device.c`: the interrupt handler is a static `usb_device_irq_handler`, installed with `irq_set_exclusive_handler()` in `usb_device_start()`, instead of a global `isr_usbctrl`. A global `isr_usbctrl` overrides the vector table entry at link time even when nothing uses it, which breaks TinyUSB's USB console (`PAB_STDIO=usb`) and hangs the box at boot in Bluetooth mode. This one is PAB's own.

## Build note
Every isochronous endpoint gets the same buffer stride (`PICO_USBDEV_ISOCHRONOUS_BUFFER_STRIDE_TYPE`, 1024 bytes here). The sound card's two double-buffered isochronous endpoints therefore reserve 0x180 + 0x800 + 0x800 = 0x1180 bytes of the 4 KB USB RAM. That works, because the feedback endpoint uses only 3 bytes of its second buffer, but a Debug build would trip `assert(next_buffer_offset <= USB_DPRAM_MAX)` (`usb_device.c`, `_usb_endpoint_hw_init`). PAB builds Release. Giving each endpoint its own stride would fix it; it is not done.
