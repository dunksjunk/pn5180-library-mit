# pn5180-library-mit

A driver for the NXP **PN5180** NFC frontend, written from scratch from NXP's
published documentation and released under the **MIT license**.

> **Status: pre-alpha.** Nothing here is usable yet. The API will change without
> notice until the first tagged release.

## Why another PN5180 library?

The PN5180 libraries most people use in the Arduino ecosystem are licensed
under the **LGPL**. Linking LGPL code into firmware is awkward: on a
microcontroller everything is statically linked into one image, so meeting the
LGPL's relinking obligations is impractical for many commercial and
closed-source products.

This project is a **full clean-room rewrite** so that it can be offered under
the permissive MIT license instead:

- **The only technical sources are NXP's public documents**, chiefly the
  *PN5180A0xx/C3,C4 product data sheet* (Rev. 4.3) and application note
  *AN12650 - Using the PN5180 without library*.
- **No code, structure or naming has been copied, ported or adapted from any
  existing PN5180 library.** Contributors must not consult those
  implementations while working on this codebase.
- **Every command code, register address, bit field and timing value in the
  source cites the data sheet or application note section it came from**,
  so where each value came from can be checked.

If you contribute, please follow the same rules. A pull request that is
derived from LGPL- or GPL-licensed PN5180 code cannot be accepted.

## Target platforms

| Priority | Platform | Interface | Status |
|---|---|---|---|
| 1 | ESP32 family (ESP32, ESP32-S3, ESP32-C3, ...) via Arduino-ESP32 | SPI + GPIO | in development |
| 2 | Raspberry Pi (Linux), starting with the Pi Zero 2 W | `spidev` + GPIO character device | planned |

The protocol logic is plain, portable C++ with no dependency on `Arduino.h`.
Each platform supplies a small hardware layer (SPI transfer, the NSS, BUSY and
RST lines, an optional IRQ line, and a millisecond clock). That keeps the core
testable on a PC and makes new targets cheap to add.

## Hardware

The PN5180 talks to the host over SPI and also needs these control lines:

| PN5180 signal | Purpose | Host connection |
|---|---|---|
| NSS | SPI chip select (active low) | any GPIO |
| MOSI / MISO / SCK | SPI data and clock | the host's SPI bus |
| BUSY | high while the PN5180 is processing; the host must wait for it | GPIO input |
| RST | hardware reset (active low) | GPIO output |
| IRQ | interrupt request (optional but recommended) | GPIO input |
| GND, 3.3 V, 5 V | power, as labelled on your module | |

The PN5180 uses **3.3 V logic**. Do not connect it to 5 V GPIO without level
shifting. Many generic breakout modules have separate 3.3 V and 5 V supply pins,
so check your module's markings.

The first test bed is a generic ESP32 dev board with a generic PN5180 breakout
wired over SPI. Its exact pin map will be documented with the first example
sketch.

## Installation

Not yet published. Once the library is usable it will be installable from the
Arduino Library Manager and PlatformIO, and as a CMake project for Linux.

## Development

`auto_dev.py` is the closed-loop development harness used to build this
library. On each iteration it:

1. compiles every example for every target board with all warnings enabled;
2. once that build is clean, collects review comments from Sourcery;
3. has Claude Code fix each finding, or dispute it with a reason a human then
   reviews;
4. commits a checkpoint.

The harness gives the coding agent no web access and limits its sources to
the NXP documents, so the clean-room rule above is enforced by the tool and
not left to good intentions. Run `python auto_dev.py --help` for usage.

The NXP PDFs are not redistributed in this repository. Download them from
[nxp.com](https://www.nxp.com/) and place them in `.specs/`.

## License

[MIT](LICENSE) © 2026 Rich Dunkel

PN5180 is a product of NXP Semiconductors. This project is independent and is
not affiliated with or endorsed by NXP.
