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

### Reference test bed

A generic ESP32 dev board (ESP32-D0WD-V3, 4 MB flash; Arduino board
"ESP32 Dev Module", `esp32:esp32:esp32`) and a generic PN5180 breakout:

| PN5180 | ESP32 GPIO | Notes |
|---|---|---|
| SCK | 18 | VSPI default |
| MISO | 19 | VSPI default |
| MOSI | 23 | VSPI default |
| NSS | 16 | driven manually by the driver |
| BUSY | 5 | input; GPIO5 is a strapping pin, harmless here |
| RST | 17 | output |
| IRQ | (not connected) | the driver polls `IRQ_STATUS` instead |

The examples default to this pin map. Every pin can be changed.

## Layout

| Path | Contents |
|---|---|
| `src/pn5180/` | portable C++17 core: `Spec.h` (cited constants), `Transport` (BUSY handshake), `Pn5180` (commands), `Hal.h` (platform interface) |
| `src/PN5180ArduinoHal.*` | Arduino-ESP32 HAL (compiled only when `ARDUINO` is defined) |
| `examples/Basic/` | reset, version readout, IRQ_STATUS on the reference test bed |
| `examples/ReadUid/`, `examples/ReadUid15693/` | UIDs of ISO/IEC 14443A / ISO/IEC 15693 cards |
| `examples/MemoryTest/` | non-destructive write test of the last user page/block of Type 2 Tags and ISO/IEC 15693 cards |
| `examples/NdefDemo/` | prints the NDEF message of Type 2 / Type 5 Tags, writes and verifies a test message, restores the original |
| `tests/host/` | PC unit test of the core against a fake HAL |

Host test: `g++ -std=c++17 -Wall -Wextra -Werror -Isrc tests/host/test_core.cpp src/pn5180/*.cpp && ./a.out`

## NDEF

Written from the public NFC Forum specifications (NDEF 1.0, RTD URI, RTD
Text, Type 2 Tag and Type 5 Tag technical specifications); no vendor tag
data sheets.

- `Ndef.h`: `NdefParser` walks a raw message record by record without
  reading outside the buffer (malformed data -> `MalformedData`, chunked
  records -> `Unsupported`); `NdefBuilder` appends records into a caller
  buffer (MB/ME, short/long records, `addUri()` with the longest URI
  prefix code, `addText()`); `decodeNdefUri()` / `decodeNdefText()`.
- `NdefTlv.h`: finds the NDEF Message TLV (skips NULL, Lock/Memory Control
  and proprietary TLVs, 1- and 3-byte lengths) in any `NdefMemory` and
  writes a message as NDEF TLV + Terminator: length 0 first, the real
  length last, so an interrupted write leaves an empty but valid TLV.
  `backupNdefArea()` / `restoreNdefArea()` save and restore the touched units.
- `Type2Tag::readNdef()/writeNdef()` (CC in page 3; pages 0-3 are never
  written; a read-only CC refuses writes) and `Type5Tag` for ISO/IEC 15693
  (CC in block 0, 4- or 8-byte CC, block geometry from GET SYSTEM
  INFORMATION). A card without a valid CC is reported as `NotFormatted`;
  the library never formats a card on its own.

```cpp
uint8_t msg[128];
pn5180::NdefBuilder b(msg, sizeof(msg));
b.addUri("https://example.com/");
b.addText("hello", "en");
pn5180::Type2Tag tag(nfc);             // after Iso14443a activation
pn5180::Status s = tag.writeNdef(b.data(), b.size());
```

## Thread safety

The driver is not thread-safe. A `Pn5180` object and the protocol objects
built on it (`Iso14443a`, `Iso15693`, `Type2Tag`, `Type5Tag`, the `MemTest`
helpers) belong to one owner; the NDEF parser/builder only touch caller
buffers. A protocol operation
issues several chip commands and temporarily changes chip registers, so two
tasks must never drive the same chip at the same time. If several tasks need
the reader, hold your own mutex around each complete operation (field on ...
field off).

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
