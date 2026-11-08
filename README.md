# pn5180-library-mit

A driver for the NXP **PN5180** NFC frontend, written from scratch from NXP's
published documentation and released under the **MIT license**.

> **Status: 0.1.0.** Tested on ESP32 (Arduino-ESP32) on the reference test
> bed below; it also builds for ESP32-S3 and ESP32-C3. Raspberry Pi (Linux) support is
> still planned. The API may still change before 1.0.

## Features

| Area | What works | Notes / measured limits |
|---|---|---|
| ISO/IEC 14443A | REQA/WUPA, anticollision over cascade levels 1..3, SELECT, HLTA; `enumerate()` lists every card in the field; `activateUid()` re-selects a known card; bounded retry | read range about 45 mm with the antenna of the generic PN5180 module |
| ISO/IEC 15693 | 1- and 16-slot INVENTORY, GET SYSTEM INFORMATION, READ/WRITE SINGLE BLOCK | |
| Tag memory | NFC Forum Type 2 Tag pages (writes limited to the data area the CC declares), ISO/IEC 15693 blocks; non-destructive `runMemTest()` | |
| NDEF | parser and builder (URI, Text, generic records), NDEF TLV read/write on Type 2 and Type 5 Tags, backup/restore of the touched area | chunked records are not supported; cards are never formatted |
| IRQ pin | optional; `transceive()` waits on the pin instead of polling `IRQ_STATUS`; pin self-test | without a pin everything works by polling |
| LPCD | low-power card detection, EEPROM settings written only where they differ | needs the IRQ pin; detects a *change* of antenna load, not a card that was already there when LPCD was armed |

Every operation returns a `pn5180::Status`; `requiresRecovery(status)` says
when the chip should be hard-reset before continuing.

## Installation

**Arduino IDE / arduino-cli:** download the repository as a ZIP and use
*Sketch > Include Library > Add .ZIP Library...*, or clone it into your
libraries folder:

```sh
cd ~/Arduino/libraries        # Windows: Documents\Arduino\libraries
git clone https://github.com/dunksjunk/pn5180-library-mit.git
```

**PlatformIO:** add the Git URL to `lib_deps` in `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = https://github.com/dunksjunk/pn5180-library-mit.git
```

## Quick start

Prints the UID of an ISO/IEC 14443A card (full version with error recovery:
`examples/ReadUid`).

```cpp
#include <SPI.h>
#include <PN5180.h>
#include <PN5180ArduinoHal.h>

//                               SCK MISO MOSI NSS BUSY RST IRQ(-1 = none)
const pn5180::ArduinoPins pins = {18, 19,  23,  16, 5,   17, -1};
pn5180::ArduinoHal hal(SPI, pins);
pn5180::Pn5180 nfc(hal);
pn5180::Iso14443a reader(nfc);

void setup() {
    Serial.begin(115200);
    pn5180::Status s = nfc.begin();
    if (pn5180::ok(s)) s = nfc.hardReset();
    if (!pn5180::ok(s)) Serial.println(pn5180::statusToString(s));
}

void loop() {
    pn5180::CardA card;
    if (pn5180::ok(reader.fieldOn()) && pn5180::ok(reader.activate(card))) {
        for (uint8_t i = 0; i < card.uidLen; ++i) Serial.printf("%02X ", card.uid[i]);
        Serial.println();
    }
    reader.fieldOff();
    delay(500);
}
```

## Why another PN5180 library?

The PN5180 libraries most people use in the Arduino ecosystem are licensed
under the **LGPL**. Linking LGPL code into firmware is awkward: on a
microcontroller everything is statically linked into one image, so meeting the
LGPL's relinking obligations is impractical for many commercial and
closed-source products.

This project is a **full clean-room rewrite** so that it can be offered under
the permissive MIT license instead:

- **The only technical sources for the chip are NXP's public documents**,
  chiefly the *PN5180A0xx/C3,C4 product data sheet* (Rev. 4.3) and
  application note *AN12650 - Using the PN5180 without library*. The card
  side (ISO/IEC 14443-3, ISO/IEC 15693-3, NFC Forum Type 2 / Type 5 Tag,
  NDEF, RTD) comes from those public standards, cited by clause.
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
| 1 | ESP32 family (ESP32, ESP32-S3, ESP32-C3, ...) via Arduino-ESP32 | SPI + GPIO | 0.1.0: tested on ESP32, builds for S3/C3 |
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
| IRQ | 15 | input; GPIO15 (MTDO) is a strapping pin: low at reset only silences the ROM boot log. `PIN_IRQ = -1` falls back to polling `IRQ_STATUS` |

The examples default to this pin map. Every pin can be changed.

## Layout

| Path | Contents |
|---|---|
| `src/PN5180.h` | umbrella header of the portable core; Arduino sketches also include `PN5180ArduinoHal.h` for `ArduinoHal` / `ArduinoPins` |
| `src/pn5180/` | portable C++17 core: `Hal.h` (platform interface), `Spec.h` (cited constants), `Transport` (BUSY handshake), `Pn5180` (chip commands, IRQ pin), `Status` |
| `src/pn5180/` (protocols) | `Iso14443a`, `Iso15693` (card activation), `Type2Tag`, `Type5Tag` (tag memory), `MemTest` (non-destructive memory test), `Ndef`, `NdefTlv` (NDEF messages and their TLV), `Lpcd` (low-power card detection) |
| `src/PN5180ArduinoHal.*` | Arduino-ESP32 HAL (compiled only when `ARDUINO` is defined) |
| `examples/Basic/` | reset, version readout, IRQ_STATUS and the IRQ pin self-test on the reference test bed |
| `examples/LowPowerDetect/` | low-power card detection (LPCD): sleep until the antenna load changes, then read the cards (needs the IRQ pin) |
| `examples/ReadUid/`, `examples/ReadUid15693/` | UIDs of all ISO/IEC 14443A / ISO/IEC 15693 cards in the field |
| `examples/MemoryTest/` | non-destructive write test of the last user page/block of Type 2 Tags and ISO/IEC 15693 cards |
| `examples/NdefDemo/` | prints the NDEF message of Type 2 / Type 5 Tags, writes and verifies a test message, restores the original |
| `tests/host/` | PC unit test of the core against a fake HAL |

## Host unit tests

The portable core is tested on a PC against a fake HAL (no hardware needed):

```sh
g++ -std=c++17 -Wall -Wextra -Werror -Isrc tests/host/test_core.cpp src/pn5180/*.cpp -o test_core
./test_core
```

CI (`.github/workflows/ci.yml`) runs them with g++ and clang++, and once more
with `-fsanitize=address,undefined`.

## API documentation

```sh
doxygen Doxyfile        # output: build/docs/html/index.html
```

The build must be warning-free (CI fails otherwise and uploads the HTML as
the `api-docs` artifact). `Spec.h` (register map and command codes, each
citing the data sheet) and `Transport.h` (raw host interface) are internal
and not part of the generated docs.

## Several ISO/IEC 14443A cards

`Iso14443a::select()` resolves bit collisions (ISO/IEC 14443-3 6.5.3, with
bit-oriented ANTICOLLISION frames). At each collision it takes the card with
a 1 at the collided bit, so exactly one card ends up selected.
`enumerate(cards, max, count)` finds every card: select one, HLTA it, repeat
with REQA. It returns `BufferTooSmall` if more than `max` cards answer.
HALTed cards stay silent until WUPA or a field reset (`enumerate(..., true)`
resets the field first). `activateUid(uid, len, card)` (WUPA + `selectUid`)
re-selects one known card while others are present.

```cpp
pn5180::CardA cards[8];
size_t count = 0;
pn5180::Status s = reader.enumerate(cards, 8, count);  // after reader.fieldOn()
```

## IRQ pin and low-power card detection

If the HAL has an IRQ line, `hardReset()` reads its polarity from EEPROM
`IRQ_PIN_CONFIG` (0x1A, never written). `transceive()` then waits for RX_IRQ
on the pin instead of polling `IRQ_STATUS` over SPI. The timeouts and returned
Status values are the same as with polling. `Pn5180::checkIrqPin()` raises a
Timer2 IRQ and checks that the pin follows it. It returns `IrqPinError` for a
stuck, miswired or wrong-polarity line.

`Lpcd` puts the chip into low-power card detection with `SWITCH_MODE`
(DS 11.8.10, Table 18). `configure()` writes the LPCD EEPROM bytes only where
they differ and reports how many it wrote, so the count is 0 from the second
boot on. No instruction may be sent while LPCD runs, so the wake can only be
seen on the IRQ pin; without a pin, `start()` returns `Unsupported`. After a
wake the chip is hard-reset. LPCD compares against a reference that is
measured when it is armed, so it detects a change in antenna load (a card
added), not a card that was already lying there. `start()` switches the RF
field off first. The wake-up counter (standby time between two detection
cycles) is accepted in the range 1..1023 ms: DS Table 21 states 1..2690, but
DS Figure 40 shows the firmware replacing values above 0x3FF by 0x3FF, so
larger values are refused with `InvalidArgument`. `readSettings()` returns
`MalformedData` if EEPROM 0x38 holds a reserved calibration mode (10b/11b);
`configure()` overwrites it.

While LPCD is armed, `Pn5180::transport()` must not be used: any NSS activity
ends LPCD (DS 11.8.10). The transport refuses frames with `LowPowerActive`
until the wake has been handled or the chip is reset.

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

## Release checklist

1. Bump `version` in **both** `library.properties` and `library.json` (and
   `PROJECT_NUMBER` in `Doxyfile`); `python tools/check_manifests.py` must
   pass.
2. All local gates green: examples compile warning-free for ESP32, ESP32-S3
   and ESP32-C3 (arduino-cli and `pio ci --lib .`), hardware tests on the
   reference test bed.
3. CI green on the release commit (host tests, sanitizers, manifests, docs).
4. Tag `vX.Y.Z` on that commit and push the tag.
5. Create the GitHub release from the tag, with the changes since the last
   release.

## Development

`auto_dev.py` is the closed-loop development harness used to build this
library. On each iteration it:

1. compiles every example for every target board with all warnings enabled
   and runs the gate commands given with `--gate` (for this project: the
   host unit tests and the hardware-in-the-loop checks on the reference test
   bed);
2. once no gate reports an error, collects review comments from Sourcery
   (remaining warnings are still handed to the author as findings);
3. has Claude Code fix each finding, or dispute it with a reason a human then
   reviews;
4. commits a checkpoint.

The harness gives the coding agent no web access and limits its sources to
the NXP documents (plus the public ISO/IEC and NFC Forum standards for the
card side), so the clean-room rule above is enforced by the tool and not
left to good intentions. Run `python auto_dev.py --help` for usage.

The NXP PDFs are not redistributed in this repository. Download them from
[nxp.com](https://www.nxp.com/) and place them in `.specs/`.

## License

[MIT](https://github.com/dunksjunk/pn5180-library-mit/blob/main/LICENSE) © 2026 Rich Dunkel

## Disclaimer

This software is provided "as is", without warranty of any kind, express or
implied, including but not limited to the warranties of merchantability,
fitness for a particular purpose and non-infringement. In no event shall the
authors or copyright holders be liable for any claim, damages or other
liability, whether in an action of contract, tort or otherwise, arising from,
out of or in connection with the software or the use or other dealings in the
software (see the MIT license in `LICENSE`).

The `MemoryTest` and `NdefDemo` examples write to the tags in the field. They
restore the original content, but use them at your own risk and not on tags
you cannot afford to lose.

This project is independent and is not affiliated with or endorsed by NXP.
PN5180 is a product of NXP Semiconductors.
