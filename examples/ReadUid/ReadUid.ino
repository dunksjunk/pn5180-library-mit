// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Reads the UIDs of all ISO/IEC 14443 Type A cards in the field (4, 7 or
// 10 bytes each), resolving collisions (Iso14443a::enumerate).
// Reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ 15 (strapping pin
//   MTDO: low at reset only silences the ROM boot log; the PN5180 IRQ idles
//   inactive, see the polarity printed by the Basic example, EEPROM 0x1A, DS Table 51).
//
// Every ~500 ms: RF field on, then per card REQA + anticollision/SELECT +
// HLTA until no card answers (a garbled or missing answer is retried up to
// twice per poll), one line per card:
//   UID: 04 A1 B2 C3 D4 E5 F6  ATQA: 0044  SAK: 00
// (with several cards the ATQA can be the superposition of their ATQAs),
// then RF field off, which wakes the HALTed cards for the next poll.
// No card: nothing is printed. An error that
// persists over all attempts is printed by Status name. Device/transport
// errors (and a failed RF off) trigger a hardware reset; a failed start-up
// is retried every 5 s.

#include <SPI.h>

#include <PN5180.h>
#include <PN5180ArduinoHal.h>

// Declared at global scope on purpose: PlatformIO's .ino converter would
// otherwise generate these prototypes inside the anonymous namespace below.
void setup();
void loop();

namespace {

constexpr int8_t PIN_SCK = 18;
constexpr int8_t PIN_MISO = 19;
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NSS = 16;
constexpr int8_t PIN_BUSY = 5;
constexpr int8_t PIN_RST = 17;
constexpr int8_t PIN_IRQ = 15;  // -1 = not connected: IRQ_STATUS is polled instead

// 1 MHz: well below the 7 Mbit/s maximum (DS 11.4.1).
constexpr uint32_t SPI_CLOCK_HZ = 1000000u;
constexpr uint32_t POLL_INTERVAL_MS = 500;
constexpr uint32_t INIT_RETRY_MS = 5000;  // retry after a failed start-up / reset
constexpr size_t MAX_CARDS = 8;           // cards listed per poll

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, SPI_CLOCK_HZ);
pn5180::Pn5180 nfc(hal);
pn5180::Iso14443a reader(nfc);
bool ready = false;

void printHex2(uint8_t b) {
    if (b < 0x10) {
        Serial.print('0');
    }
    Serial.print(b, HEX);
}

void printError(const char* what, pn5180::Status s) {
    Serial.print("ERROR ");
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
}

void printCard(const pn5180::CardA& card) {
    Serial.print("UID:");
    for (uint8_t i = 0; i < card.uidLen; ++i) {
        Serial.print(' ');
        printHex2(card.uid[i]);
    }
    Serial.print("  ATQA: ");
    printHex2(static_cast<uint8_t>(card.atqa >> 8));
    printHex2(static_cast<uint8_t>(card.atqa));
    Serial.print("  SAK: ");
    printHex2(card.sak);
    Serial.println();
}

// The only place the chip is (re)initialised: HAL setup until it succeeds
// once, then a hardware reset. Used at start-up and after device errors.
bool initReader() {
    static bool begun = false;
    static bool announced = false;
    if (!begun) {
        const pn5180::Status s = nfc.begin();
        if (!pn5180::ok(s)) {
            printError("begin", s);
            return false;
        }
        begun = true;
    }
    const pn5180::Status s = nfc.hardReset();
    if (!pn5180::ok(s)) {
        printError("hard reset", s);
        return false;
    }
    ready = true;
    if (!announced) {  // start-up message only, not after a recovery reset
        announced = true;
        Serial.println("Waiting for an ISO/IEC 14443 Type A card...");
    }
    return true;
}

// Reports `s` (NoResponse is silent). If it needs a chip reset
// (pn5180::requiresRecovery), switches the field off (best effort; the reset
// also turns it off), marks the reader not ready and returns false so the
// current poll stops; loop() then performs the reset.
bool check(const char* what, pn5180::Status s) {
    if (pn5180::ok(s) || s == pn5180::Status::NoResponse) {
        return true;
    }
    printError(what, s);
    if (pn5180::requiresRecovery(s)) {
        (void)reader.fieldOff();
        ready = false;
        return false;
    }
    return true;  // card-level error: keep polling
}

void pollOnce() {
    pn5180::Status s = reader.fieldOn();
    if (!pn5180::ok(s)) {
        if (check("RF on", s)) {
            (void)reader.fieldOff();  // best effort; the RF on error was reported
        }
        return;
    }
    // Every card in the field: anticollision resolves the collisions, each
    // card is SELECTed, then HALTed so the next REQA finds the others. A
    // garbled or missing answer is retried (bounded, see ActivateRetry).
    pn5180::CardA cards[MAX_CARDS];
    size_t count = 0;
    s = reader.enumerate(cards, MAX_CARDS, count);
    for (size_t i = 0; i < count; ++i) {
        printCard(cards[i]);
    }
    if (s == pn5180::Status::BufferTooSmall) {
        Serial.println("(more cards in the field than MAX_CARDS)");
    } else if (!check("enumerate", s)) {
        return;
    }
    s = reader.fieldOff();  // RF field off between polls
    if (!pn5180::ok(s)) {
        // Field state unknown: always reset, whatever the status.
        printError("RF off", s);
        ready = false;
    }
}

// millis() time of the next action (initialisation or poll).
uint32_t nextActionMs = 0;

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("PN5180-MIT ReadUid example");
    nextActionMs = millis();  // initialise and poll right away in loop()
}

void loop() {
    const uint32_t now = millis();
    // Signed difference: correct across millis() wrap-around.
    if (static_cast<int32_t>(now - nextActionMs) < 0) {
        delay(1);
        return;
    }
    if (!ready) {
        // Start-up or recovery reset; on failure retry after INIT_RETRY_MS,
        // on success poll immediately.
        nextActionMs = initReader() ? now : now + INIT_RETRY_MS;
        return;
    }
    pollOnce();
    // After a device error the reset runs on the next pass.
    nextActionMs = ready ? now + POLL_INTERVAL_MS : now;
}
