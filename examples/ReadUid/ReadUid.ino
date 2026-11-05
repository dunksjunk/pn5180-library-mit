// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Reads the UID of an ISO/IEC 14443 Type A card (4, 7 or 10 bytes).
// Reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ not connected.
//
// Every ~500 ms: RF field on, REQA + anticollision/SELECT (retried up to
// three times after a garbled or missing answer, Iso14443a::activateWithRetry),
// print
//   UID: 04 A1 B2 C3 D4 E5 F6  ATQA: 0044  SAK: 00
// then HLTA and RF field off. No card: nothing is printed. An error that
// persists over all attempts is printed by Status name. Device/transport
// errors (and a failed RF off) trigger a hardware reset; a failed start-up
// is retried every 5 s.

#include <SPI.h>

#include <PN5180.h>
#include <PN5180ArduinoHal.h>

namespace {

constexpr int8_t PIN_SCK = 18;
constexpr int8_t PIN_MISO = 19;
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NSS = 16;
constexpr int8_t PIN_BUSY = 5;
constexpr int8_t PIN_RST = 17;
constexpr int8_t PIN_IRQ = -1;  // not connected: IRQ_STATUS is polled

// 1 MHz: well below the 7 Mbit/s maximum (DS 11.4.1).
constexpr uint32_t SPI_CLOCK_HZ = 1000000u;
constexpr uint32_t POLL_INTERVAL_MS = 500;
constexpr uint32_t INIT_RETRY_MS = 5000;  // retry after a failed start-up / reset

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
    pn5180::CardA card;
    // A single garbled or missing answer is retried (bounded, see ActivateRetry).
    s = reader.activateWithRetry(card);
    if (pn5180::ok(s)) {
        printCard(card);
        if (!check("HLTA", reader.halt())) {
            return;
        }
    } else if (!check("activate", s)) {
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
