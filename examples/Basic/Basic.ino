// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Basic bring-up sketch for the reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ not connected.
//
// Milestone 1 (in progress): initialises the HAL and reports the BUSY line.
// Reset, version readout from EEPROM and IRQ_STATUS readout will be added once
// the corresponding constants are taken (with citations) from the data sheet.

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
constexpr int8_t PIN_IRQ = -1;  // not connected

// PROVISIONAL: SPI clock / mode not yet verified against the data sheet SPI
// section (spec text unavailable when written; see open questions).
const SPISettings kSpiSettings(1000000, MSBFIRST, SPI_MODE0);

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, kSpiSettings);

void report(const char* what, pn5180::Status s) {
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("PN5180-MIT Basic example");

    const pn5180::Status s = hal.begin();
    report("HAL begin", s);
    if (!pn5180::ok(s)) {
        Serial.println("ERROR: HAL initialisation failed, halting.");
        return;
    }

    Serial.print("BUSY line is ");
    Serial.println(hal.busy() ? "HIGH" : "LOW");
}

void loop() {}
