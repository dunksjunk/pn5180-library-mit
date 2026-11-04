// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Basic bring-up sketch for the reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ not connected.
//
// Resets the PN5180, prints the product / firmware / EEPROM versions stored
// in EEPROM and the IRQ_STATUS register, reporting every error.

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

// 1 MHz: well below the 7 Mbit/s maximum (DS 11.4.1); SPI mode 0 is fixed
// inside the HAL (DS 11.4.1).
constexpr uint32_t SPI_CLOCK_HZ = 1000000u;

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, SPI_CLOCK_HZ);
pn5180::Pn5180 nfc(hal);

bool check(const char* what, pn5180::Status s) {
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
    return pn5180::ok(s);
}

void printVersion(const char* label, const pn5180::Version& v) {
    Serial.print(label);
    Serial.print(v.major);
    Serial.print('.');
    // Hex minor matches NXP's notation, e.g. FW 3.A / 4.1 (DS Table 51).
    Serial.println(v.minor, HEX);
}

void printHex2(uint8_t b) {
    if (b < 0x10) {
        Serial.print('0');
    }
    Serial.print(b, HEX);
}

// The data sheet gives no major.minor meaning for the EEPROM version
// (DS Table 51), so print the raw 16-bit little-endian value and its bytes
// in address order (0x14, 0x15).
void printRawVersion(const char* label, const pn5180::Version& v) {
    Serial.print(label);
    Serial.print("0x");
    printHex2(v.major);  // byte at the higher address  DS Table 51
    printHex2(v.minor);  // byte at the lower address   DS Table 51
    Serial.print(" (bytes ");
    printHex2(v.minor);
    Serial.print(' ');
    printHex2(v.major);
    Serial.println(')');
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("PN5180-MIT Basic example");

    if (!check("begin", nfc.begin())) {
        Serial.println("ERROR: HAL initialisation failed (check pin map / SPI clock).");
        return;
    }
    if (!check("hard reset", nfc.hardReset())) {
        Serial.println("ERROR: PN5180 did not finish start-up. Check wiring, power and BUSY/RST pins.");
        return;
    }

    pn5180::Versions v;
    if (check("read versions", nfc.readVersions(v))) {
        printVersion("  product version:  ", v.product);
        printVersion("  firmware version: ", v.firmware);
        printRawVersion("  EEPROM version:   ", v.eeprom);
    }

    uint32_t irq = 0;
    if (check("read IRQ_STATUS", nfc.readIrqStatus(irq))) {
        Serial.print("  IRQ_STATUS = 0x");
        Serial.println(irq, HEX);
        if (irq & pn5180::spec::irq::GENERAL_ERROR) {
            Serial.println("  GENERAL_ERROR_IRQ is set");
        }
    }
}

void loop() {}
