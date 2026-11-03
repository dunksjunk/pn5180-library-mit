// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Arduino (ESP32) implementation of pn5180::Hal.
#if defined(ARDUINO)

#include "PN5180ArduinoHal.h"

namespace pn5180 {

Status ArduinoHal::begin() {
    if (pins_.nss < 0 || pins_.busy < 0 || pins_.rst < 0 || pins_.sck < 0 || pins_.miso < 0 ||
        pins_.mosi < 0) {
        return Status::InvalidArgument;
    }

    // NSS is a plain GPIO owned by this HAL. Release it (high) before the bus
    // comes up so the PN5180 never sees a spurious frame.
    pinMode(pins_.nss, OUTPUT);
    digitalWrite(pins_.nss, HIGH);

    pinMode(pins_.busy, INPUT);
    pinMode(pins_.rst, OUTPUT);
    digitalWrite(pins_.rst, HIGH);  // not in reset
    if (pins_.irq >= 0) {
        pinMode(pins_.irq, INPUT);
    }

    // ss = -1: never let the SPI driver claim a chip-select pin. On the
    // reference board the Arduino default SS (GPIO5) is wired to BUSY.
    spi_.begin(pins_.sck, pins_.miso, pins_.mosi, -1);
    begun_ = true;
    return Status::Ok;
}

Status ArduinoHal::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    if (!begun_) {
        return Status::NotInitialized;
    }
    spi_.beginTransaction(settings_);
    for (size_t i = 0; i < len; ++i) {
        // 0xFF filler when no tx data: host-side choice, see open questions.
        const uint8_t out = (tx != nullptr) ? tx[i] : 0xFF;
        const uint8_t in = spi_.transfer(out);
        if (rx != nullptr) {
            rx[i] = in;
        }
    }
    spi_.endTransaction();
    return Status::Ok;
}

void ArduinoHal::setNss(bool active) { digitalWrite(pins_.nss, active ? LOW : HIGH); }

void ArduinoHal::setReset(bool asserted) { digitalWrite(pins_.rst, asserted ? LOW : HIGH); }

bool ArduinoHal::busy() { return digitalRead(pins_.busy) == HIGH; }

bool ArduinoHal::irq() { return pins_.irq >= 0 && digitalRead(pins_.irq) == HIGH; }

uint32_t ArduinoHal::millis() { return static_cast<uint32_t>(::millis()); }

void ArduinoHal::delayMs(uint32_t ms) { ::delay(ms); }

}  // namespace pn5180

#endif  // ARDUINO
