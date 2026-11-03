// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Arduino (ESP32) implementation of pn5180::Hal. Platform code: this is the
// only place that includes Arduino.h / SPI.h.
#pragma once

#if defined(ARDUINO)

#include <Arduino.h>
#include <SPI.h>

#include "pn5180/Hal.h"

namespace pn5180 {

struct ArduinoPins {
    int8_t sck;
    int8_t miso;
    int8_t mosi;
    int8_t nss;
    int8_t busy;
    int8_t rst;
    int8_t irq;  // -1 when not connected; the core then polls IRQ_STATUS
};

class ArduinoHal : public Hal {
public:
    // `settings` (clock, bit order, SPI mode) is supplied by the caller so no
    // SPI timing is hard-coded in the library.
    ArduinoHal(SPIClass& spi, const ArduinoPins& pins, const SPISettings& settings)
        : spi_(spi), pins_(pins), settings_(settings) {}

    Status begin() override;
    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
    void setNss(bool active) override;
    void setReset(bool asserted) override;
    bool busy() override;
    bool hasIrq() const override { return pins_.irq >= 0; }
    bool irq() override;
    uint32_t millis() override;
    void delayMs(uint32_t ms) override;

private:
    SPIClass& spi_;
    ArduinoPins pins_;
    SPISettings settings_;
    bool begun_ = false;
};

}  // namespace pn5180

#endif  // ARDUINO
