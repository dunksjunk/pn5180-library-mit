// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Arduino (ESP32) implementation of pn5180::Hal. Platform code: this is the
// only place that includes Arduino.h / SPI.h.
#pragma once

#if defined(ARDUINO)

#include <Arduino.h>
#include <SPI.h>

#if defined(ESP_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#include "pn5180/Hal.h"
#include "pn5180/Spec.h"

namespace pn5180 {

struct ArduinoPins {
    int8_t sck;
    int8_t miso;
    int8_t mosi;
    int8_t nss;   // plain GPIO driven by the HAL, never claimed by SPI.begin()
    int8_t busy;
    int8_t rst;
    int8_t irq;   // -1 when not connected; callers then poll IRQ_STATUS
};

class ArduinoHal : public Hal {
public:
    // Host-side default SPI clock; must not exceed spec::kSpiMaxClockHz
    // (7 Mbit/s, DS 11.4.1). 1 MHz is a conservative choice for jumper wires.
    static constexpr uint32_t kDefaultSpiClockHz = 1000000u;

    ArduinoHal(SPIClass& spi, const ArduinoPins& pins, uint32_t spiClockHz = kDefaultSpiClockHz)
        : spi_(spi), pins_(pins), clockHz_(spiClockHz) {}

    Status begin() override;
    Status beginFrame() override;
    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len, uint8_t filler) override;
    void endFrame() override;
    void setReset(bool asserted) override;
    bool busy() override;
    bool hasIrq() const override { return pins_.irq >= 0; }
    bool irq() override;
    uint32_t millis() override;
    void delayMs(uint32_t ms) override;
    void delayUs(uint32_t us) override;
    void yield() override;
    void lock() override;
    void unlock() override;

private:
    SPIClass& spi_;
    ArduinoPins pins_;
    uint32_t clockHz_;
    bool begun_ = false;
#if defined(ESP_PLATFORM)
    StaticSemaphore_t mutexBuf_{};
    SemaphoreHandle_t mutex_ = nullptr;
#endif
};

}  // namespace pn5180

#endif  // ARDUINO
