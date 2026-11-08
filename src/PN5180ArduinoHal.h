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

/// GPIO numbers of the PN5180 connection for ArduinoHal.
struct ArduinoPins {
    int8_t sck;   ///< SPI clock
    int8_t miso;  ///< SPI data PN5180 -> host
    int8_t mosi;  ///< SPI data host -> PN5180
    int8_t nss;   ///< chip select: plain GPIO driven by the HAL, never claimed by SPI.begin()
    int8_t busy;  ///< BUSY input
    int8_t rst;   ///< RESET_N output
    int8_t irq;   ///< IRQ input; -1 when not connected: callers then poll IRQ_STATUS
};

/// pn5180::Hal for Arduino-ESP32: an SPIClass bus plus GPIOs, and a FreeRTOS
/// mutex for lock()/unlock() on ESP_PLATFORM.
class ArduinoHal : public Hal {
public:
    /// Host-side default SPI clock; must not exceed spec::kSpiMaxClockHz
    /// (7 Mbit/s, DS 11.4.1). 1 MHz is a conservative choice for jumper wires.
    static constexpr uint32_t kDefaultSpiClockHz = 1000000u;

    /// @param spi        SPI bus (e.g. `SPI`); begun by begin() with the given
    ///                   SCK/MISO/MOSI pins and without claiming SS
    /// @param pins       pin map
    /// @param spiClockHz SPI clock, at most 7 MHz (DS 11.4.1)
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
