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
    if (clockHz_ == 0 || clockHz_ > spec::kSpiMaxClockHz) {  // DS 11.4.1
        return Status::InvalidArgument;
    }

#if defined(ESP_PLATFORM)
    if (mutex_ == nullptr) {
        mutex_ = xSemaphoreCreateMutexStatic(&mutexBuf_);  // static storage, no heap
        if (mutex_ == nullptr) {
            return Status::HalError;
        }
    }
#endif

    // NSS is a plain GPIO owned by this HAL. Release it (high) before the bus
    // comes up so the PN5180 never sees a spurious frame.
    pinMode(pins_.nss, OUTPUT);
    digitalWrite(pins_.nss, HIGH);

    pinMode(pins_.busy, INPUT);
    pinMode(pins_.rst, OUTPUT);
    digitalWrite(pins_.rst, HIGH);  // RESET_N high = not in reset (DS 11.2.1.2)
    if (pins_.irq >= 0) {
        pinMode(pins_.irq, INPUT);
    }

    // ss = -1: never let the SPI driver claim a chip-select pin. On the
    // reference board the Arduino default SS (GPIO5) is wired to BUSY.
    spi_.begin(pins_.sck, pins_.miso, pins_.mosi, -1);
    begun_ = true;
    return Status::Ok;
}

Status ArduinoHal::beginFrame() {
    if (!begun_) {
        return Status::NotInitialized;
    }
    // MSB first, CPOL = 0 / CPHA = 0 (SPI mode 0).  DS 11.4.1
    // The transaction is opened before NSS goes low and closed after NSS goes
    // high, so no other device on a shared bus can clock while selected.
    spi_.beginTransaction(SPISettings(clockHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(pins_.nss, LOW);
    return Status::Ok;
}

Status ArduinoHal::transfer(const uint8_t* tx, uint8_t* rx, size_t len, uint8_t filler) {
    if (!begun_) {
        return Status::NotInitialized;
    }
    for (size_t i = 0; i < len; ++i) {
        const uint8_t in = spi_.transfer(tx != nullptr ? tx[i] : filler);
        if (rx != nullptr) {
            rx[i] = in;
        }
    }
    return Status::Ok;
}

void ArduinoHal::endFrame() {
    digitalWrite(pins_.nss, HIGH);
    spi_.endTransaction();
}

void ArduinoHal::setReset(bool asserted) { digitalWrite(pins_.rst, asserted ? LOW : HIGH); }

bool ArduinoHal::busy() { return digitalRead(pins_.busy) == HIGH; }

bool ArduinoHal::irq() { return pins_.irq >= 0 && digitalRead(pins_.irq) == HIGH; }

uint32_t ArduinoHal::millis() { return static_cast<uint32_t>(::millis()); }

void ArduinoHal::delayMs(uint32_t ms) { ::delay(ms); }

void ArduinoHal::delayUs(uint32_t us) { ::delayMicroseconds(us); }

void ArduinoHal::yield() { ::yield(); }

void ArduinoHal::lock() {
#if defined(ESP_PLATFORM)
    if (mutex_ != nullptr) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
    }
#endif
}

void ArduinoHal::unlock() {
#if defined(ESP_PLATFORM)
    if (mutex_ != nullptr) {
        xSemaphoreGive(mutex_);
    }
#endif
}

}  // namespace pn5180

#endif  // ARDUINO
