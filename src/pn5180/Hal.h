// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Hardware abstraction interface between the portable PN5180 core and a
// platform (Arduino-ESP32 today, Linux spidev + libgpiod later, a fake HAL in
// PC unit tests). Portable core header: no Arduino.h, no OS headers.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Status.h"

namespace pn5180 {

class Hal {
public:
    virtual ~Hal() = default;

    // Prepare SPI and GPIO. Called once by the driver's begin().
    virtual Status begin() = 0;

    // Full-duplex SPI transfer of `len` bytes. `tx` may be nullptr (send
    // filler bytes); `rx` may be nullptr (discard received bytes). NSS is not
    // touched here; the core drives it explicitly with setNss().
    virtual Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;

    // Drive the NSS (chip select) line. `active == true` means asserted (low).
    virtual void setNss(bool active) = 0;

    // Drive the RST line. `asserted == true` means reset is held (line low).
    virtual void setReset(bool asserted) = 0;

    // Current level of the BUSY line (true = high).
    virtual bool busy() = 0;

    // Whether an IRQ line is wired. If false, the core polls IRQ_STATUS.
    virtual bool hasIrq() const { return false; }

    // Current level of the IRQ line (true = high). Only meaningful when
    // hasIrq() returns true.
    virtual bool irq() { return false; }

    // Monotonic millisecond clock (wrap-around safe with unsigned arithmetic).
    virtual uint32_t millis() = 0;

    // Block for at least `ms` milliseconds.
    virtual void delayMs(uint32_t ms) = 0;
};

}  // namespace pn5180
