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

/// Platform interface of the portable core: SPI transfer, the NSS, BUSY and
/// RST lines, an optional IRQ line and a millisecond clock.
///
/// Implemented by ArduinoHal (Arduino-ESP32), later a Linux HAL (spidev +
/// GPIO character device), and a fake HAL in the PC unit tests.
class Hal {
public:
    virtual ~Hal() = default;

    /// Prepare SPI and GPIO. Called once by the driver's begin().
    virtual Status begin() = 0;

    /// Start one SPI frame: acquire the SPI bus (e.g. begin an SPI
    /// transaction so no other device on a shared bus can clock data), then
    /// assert NSS (low). Every successful beginFrame() is paired with exactly
    /// one endFrame(), which the core guarantees even on error paths.
    virtual Status beginFrame() = 0;

    /// Full-duplex SPI transfer of @p len bytes inside a frame.
    /// @param tx     bytes to send; may be nullptr: the HAL then sends @p filler
    /// @param rx     received bytes; may be nullptr: received bytes are discarded
    /// @param len    number of bytes
    /// @param filler byte sent when @p tx is nullptr
    virtual Status transfer(const uint8_t* tx, uint8_t* rx, size_t len, uint8_t filler) = 0;

    /// End the SPI frame: release NSS (high), then release the SPI bus.
    virtual void endFrame() = 0;

    /// Drive the RST (RESET_N) line. @p asserted == true holds the chip in reset (low).
    virtual void setReset(bool asserted) = 0;

    /// Current level of the BUSY line (true = high).
    virtual bool busy() = 0;

    /// Whether an IRQ line is wired. If false, callers poll IRQ_STATUS.
    virtual bool hasIrq() const { return false; }

    /// Current level of the IRQ line (true = high). Only meaningful when
    /// hasIrq() returns true.
    virtual bool irq() { return false; }

    /// Monotonic millisecond clock (wrap-around safe with unsigned arithmetic).
    virtual uint32_t millis() = 0;

    /// Block for at least @p ms milliseconds (should let other tasks run).
    virtual void delayMs(uint32_t ms) = 0;

    /// Delay of at least @p us microseconds (busy-wait acceptable).
    virtual void delayUs(uint32_t us) = 0;

    /// Give other tasks a chance to run during short polling loops.
    virtual void yield() {}

    /// Serialise complete host interface commands (both SPI frames plus their
    /// BUSY handshakes) between tasks/threads sharing one driver. The default
    /// is a no-op for single-threaded use; RTOS/OS HALs should use a mutex.
    virtual void lock() {}
    /// Counterpart of lock().
    virtual void unlock() {}
};

}  // namespace pn5180
