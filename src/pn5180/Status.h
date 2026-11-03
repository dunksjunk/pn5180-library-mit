// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Typed status codes returned by every public PN5180-MIT function.
// Portable core header: must not include Arduino.h or any OS header.
#pragma once

#include <cstdint>

namespace pn5180 {

enum class Status : uint8_t {
    Ok = 0,
    InvalidArgument,   // caller passed a null buffer, bad length, etc.
    NotInitialized,    // begin() was not called or failed
    BusyTimeout,       // BUSY did not reach the expected level in time
    IrqTimeout,        // the expected IRQ / IRQ_STATUS bit did not appear in time
    HalError,          // the platform layer reported an SPI/GPIO failure
    BufferTooSmall,    // a fixed-size internal or caller buffer is too small
    WriteProtected,    // guarded operation (e.g. EEPROM write) not unlocked
    ChipError,         // the PN5180 reported an error condition
};

// Human-readable name for logging. Never returns nullptr.
const char* statusToString(Status s);

inline bool ok(Status s) { return s == Status::Ok; }

}  // namespace pn5180
