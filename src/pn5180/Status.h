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
    ChipError,         // the PN5180 raised GENERAL_ERROR_IRQ for the command
    ReadOnly,          // write to a read-only register, or to a card the card marks read-only
    Unsupported,       // not available on the detected firmware version
    NoResponse,        // no RF response within the receive timeout (no card)
    Collision,         // RX_STATUS reported a bit collision (several cards)
    IntegrityError,    // RX_STATUS: wrong parity or CRC in the received frame
    ProtocolError,     // RX_STATUS: framing error (stop bit, SOF/EOF, length)
    CardError,         // card answered with an invalid / unexpected frame
    RfActiveError,     // RF_ON refused: external field detected (RF_ACTIVE_ERROR_IRQ)
    RfFieldTimeout,    // RF field did not come on (RF_STATUS.TX_RF_STATUS) in time
    NotFormatted,      // card has no valid NFC Forum Capability Container (not NDEF formatted)
    MalformedData,     // NDEF message / TLV structure invalid, or an EEPROM setting holds a reserved encoding
    IrqPinError,       // the IRQ line does not follow IRQ_STATUS (miswired, stuck, wrong polarity)
    LowPowerActive,    // chip is in LPCD/standby: no instruction may be sent (DS Table 18)
};

// Human-readable name for logging. Never returns nullptr.
const char* statusToString(Status s);

inline bool ok(Status s) { return s == Status::Ok; }

// Recovery policy: true when the status means the PN5180 or the link to it
// is in an unknown state (transport/BUSY/IRQ timeout, HAL failure, chip
// exception, RF field not coming on, driver not initialised), so the
// application should hard-reset the chip before continuing. False for
// success, caller errors (bad arguments, protection) and card-level results
// (no card, collision, CRC/parity/framing errors, external field), after
// which polling can simply continue. Every Status value is classified.
bool requiresRecovery(Status s);

}  // namespace pn5180
