// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
#include "Status.h"

namespace pn5180 {

const char* statusToString(Status s) {
    switch (s) {
        case Status::Ok:              return "Ok";
        case Status::InvalidArgument: return "InvalidArgument";
        case Status::NotInitialized:  return "NotInitialized";
        case Status::BusyTimeout:     return "BusyTimeout";
        case Status::IrqTimeout:      return "IrqTimeout";
        case Status::HalError:        return "HalError";
        case Status::BufferTooSmall:  return "BufferTooSmall";
        case Status::WriteProtected:  return "WriteProtected";
        case Status::ChipError:       return "ChipError";
        case Status::ReadOnly:        return "ReadOnly";
        case Status::Unsupported:     return "Unsupported";
        case Status::NoResponse:      return "NoResponse";
        case Status::Collision:       return "Collision";
        case Status::IntegrityError:  return "IntegrityError";
        case Status::ProtocolError:   return "ProtocolError";
        case Status::CardError:       return "CardError";
        case Status::RfActiveError:   return "RfActiveError";
        case Status::RfFieldTimeout:  return "RfFieldTimeout";
        case Status::NotFormatted:    return "NotFormatted";
        case Status::MalformedData:   return "MalformedData";
    }
    return "Unknown";
}

bool requiresRecovery(Status s) {
    // No default: -Wswitch flags any Status added without a classification.
    switch (s) {
        // Device / transport state unknown: reset the chip.
        case Status::NotInitialized:
        case Status::BusyTimeout:
        case Status::IrqTimeout:
        case Status::HalError:
        case Status::ChipError:
        case Status::RfFieldTimeout:
            return true;
        // Success, caller errors and guarded operations: chip state is fine.
        case Status::Ok:
        case Status::InvalidArgument:
        case Status::BufferTooSmall:
        case Status::WriteProtected:
        case Status::ReadOnly:
        case Status::Unsupported:
        // Card-level / RF-environment results: keep polling.
        case Status::NoResponse:
        case Status::Collision:
        case Status::IntegrityError:
        case Status::ProtocolError:
        case Status::CardError:
        case Status::RfActiveError:
        // Card content results: the card is not NDEF formatted / holds bad data.
        case Status::NotFormatted:
        case Status::MalformedData:
            return false;
    }
    return true;  // out-of-range value: treat as unknown device state
}

}  // namespace pn5180
