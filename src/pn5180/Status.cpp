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
    }
    return "Unknown";
}

}  // namespace pn5180
