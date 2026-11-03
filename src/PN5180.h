// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Umbrella header for the PN5180-MIT library (portable part only).
// Platform HALs are included separately, e.g. "PN5180ArduinoHal.h".
//
// Milestone 1 status: library layout, HAL interface, typed status codes and
// the BUSY-handshake transport exist. PN5180 command codes, register / EEPROM
// addresses, IRQ_STATUS bits and reset timings are intentionally NOT present
// yet: they must be taken (with citations) from the data sheet text, which was
// not readable in the iteration that created this file.
#pragma once

#include "pn5180/Hal.h"
#include "pn5180/Status.h"
#include "pn5180/Transport.h"
