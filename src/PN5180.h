// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Umbrella header for the PN5180-MIT library (portable part only).
// Platform HALs are included separately, e.g. "PN5180ArduinoHal.h".
#pragma once

#include "pn5180/Hal.h"
#include "pn5180/Iso14443a.h"
#include "pn5180/Iso15693.h"
#include "pn5180/Lpcd.h"
#include "pn5180/MemTest.h"
#include "pn5180/Ndef.h"
#include "pn5180/NdefTlv.h"
#include "pn5180/Pn5180.h"
#include "pn5180/Spec.h"
#include "pn5180/Status.h"
#include "pn5180/Transport.h"
#include "pn5180/Type2Tag.h"
#include "pn5180/Type5Tag.h"

/// @namespace pn5180
/// @brief PN5180-MIT: portable driver core (no Arduino.h, no OS headers) and
/// the platform HALs.
///
/// Start with pn5180::Pn5180 (chip commands) on a pn5180::Hal implementation
/// such as pn5180::ArduinoHal, then use pn5180::Iso14443a or
/// pn5180::Iso15693 to talk to cards. Every operation returns a
/// pn5180::Status.
