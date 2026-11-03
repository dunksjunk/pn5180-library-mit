// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host-interface framing and BUSY handshake. Portable core: no Arduino.h.
//
// Sequence per SPI frame (as required by the milestone task statement; to be
// cross-checked against the data sheet "logical host interface" section and
// AN12650 once spec text is available to the build - see open questions):
//   1. wait for BUSY low            (timeout -> Status::BusyTimeout)
//   2. assert NSS, transfer bytes, release NSS
//   3. wait for BUSY high, then low (timeouts -> Status::BusyTimeout)
// A command that returns data is followed by a separate read-back frame using
// the same handshake.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Hal.h"
#include "Status.h"

namespace pn5180 {

class Transport {
public:
    explicit Transport(Hal& hal) : hal_(hal) {}

    // Software timeout for each BUSY wait. This is a host-side policy value,
    // not a PN5180 timing constant; it can be tuned by the application.
    void setBusyTimeoutMs(uint32_t ms) { busyTimeoutMs_ = ms; }
    uint32_t busyTimeoutMs() const { return busyTimeoutMs_; }

    // Send one command frame. If `rx`/`rxLen` are non-zero, a second frame
    // reads `rxLen` response bytes back. Either part honours the handshake.
    Status exchange(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen);

private:
    Status waitBusy(bool level);
    Status frame(const uint8_t* tx, uint8_t* rx, size_t len);

    Hal& hal_;
    uint32_t busyTimeoutMs_ = 100;  // host policy default, see setBusyTimeoutMs()
};

}  // namespace pn5180
