// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host-interface framing and BUSY handshake. Portable core: no Arduino.h.
#include "Transport.h"

namespace pn5180 {

Status Transport::waitBusy(bool level) {
    const uint32_t start = hal_.millis();
    while (hal_.busy() != level) {
        if (static_cast<uint32_t>(hal_.millis() - start) >= busyTimeoutMs_) {
            // One last sample to avoid a false timeout after a long preemption.
            return hal_.busy() == level ? Status::Ok : Status::BusyTimeout;
        }
    }
    return Status::Ok;
}

Status Transport::frame(const uint8_t* tx, uint8_t* rx, size_t len) {
    Status s = waitBusy(false);
    if (!ok(s)) {
        return s;
    }

    hal_.setNss(true);
    s = hal_.transfer(tx, rx, len);
    hal_.setNss(false);
    if (!ok(s)) {
        return s;
    }

    s = waitBusy(true);
    if (!ok(s)) {
        return s;
    }
    return waitBusy(false);
}

Status Transport::exchange(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) {
    if (tx == nullptr || txLen == 0 || (rx == nullptr && rxLen != 0)) {
        return Status::InvalidArgument;
    }

    Status s = frame(tx, nullptr, txLen);
    if (!ok(s) || rxLen == 0) {
        return s;
    }
    return frame(nullptr, rx, rxLen);
}

}  // namespace pn5180
