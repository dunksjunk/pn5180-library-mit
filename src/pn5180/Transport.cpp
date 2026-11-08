// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host-interface framing and BUSY handshake (DS 11.4.1, 11.4.3.1).
// Portable core: no Arduino.h.
#include "Transport.h"

#include "Spec.h"

namespace pn5180 {

namespace {

class LockGuard {
public:
    explicit LockGuard(Hal& hal) : hal_(hal) { hal_.lock(); }
    ~LockGuard() { hal_.unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    Hal& hal_;
};

}  // namespace

Status Transport::waitBusy(bool level) {
    const uint32_t start = hal_.millis();
    for (;;) {
        if (hal_.busy() == level) {
            return Status::Ok;
        }
        const uint32_t elapsed = static_cast<uint32_t>(hal_.millis() - start);
        if (elapsed >= busyTimeoutMs_) {
            // Last sample guards against a false timeout after preemption.
            return hal_.busy() == level ? Status::Ok : Status::BusyTimeout;
        }
        // Spin (yielding) during the first millisecond for low latency, then
        // sleep so long BUSY phases don't monopolise the CPU.
        if (elapsed == 0) {
            hal_.yield();
        } else {
            hal_.delayMs(1);
        }
    }
}

Status Transport::frame(const uint8_t* tx, uint8_t* rx, size_t len, bool waitDone) {
    Status s = waitBusy(false);  // step 0
    if (!ok(s)) {
        return s;
    }

    s = hal_.beginFrame();  // step 1 (bus acquired, NSS low)
    if (!ok(s)) {
        return s;
    }
    s = hal_.transfer(tx, rx, len, spec::kReadFillerByte);  // step 2
    if (ok(s) && waitForBusyHigh_) {
        s = waitBusy(true);  // step 3
    }
    hal_.endFrame();  // step 4 (NSS high, bus released) - always paired
    if (!ok(s) || !waitDone) {
        return s;
    }
    return waitBusy(false);  // step 5
}

Status Transport::commandNoCompletion(const uint8_t* tx, size_t txLen) {
    if (tx == nullptr || txLen == 0) {
        return Status::InvalidArgument;
    }
    if (lowPowerGuard_) {
        return Status::LowPowerActive;  // DS Table 18, DS 11.8.10
    }
    LockGuard guard(hal_);
    return frame(tx, nullptr, txLen, false);
}

Status Transport::command(const uint8_t* tx, size_t txLen) {
    return command(tx, txLen, nullptr, 0);
}

Status Transport::command(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) {
    if (tx == nullptr || txLen == 0 || (rx == nullptr && rxLen != 0)) {
        return Status::InvalidArgument;
    }
    if (lowPowerGuard_) {
        return Status::LowPowerActive;  // DS Table 18, DS 11.8.10
    }

    LockGuard guard(hal_);
    Status s = frame(tx, nullptr, txLen);
    if (!ok(s) || rxLen == 0) {
        return s;
    }
    return frame(nullptr, rx, rxLen);
}

}  // namespace pn5180
