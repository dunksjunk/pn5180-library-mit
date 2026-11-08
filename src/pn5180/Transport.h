// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host-interface framing and BUSY handshake. Portable core: no Arduino.h.
//
// A host interface command is 1 SPI frame (write) or 2 SPI frames (write the
// instruction, then read the response).                          DS 11.4.3.1, AN12650 2.2
// Each frame follows the host BUSY recommendation of DS 11.4.1:
//   0. wait until BUSY is low (BUSY high = PN5180 cannot receive)  DS 11.4.1
//   1. assert NSS low
//   2. perform the data exchange (whole frame at once, no chaining)
//   3. wait until BUSY is high (optional in normal mode, mandatory with the
//      test bus enabled - configurable, enabled by default)
//   4. deassert NSS
//   5. wait until BUSY is low
// Response frames clock out 0xFF filler bytes.                     DS 11.4.1 Fig. 6, Fig. 9
#pragma once

#include <cstddef>
#include <cstdint>

#include "Hal.h"
#include "Status.h"

namespace pn5180 {

class Transport {
public:
    explicit Transport(Hal& hal) : hal_(hal) {}

    // Host-side timeout for each BUSY wait. The data sheet gives no maximum
    // BUSY duration, so this is an application policy value.
    void setBusyTimeoutMs(uint32_t ms) { busyTimeoutMs_ = ms; }
    uint32_t busyTimeoutMs() const { return busyTimeoutMs_; }

    // Step 3 of the DS 11.4.1 sequence (wait for BUSY high before NSS release).
    void setWaitForBusyHigh(bool enable) { waitForBusyHigh_ = enable; }
    bool waitForBusyHigh() const { return waitForBusyHigh_; }

    // One-frame command (no response).
    Status command(const uint8_t* tx, size_t txLen);

    // Two-frame command: instruction frame, then a `rxLen`-byte response
    // frame. Both frames run under one Hal::lock() so no other caller can
    // interleave.
    Status command(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen);

    // One-frame command that puts the chip into standby / LPCD (SWITCH_MODE,
    // DS Table 18): steps 0-4 only. Step 5 (wait for BUSY low) is skipped
    // because the data sheet does not describe BUSY while the chip is in
    // standby/LPCD, and nothing else may be sent until it wakes.
    Status commandNoCompletion(const uint8_t* tx, size_t txLen);

    // True while the owning Pn5180 has LPCD/standby armed: every command
    // returns LowPowerActive without touching NSS, because any NSS activity
    // ends LPCD (DS 11.8.10 item 3) and no instruction may be sent then
    // (DS Table 18). Set and cleared only by Pn5180.
    bool lowPowerGuard() const { return lowPowerGuard_; }

private:
    friend class Pn5180;
    void setLowPowerGuard(bool armed) { lowPowerGuard_ = armed; }
    Status waitBusy(bool level);
    Status frame(const uint8_t* tx, uint8_t* rx, size_t len, bool waitDone = true);

    Hal& hal_;
    uint32_t busyTimeoutMs_ = 100;  // host policy, see setBusyTimeoutMs()
    bool waitForBusyHigh_ = true;
    bool lowPowerGuard_ = false;
};

}  // namespace pn5180
