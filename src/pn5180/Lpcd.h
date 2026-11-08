// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Low-power card detection (LPCD) on top of Pn5180. Portable core header:
// no Arduino.h, no OS headers, no heap, no exceptions.
//
// How the PN5180 does it (DS 11.8.10, Figure 40):
//   - SWITCH_MODE LPCD (DS Table 18, Table 21) first takes a reference AGC
//     value with the ISO 14443-A 106 RF configuration and the field on,
//     either with the gear from EEPROM LPCD_REFERENCE_VALUE (auto
//     calibration) or with the gear/AGC from register AGC_REF_CONFIG (self
//     calibration), selected by EEPROM LPCD_REFVAL_GPO_CONTROL bits 1:0.
//   - It then loops: standby for the wake-up counter period, field on for
//     LPCD_FIELD_ON_TIME, measure AGC; if (reference - actual) >
//     LPCD_THRESHOLD it raises LPCD_IRQ (non-maskable, DS 11.3.2.2) and
//     returns to normal mode, otherwise it goes back to standby.
//   - The loop also ends on a reset, on NSS activity on the host interface
//     and on an RF level detection (DS 11.8.10 items 2-4). No instruction
//     may be sent while it runs (DS Table 18).
//
// Consequences for this driver:
//   - The wake can only be observed on the IRQ pin: polling IRQ_STATUS
//     needs SPI frames, which are forbidden and would themselves end LPCD.
//     So Lpcd requires Hal::hasIrq() and returns Unsupported without it.
//   - The reference is taken when LPCD starts, with whatever is on the
//     antenna at that moment. LPCD therefore detects a CHANGE relative to
//     that moment (a card or metal added, which loads the antenna and lowers
//     the AGC value), not the presence of a card that was already there.
//   - After the wake the register configuration is lost (DS 11.8.10), so
//     the driver hard-resets the chip; a following activate()/inventory()
//     starts from the defined power-on state.
//
// Single owner: an Lpcd shares its Pn5180 with the protocol objects; do not
// use any of them while LPCD is armed (they return LowPowerActive).
#pragma once

#include <cstddef>
#include <cstdint>

#include "Pn5180.h"
#include "Status.h"

namespace pn5180 {

// Source of the LPCD reference value: EEPROM 0x38 bits 1:0.  DS Table 73
enum class LpcdCalibration : uint8_t {
    Auto = 0,  // 00b: calibrate once with the gear in LPCD_REFERENCE_VALUE
    Self = 1,  // 01b: use gear/AGC from register AGC_REF_CONFIG (DS Table 112)
};

struct LpcdSettings {
    // EEPROM 0x37: wake when (reference AGC - actual AGC) > threshold.  DS Table 73
    uint8_t threshold = 0;
    // EEPROM 0x36: RF-on time before the AGC measurement, unit 8 us.     DS Table 73
    uint8_t fieldOnTime = 0;
    // EEPROM 0x38 bits 1:0.                                               DS Table 73
    LpcdCalibration calibration = LpcdCalibration::Auto;
    // EEPROM 0x34 bits 3:0: AGC gear for auto calibration (0..15).        DS Table 51, 73
    uint8_t gear = 0;
};

enum class LpcdWakeReason : uint8_t {
    CardDetected,  // LPCD_IRQ set                              DS Table 77 bit 19
    Other,         // the pin fired without LPCD_IRQ (see irqStatus)
};

struct LpcdWake {
    LpcdWakeReason reason = LpcdWakeReason::Other;
    uint32_t irqStatus = 0;  // IRQ_STATUS read right after the wake  DS Table 77
    uint32_t elapsedMs = 0;  // since start()
};

class Lpcd {
public:
    explicit Lpcd(Pn5180& nfc) : nfc_(nfc) {}

    // Reads the LPCD EEPROM settings (0x34..0x38). MalformedData if 0x38
    // bits 1:0 hold a reserved calibration mode (10b/11b, DS Table 73); the
    // other fields of `out` are filled even then, `out.calibration` is left
    // unchanged. configure() overwrites such a value with the requested mode.
    Status readSettings(LpcdSettings& out);

    // Writes the settings to EEPROM, byte by byte and only where the stored
    // value differs (EEPROM write endurance); RFU / GPO bits sharing a byte
    // are preserved. `bytesWritten` counts the EEPROM bytes written (0 when
    // everything already matched). InvalidArgument for gear > 15 or an
    // unknown calibration mode. The Pn5180 EEPROM write guard is enabled
    // only for these writes and restored afterwards.
    Status configure(const LpcdSettings& settings, size_t& bytesWritten);

    // Enters LPCD (SWITCH_MODE mode 1) with `wakeupCounterMs` (1..1023,
    // see spec::switchmode::kWakeupCounterMaxMs: DS Table 21 vs. Figure 40)
    // of standby between two detection cycles. Unsupported without an IRQ
    // pin, InvalidArgument outside the range. Switches the RF field off
    // (RF_OFF) before SWITCH_MODE. With self calibration, AGC_REF_CONFIG is
    // read first to run the calibration (DS Table 112).
    Status start(uint16_t wakeupCounterMs);

    // Waits on the IRQ pin only (no SPI) for up to `timeoutMs`. IrqTimeout:
    // nothing happened, LPCD is still armed - call again or cancel(). Ok:
    // the chip woke; `wake` holds the reason, IRQ_STATUS has been read and
    // cleared and the chip hard-reset, so it is ready for normal use.
    Status waitForWake(uint32_t timeoutMs, LpcdWake& wake);

    // Leaves LPCD by hard reset (the only forced exit, DS Table 18).
    Status cancel();

    bool armed() const { return nfc_.lowPowerActive(); }

private:
    Pn5180& nfc_;
    uint32_t startMs_ = 0;
};

}  // namespace pn5180
