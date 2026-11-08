// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Low-power card detection. Portable core: no Arduino.h.
#include "Lpcd.h"

#include "Spec.h"

namespace pn5180 {

namespace {

// EEPROM 0x34..0x38 are read in one command.                 DS Table 51, Table 73
constexpr uint8_t kFirst = spec::eeprom::LPCD_REFERENCE_VALUE;
constexpr size_t kSpan = spec::eeprom::LPCD_REFVAL_GPO_CONTROL - kFirst + 1;
constexpr size_t idx(uint8_t address) { return static_cast<size_t>(address - kFirst); }

}  // namespace

Status Lpcd::readSettings(LpcdSettings& out) {
    uint8_t raw[kSpan] = {};
    const Status s = nfc_.readEeprom(kFirst, raw, sizeof(raw));
    if (!ok(s)) {
        return s;
    }
    out.gear = raw[idx(spec::eeprom::LPCD_REFERENCE_VALUE)] & spec::eeprom::kLpcdGearMask;
    out.fieldOnTime = raw[idx(spec::eeprom::LPCD_FIELD_ON_TIME)];
    out.threshold = raw[idx(spec::eeprom::LPCD_THRESHOLD)];
    // Only 00b (auto) and 01b (self) are documented; 10b/11b are RFU.  DS Table 73
    const uint8_t mode = raw[idx(spec::eeprom::LPCD_REFVAL_GPO_CONTROL)] & spec::eeprom::kLpcdModeMask;
    if (mode == spec::eeprom::kLpcdModeSelfCalibration) {
        out.calibration = LpcdCalibration::Self;
    } else if (mode == spec::eeprom::kLpcdModeAutoCalibration) {
        out.calibration = LpcdCalibration::Auto;
    } else {
        return Status::MalformedData;
    }
    return Status::Ok;
}

Status Lpcd::configure(const LpcdSettings& settings, size_t& bytesWritten) {
    bytesWritten = 0;
    if (settings.gear > spec::eeprom::kLpcdGearMask ||
        (settings.calibration != LpcdCalibration::Auto && settings.calibration != LpcdCalibration::Self)) {
        return Status::InvalidArgument;
    }
    uint8_t raw[kSpan] = {};
    Status s = nfc_.readEeprom(kFirst, raw, sizeof(raw));
    if (!ok(s)) {
        return s;
    }
    // Wanted image: only the documented LPCD fields change; RFU bits of 0x34
    // and the GPO control bits 4:2 of 0x38 keep their stored values.  DS Table 73
    uint8_t want[kSpan];
    for (size_t i = 0; i < kSpan; ++i) {
        want[i] = raw[i];
    }
    uint8_t& ref = want[idx(spec::eeprom::LPCD_REFERENCE_VALUE)];
    ref = static_cast<uint8_t>((ref & ~spec::eeprom::kLpcdGearMask) | settings.gear);
    want[idx(spec::eeprom::LPCD_FIELD_ON_TIME)] = settings.fieldOnTime;
    want[idx(spec::eeprom::LPCD_THRESHOLD)] = settings.threshold;
    const uint8_t mode = settings.calibration == LpcdCalibration::Self
                             ? spec::eeprom::kLpcdModeSelfCalibration
                             : spec::eeprom::kLpcdModeAutoCalibration;
    uint8_t& ctl = want[idx(spec::eeprom::LPCD_REFVAL_GPO_CONTROL)];
    ctl = static_cast<uint8_t>((ctl & ~spec::eeprom::kLpcdModeMask) | mode);

    // Read, compare, write only the differing bytes (EEPROM endurance).
    // 0x35 is RFU (DS Table 51) and always equals its stored value here.
    const bool wasEnabled = nfc_.eepromWriteEnabled();
    nfc_.setEepromWriteEnabled(true);
    for (size_t i = 0; i < kSpan && ok(s); ++i) {
        if (want[i] != raw[i]) {
            s = nfc_.writeEeprom(static_cast<uint8_t>(kFirst + i), &want[i], 1);  // DS Table 12
            if (ok(s)) {
                ++bytesWritten;
            }
        }
    }
    nfc_.setEepromWriteEnabled(wasEnabled);
    if (!ok(s) || bytesWritten == 0) {
        return s;
    }
    // Verify what the EEPROM now holds.
    s = nfc_.readEeprom(kFirst, raw, sizeof(raw));
    if (!ok(s)) {
        return s;
    }
    for (size_t i = 0; i < kSpan; ++i) {
        if (raw[i] != want[i]) {
            return Status::ChipError;
        }
    }
    return Status::Ok;
}

Status Lpcd::start(uint16_t wakeupCounterMs) {
    if (!nfc_.hal().hasIrq()) {
        return Status::Unsupported;  // the wake cannot be observed without SPI
    }
    if (wakeupCounterMs < spec::switchmode::kWakeupCounterMinMs ||
        wakeupCounterMs > spec::switchmode::kWakeupCounterMaxMs) {
        return Status::InvalidArgument;  // DS Table 18 condition, Table 21
    }
    // The wake is only visible on the pin, so prove the pin works first.
    // This also leaves IRQ_ENABLE = 0, so only the non-maskable LPCD /
    // GENERAL_ERROR / HV_ERROR IRQs drive it (DS 11.3.2.2).
    Status s = nfc_.checkIrqPin();
    if (ok(s)) {
        s = nfc_.clearIrq(spec::irq::ALL);  // DS Table 78
    }
    if (ok(s)) {
        // Field off so SWITCH_MODE starts from a defined state (LPCD switches
        // the field on itself for its measurements, DS 11.8.10). The DS
        // (Table 18 description) does not say SWITCH_MODE fails with the
        // field on, and the chip cannot be queried after the switch without
        // ending LPCD (DS 11.8.10 item 3), so there is no post-check.
        s = nfc_.rfOff();  // DS Table 38
    }
    if (!ok(s)) {
        return s;
    }
    if (nfc_.irqPinActive()) {
        return Status::IrqPinError;
    }
    uint8_t ctl = 0;
    s = nfc_.readEeprom(spec::eeprom::LPCD_REFVAL_GPO_CONTROL, &ctl, 1);
    if (!ok(s)) {
        return s;
    }
    if ((ctl & spec::eeprom::kLpcdModeMask) == spec::eeprom::kLpcdModeSelfCalibration) {
        // Reading AGC_REF_CONFIG without a prior write runs the calibration
        // whose gear/AGC the self-calibration mode uses.  DS Table 112, Figure 40
        uint32_t agcRef = 0;
        s = nfc_.readRegister(spec::reg::AGC_REF_CONFIG, agcRef);
        if (!ok(s)) {
            return s;
        }
    }
    s = nfc_.switchModeLpcd(wakeupCounterMs);  // DS Table 18, Table 21
    if (ok(s)) {
        startMs_ = nfc_.hal().millis();
    }
    return s;
}

Status Lpcd::waitForWake(uint32_t timeoutMs, LpcdWake& wake) {
    if (!nfc_.lowPowerActive()) {
        return Status::InvalidArgument;  // start() first
    }
    // Pin only: any SPI frame would be a forbidden instruction and would
    // itself end LPCD.  DS Table 18, DS 11.8.10
    const uint32_t t0 = nfc_.hal().millis();
    while (!nfc_.irqPinActive()) {
        if (static_cast<uint32_t>(nfc_.hal().millis() - t0) >= timeoutMs) {
            return Status::IrqTimeout;  // still armed
        }
        nfc_.hal().delayMs(1);
    }
    wake.elapsedMs = static_cast<uint32_t>(nfc_.hal().millis() - startMs_);
    const Status s = nfc_.leaveLowPower(wake.irqStatus);
    wake.reason = (wake.irqStatus & spec::irq::LPCD) != 0 ? LpcdWakeReason::CardDetected
                                                           : LpcdWakeReason::Other;  // DS Table 77 bit 19
    return s;
}

Status Lpcd::cancel() { return nfc_.hardReset(); }

}  // namespace pn5180
