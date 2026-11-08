// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PN5180 driver: host interface commands. Portable core: no Arduino.h.
#include "Pn5180.h"

namespace pn5180 {

namespace {

// 32-bit register values are transferred little endian.  DS 11.4.3.1
void putLe32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint32_t getLe32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

// Range check shared by READ_EEPROM / WRITE_EEPROM: address 0..254,
// 1..255 bytes, last address <= 254.  DS Table 12, Table 13
bool eepromRangeOk(uint8_t address, size_t len) {
    return len >= 1 && len <= spec::kEepromMaxTransfer && address <= spec::kEepromMaxAddress &&
           static_cast<size_t>(address) + len - 1 <= spec::kEepromMaxAddress;
}

// Host policy for the IRQ pin self-test: Timer2 expires after 100 us
// (spec::timer::kSelfTestReload); allow generous scheduling slack.
constexpr uint32_t kIrqSelfTestTimeoutMs = 10;

bool versionAtLeast(const Version& v, uint8_t major, uint8_t minor) {
    return v.major > major || (v.major == major && v.minor >= minor);
}

}  // namespace

Status Pn5180::begin() {
    const Status s = hal_.begin();
    begun_ = ok(s);
    return s;
}

// ---------------------------------------------------------------------------
// Low-level register access without validation or error check
// ---------------------------------------------------------------------------
Status Pn5180::rawWriteRegister(uint8_t command, uint8_t reg, uint32_t value) {
    // [code][address][value LE x4]  DS Tables 6, 7, 8
    uint8_t frame[2 + spec::kRegisterValueBytes];
    frame[0] = command;
    frame[1] = reg;
    putLe32(&frame[2], value);
    return transport_.command(frame, sizeof(frame));
}

Status Pn5180::rawReadRegister(uint8_t reg, uint32_t& value) {
    // [code][address] -> 4-byte LE response  DS Table 10
    const uint8_t frame[2] = {spec::cmd::READ_REGISTER, reg};
    uint8_t rsp[spec::kRegisterValueBytes] = {};
    const Status s = transport_.command(frame, sizeof(frame), rsp, sizeof(rsp));
    if (ok(s)) {
        value = getLe32(rsp);
    }
    return s;
}

Status Pn5180::finish(Status s) {
    if (!ok(s) || !errorCheck_) {
        return s;
    }
    // A violated command condition raises GENERAL_ERROR_IRQ.  DS 11.4.1, 11.4.3.1, Table 77
    uint32_t irq = 0;
    s = rawReadRegister(spec::reg::IRQ_STATUS, irq);
    if (!ok(s)) {
        return s;
    }
    // Keep other flags: the read may have auto-cleared them.  DS Table 51 (0x1A bit 1)
    irqShadow_ |= irq & spec::irq::ALL & ~spec::irq::GENERAL_ERROR;
    if ((irq & spec::irq::GENERAL_ERROR) == 0) {
        return Status::Ok;
    }
    // Clear the flag so the next command starts clean.  DS Table 78 bit 17
    s = rawWriteRegister(spec::cmd::WRITE_REGISTER, spec::reg::IRQ_CLEAR, spec::irq::GENERAL_ERROR);
    return ok(s) ? Status::ChipError : s;
}

Status Pn5180::checkRegister(uint8_t reg, bool write) const {
    if (!spec::reg::isDocumented(reg)) {
        return Status::InvalidArgument;
    }
    if (write && spec::reg::isReadOnly(reg)) {
        return Status::ReadOnly;
    }
    if (reg == spec::reg::FELICA_EMD_CONTROL && fwKnown_ &&
        !versionAtLeast(fw_, spec::reg::kFelicaEmdMinFwMajor, spec::reg::kFelicaEmdMinFwMinor)) {
        return Status::Unsupported;
    }
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------
Status Pn5180::hardReset(uint32_t bootTimeoutMs) {
    // Allowed while LPCD/standby is armed: a reset is the documented way
    // out.  DS Table 18 description
    if (!begun_) {
        return Status::NotInitialized;
    }
    Status s = Status::Ok;

    // RESET_N low for at least t(reset) = 10 us.  DS 11.2.2, Table 129
    hal_.setReset(true);
    hal_.delayUs(spec::kResetLowMinUs);
    hal_.setReset(false);
    lowPower_ = false;
    transport_.setLowPowerGuard(false);
    irqShadow_ = 0;  // all registers return to defaults  DS 11.2.1.2
    fwKnown_ = false;
    irqEnable_ = 0;  // IRQ_ENABLE reset value 0  DS Table 76
    irqEnableKnown_ = true;

    // Typical start-up time before the host interface is usable.  DS Table 141
    hal_.delayMs(spec::kBootTypicalMs);

    // End of start-up is signalled by IDLE_IRQ.  DS 11.2.2, Table 77
    const uint32_t start = hal_.millis();
    for (;;) {
        uint32_t irq = 0;
        s = rawReadRegister(spec::reg::IRQ_STATUS, irq);
        if (!ok(s)) {
            return s;
        }
        if ((irq & spec::irq::IDLE) != 0) {
            s = finish(rawWriteRegister(spec::cmd::WRITE_REGISTER, spec::reg::IRQ_CLEAR, spec::irq::IDLE));
            break;
        }
        if (static_cast<uint32_t>(hal_.millis() - start) >= bootTimeoutMs) {
            return Status::IrqTimeout;
        }
        hal_.delayMs(1);
    }
    if (!ok(s)) {
        return s;
    }
    Versions v;
    s = readVersions(v);  // records the firmware version for gating
    if (ok(s) && hal_.hasIrq()) {
        s = readIrqPinConfig();
    }
    return s;
}

// ---------------------------------------------------------------------------
// IRQ pin
// ---------------------------------------------------------------------------
Status Pn5180::readIrqPinConfig() {
    // Polarity of the IRQ pin; read only, never rewritten.  DS 11.3.2.1, Table 51
    uint8_t cfg = 0;
    const Status s = readEeprom(spec::eeprom::IRQ_PIN_CONFIG, &cfg, 1);
    if (ok(s)) {
        irqActiveHigh_ = (cfg & spec::eeprom::kIrqPinActiveHigh) != 0;
        irqConfigKnown_ = true;
    }
    return s;
}

bool Pn5180::irqPinActive() { return hal_.hasIrq() && hal_.irq() == irqActiveHigh_; }

Status Pn5180::setIrqEnable(uint32_t mask) {
    mask &= spec::irq::ENABLE_WRITABLE;  // bits 19:17 are fixed to 1, read-only  DS Table 76
    if (irqEnableKnown_ && irqEnable_ == mask) {
        return checkReady();
    }
    const Status s = writeRegister(spec::reg::IRQ_ENABLE, mask);
    irqEnableKnown_ = ok(s);
    irqEnable_ = mask;
    return s;
}

Status Pn5180::waitIrq(uint32_t mask, uint32_t timeoutMs, uint32_t& irq) {
    const bool usePin = irqPinInUse();
    const uint32_t want = mask | spec::irq::GENERAL_ERROR;
    if (usePin && (irqShadow_ & want) != 0) {
        // Already seen by a command's error check (finish()): with
        // auto-clear-on-read (DS Table 51, 0x1A bit 1) that read cleared the
        // flag, so the pin will not become active for it. Consume it the
        // way readIrqStatus() does.
        return readIrqStatus(irq);
    }
    const uint32_t start = hal_.millis();
    for (;;) {
        // With the pin, IRQ_STATUS is only read once the pin reports a
        // pending enabled IRQ (DS 11.3.2.1); otherwise it is polled.
        if (!usePin || irqPinActive()) {
            const Status s = readIrqStatus(irq);
            if (!ok(s)) {
                return s;
            }
            if ((irq & want) != 0) {
                return Status::Ok;
            }
        }
        // Strictly greater: millis() granularity must not shorten the wait.
        if (static_cast<uint32_t>(hal_.millis() - start) > timeoutMs) {
            break;
        }
        hal_.yield();
    }
    if (!usePin) {
        return Status::IrqTimeout;
    }
    // Pin never became active: cross-check IRQ_STATUS once so a dead pin is
    // reported as such, not as a missing card. The pin is sampled before the
    // read (with auto-clear-on-read the read itself releases it, DS Table 51
    // 0x1A bit 1) and again after it (an IRQ that arrived between the two
    // is the pin's due, not a failure). IrqPinError only if both show it
    // inactive although IRQ_STATUS has the wanted bit.
    const bool pinBefore = irqPinActive();
    const Status s = readIrqStatus(irq);
    if (!ok(s)) {
        return s;
    }
    if ((irq & want) == 0) {
        return Status::IrqTimeout;
    }
    return (pinBefore || irqPinActive()) ? Status::Ok : Status::IrqPinError;
}

Status Pn5180::checkIrqPin() {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if (!hal_.hasIrq()) {
        return Status::Unsupported;
    }
    if (!irqConfigKnown_) {
        s = readIrqPinConfig();
        if (!ok(s)) {
            return s;
        }
    }
    // Quiet state: Timer2 stopped, only TIMER2_IRQ routed, all flags cleared.
    s = writeRegister(spec::reg::TIMER2_CONFIG, 0);  // DS Table 90
    if (ok(s)) {
        s = setIrqEnable(spec::irq::TIMER2);  // DS Table 76 bit 13
    }
    if (ok(s)) {
        s = clearIrq(spec::irq::ALL);  // DS Table 78
    }
    if (ok(s)) {
        s = writeRegister(spec::reg::TIMER2_RELOAD, spec::timer::kSelfTestReload);  // DS Table 87
    }
    if (!ok(s)) {
        return s;
    }
    Status result = Status::Ok;
    if (irqPinActive()) {
        result = Status::IrqPinError;  // active with nothing pending: stuck / wrong polarity
    }
    // Raise TIMER2_IRQ: start Timer2 now.  DS Table 90 bits 6, 0
    if (ok(result)) {
        s = writeRegister(spec::reg::TIMER2_CONFIG, spec::timer::ENABLE | spec::timer::START_NOW);
        if (!ok(s)) {
            return s;
        }
        // Watch only the pin (no SPI) until it shows the IRQ or time runs out.
        const uint32_t start = hal_.millis();
        while (!irqPinActive() &&
               static_cast<uint32_t>(hal_.millis() - start) <= kIrqSelfTestTimeoutMs) {
            hal_.yield();
        }
        const bool pin = irqPinActive();
        uint32_t irq = 0;
        s = readIrqStatus(irq);
        if (!ok(s)) {
            return s;
        }
        if ((irq & spec::irq::TIMER2) == 0) {
            // Timer never fired: a chip problem, not the pin (unless the pin
            // shows something else, e.g. GENERAL_ERROR, which is fine).
            result = (irq & spec::irq::GENERAL_ERROR) != 0 ? Status::ChipError : Status::IrqTimeout;
        } else if (!pin) {
            result = Status::IrqPinError;  // flag set, pin silent: stuck / unconnected
        }
    }
    // Clearing the flag must release the pin.  DS 11.3.2.1
    if (ok(result)) {
        s = clearIrq(spec::irq::TIMER2);
        if (!ok(s)) {
            return s;
        }
        if (irqPinActive()) {
            result = Status::IrqPinError;
        }
    }
    // Restore: Timer2 stopped, no IRQ routed, flags cleared.
    s = writeRegister(spec::reg::TIMER2_CONFIG, 0);
    if (ok(s)) {
        s = setIrqEnable(0);
    }
    if (ok(s)) {
        s = clearIrq(spec::irq::TIMER2 | spec::irq::GENERAL_ERROR);
    }
    return ok(result) ? s : result;
}

// ---------------------------------------------------------------------------
// Low-power modes
// ---------------------------------------------------------------------------
Status Pn5180::switchModeLpcd(uint16_t wakeupCounterMs) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if (wakeupCounterMs < spec::switchmode::kWakeupCounterMinMs ||
        wakeupCounterMs > spec::switchmode::kWakeupCounterMaxMs) {
        return Status::InvalidArgument;  // DS Table 18 condition, Table 21
    }
    // [0x0B][mode = 1 (LPCD)][wake-up counter, 2 bytes LSB first]
    // DS Table 18, Table 21; byte order: see spec::switchmode.
    const uint8_t frame[4] = {spec::cmd::SWITCH_MODE, spec::switchmode::kModeLpcd,
                              static_cast<uint8_t>(wakeupCounterMs),
                              static_cast<uint8_t>(wakeupCounterMs >> 8)};
    // No GENERAL_ERROR check afterwards: reading IRQ_STATUS would be an
    // instruction during LPCD, which the DS forbids (Table 18) and which
    // would end LPCD (DS 11.8.10 item 3, "NSS on Host IF").
    s = transport_.commandNoCompletion(frame, sizeof(frame));
    if (ok(s)) {
        lowPower_ = true;
        transport_.setLowPowerGuard(true);  // raw transport() frames would end LPCD too
    }
    return s;
}

Status Pn5180::leaveLowPower(uint32_t& irqStatus) {
    if (!begun_) {
        return Status::NotInitialized;
    }
    // The chip signalled the end of the mode: it is back in normal mode
    // (DS Table 18 description), so instructions are allowed again.
    lowPower_ = false;
    transport_.setLowPowerGuard(false);
    irqStatus = 0;
    Status s = readIrqStatus(irqStatus);
    if (ok(s)) {
        s = clearIrq(spec::irq::ALL);
    }
    // Reset regardless, so a failed read still ends in a defined state.
    const Status r = hardReset();
    return ok(s) ? r : s;
}

// ---------------------------------------------------------------------------
// Register commands
// ---------------------------------------------------------------------------
Status Pn5180::registerCommand(uint8_t command, uint8_t reg, uint32_t value) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    s = checkRegister(reg, true);
    if (!ok(s)) {
        return s;
    }
    if (reg == spec::reg::IRQ_ENABLE) {
        // Any write (also OR/AND masks) makes the setIrqEnable() cache stale;
        // setIrqEnable() re-validates it after its own write.
        irqEnableKnown_ = false;
    }
    return finish(rawWriteRegister(command, reg, value));
}

Status Pn5180::writeRegister(uint8_t reg, uint32_t value) {
    return registerCommand(spec::cmd::WRITE_REGISTER, reg, value);
}

Status Pn5180::writeRegisterOrMask(uint8_t reg, uint32_t orMask) {
    return registerCommand(spec::cmd::WRITE_REGISTER_OR_MASK, reg, orMask);
}

Status Pn5180::writeRegisterAndMask(uint8_t reg, uint32_t andMask) {
    return registerCommand(spec::cmd::WRITE_REGISTER_AND_MASK, reg, andMask);
}

Status Pn5180::readRegister(uint8_t reg, uint32_t& value) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    s = checkRegister(reg, false);
    if (!ok(s)) {
        return s;
    }
    if (reg == spec::reg::IRQ_STATUS) {
        return readIrqStatus(value);
    }
    uint32_t v = 0;
    s = finish(rawReadRegister(reg, v));
    if (ok(s)) {
        value = v;
    }
    return s;
}

Status Pn5180::readIrqStatus(uint32_t& status) {
    const Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    uint32_t irq = 0;
    const Status r = rawReadRegister(spec::reg::IRQ_STATUS, irq);
    if (!ok(r)) {
        return r;
    }
    // GENERAL_ERROR is reported to the caller in the value, not as a failure
    // of this read.
    status = irq | irqShadow_;
    irqShadow_ = 0;
    return Status::Ok;
}

Status Pn5180::clearIrq(uint32_t mask) {
    const Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    mask &= spec::irq::ALL;
    irqShadow_ &= ~mask;
    return finish(rawWriteRegister(spec::cmd::WRITE_REGISTER, spec::reg::IRQ_CLEAR, mask));
}

// ---------------------------------------------------------------------------
// EEPROM
// ---------------------------------------------------------------------------
Status Pn5180::readEeprom(uint8_t address, uint8_t* data, size_t len) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if (data == nullptr || !eepromRangeOk(address, len)) {
        return Status::InvalidArgument;
    }
    // [code][address][length] -> `len` bytes  DS Table 13
    const uint8_t frame[3] = {spec::cmd::READ_EEPROM, address, static_cast<uint8_t>(len)};
    return finish(transport_.command(frame, sizeof(frame), data, len));
}

Status Pn5180::writeEeprom(uint8_t address, const uint8_t* data, size_t len) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if (data == nullptr || !eepromRangeOk(address, len)) {
        return Status::InvalidArgument;
    }
    // Writes run upward from `address` (DS Table 12), so a start address at or
    // above the first writable byte keeps the whole range out of the
    // read-only area 0x00..0x15 (DS Table 51).
    if (!eepromWriteEnabled_ || address < spec::eeprom::kFirstWritableAddress) {
        return Status::WriteProtected;
    }
    // [code][address][data...]  DS Table 12
    txBuf_[0] = spec::cmd::WRITE_EEPROM;
    txBuf_[1] = address;
    for (size_t i = 0; i < len; ++i) {
        txBuf_[2 + i] = data[i];
    }
    return finish(transport_.command(txBuf_, 2 + len));
}

// ---------------------------------------------------------------------------
// RF data and configuration
// ---------------------------------------------------------------------------
Status Pn5180::sendData(const uint8_t* data, size_t len, uint8_t validBitsInLastByte) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if ((data == nullptr && len != 0) || len > spec::kSendDataMaxBytes ||
        validBitsInLastByte > spec::kSendDataMaxValidBits) {
        return Status::InvalidArgument;
    }
    // [code][valid bits in last byte][data...]  DS Table 15, Table 16
    txBuf_[0] = spec::cmd::SEND_DATA;
    txBuf_[1] = validBitsInLastByte;
    for (size_t i = 0; i < len; ++i) {
        txBuf_[2 + i] = data[i];
    }
    return finish(transport_.command(txBuf_, 2 + len));
}

Status Pn5180::readData(uint8_t* data, size_t len) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    if (data == nullptr || len == 0 || len > spec::kReadDataMaxBytes) {
        return Status::InvalidArgument;
    }
    // [code][0x00] -> `len` bytes (host chooses how many to read)  DS Table 17
    const uint8_t frame[2] = {spec::cmd::READ_DATA, spec::kReadDataParam};
    return finish(transport_.command(frame, sizeof(frame), data, len));
}

Status Pn5180::loadRfConfig(uint8_t txConfig, uint8_t rxConfig) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    const bool txOk = txConfig <= spec::kRfConfigTxMax || txConfig == spec::kRfConfigUnchanged;
    const bool rxOk = (rxConfig >= spec::kRfConfigRxMin && rxConfig <= spec::kRfConfigRxMax) ||
                      rxConfig == spec::kRfConfigUnchanged;
    if (!txOk || !rxOk) {
        return Status::InvalidArgument;
    }
    // [code][tx config][rx config]  DS Table 31
    const uint8_t frame[3] = {spec::cmd::LOAD_RF_CONFIG, txConfig, rxConfig};
    return finish(transport_.command(frame, sizeof(frame)));
}

Status Pn5180::rfOn(uint8_t flags) {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    constexpr uint8_t kKnown = spec::kRfOnDisableCollisionAvoidance | spec::kRfOnActiveCommunication;
    if ((flags & static_cast<uint8_t>(~kKnown)) != 0) {
        return Status::InvalidArgument;
    }
    // [code][flags]  DS Table 37
    const uint8_t frame[2] = {spec::cmd::RF_ON, flags};
    return finish(transport_.command(frame, sizeof(frame)));
}

Status Pn5180::rfOff() {
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }
    // [code][dummy]  DS Table 38
    const uint8_t frame[2] = {spec::cmd::RF_OFF, spec::kRfOffDummy};
    return finish(transport_.command(frame, sizeof(frame)));
}

Status Pn5180::rfOnAndWait(uint8_t flags, uint32_t timeoutMs) {
    Status s = clearIrq(spec::irq::RF_ACTIVE_ERROR | spec::irq::TX_RFON);  // DS Table 78
    if (ok(s)) {
        s = rfOn(flags);  // DS Table 37
    }
    if (!ok(s)) {
        return s;
    }
    const uint32_t start = hal_.millis();
    for (;;) {
        uint32_t irq = 0;
        s = readIrqStatus(irq);
        if (!ok(s)) {
            return s;
        }
        // External field seen during collision avoidance: field stays off.  DS 11.8.2
        if ((irq & spec::irq::RF_ACTIVE_ERROR) != 0) {
            (void)clearIrq(spec::irq::RF_ACTIVE_ERROR);  // report RfActiveError regardless
            return Status::RfActiveError;
        }
        uint32_t rf = 0;
        s = readRegister(spec::reg::RF_STATUS, rf);
        if (!ok(s)) {
            return s;
        }
        if ((rf & spec::rfstatus::TX_RF_STATUS) != 0) {  // DS Table 103 bit 17
            return Status::Ok;
        }
        if (static_cast<uint32_t>(hal_.millis() - start) > timeoutMs) {
            return Status::RfFieldTimeout;
        }
        hal_.yield();
    }
}

Status Pn5180::setCrc(bool txEnabled, bool rxEnabled) {
    Status s = txEnabled ? writeRegisterOrMask(spec::reg::CRC_TX_CONFIG, spec::kCrcTxEnable)
                         : writeRegisterAndMask(spec::reg::CRC_TX_CONFIG, ~spec::kCrcTxEnable);
    if (!ok(s)) {
        return s;
    }
    s = rxEnabled ? writeRegisterOrMask(spec::reg::CRC_RX_CONFIG, spec::kCrcRxEnable)
                  : writeRegisterAndMask(spec::reg::CRC_RX_CONFIG, ~spec::kCrcRxEnable);
    return s;
}

Status Pn5180::setRxBitAlign(uint8_t bits) {
    if (bits > 7) {
        return Status::InvalidArgument;
    }
    // Clear RX_BIT_ALIGN, then OR in the new value.  DS Table 92
    Status s = writeRegisterAndMask(spec::reg::CRC_RX_CONFIG, ~spec::kCrcRxBitAlignMask);
    if (ok(s) && bits != 0) {
        s = writeRegisterOrMask(spec::reg::CRC_RX_CONFIG,
                                static_cast<uint32_t>(bits) << spec::kCrcRxBitAlignShift);
    }
    return s;
}

Status Pn5180::idle() {
    // AND mask clears COMMAND (bits 2:0) to IDLE.  DS Table 75; AN12650 4.1.3 line 6
    return writeRegisterAndMask(spec::reg::SYSTEM_CONFIG, ~spec::kSystemConfigCommandMask);
}

Status Pn5180::transceive(const uint8_t* tx, size_t txLen, uint8_t txLastBits, uint8_t* rx,
                          size_t rxCap, size_t& rxLen, uint8_t& rxLastBits, uint32_t timeoutMs) {
    rxLen = 0;
    rxLastBits = 0;
    lastRxStatus_ = 0;  // stays 0 unless a frame is received
    if (rx == nullptr && rxCap != 0) {
        return Status::InvalidArgument;
    }
    // With the IRQ pin, route only RX_IRQ to it (GENERAL_ERROR is
    // non-maskable).  DS Table 76, DS 11.3.2.2
    Status s = irqPinInUse() ? setIrqEnable(spec::irq::RX) : checkReady();
    // Sequence of AN12650 4.1.1 lines 5-9; SEND_DATA requires the transceiver
    // in WaitTransmit with the Transceive command set (DS Table 15 condition).
    if (ok(s)) {
        s = clearIrq(spec::irq::ALL);  // AN12650 4.1.3 line 5
    }
    if (ok(s)) {
        s = idle();  // AN12650 4.1.3 line 6
    }
    if (ok(s)) {
        // AN12650 4.1.3 line 7; COMMAND was just cleared, so OR yields 011b.  DS Table 75
        s = writeRegisterOrMask(spec::reg::SYSTEM_CONFIG, spec::kCommandTransceive);
    }
    if (ok(s)) {
        s = sendData(tx, txLen, txLastBits);  // DS Table 15
    }
    if (!ok(s)) {
        return s;
    }

    // Wait for end of reception: RX_IRQ.  DS Table 77 bit 0; AN12650 4.1.2 line 9
    uint32_t irq = 0;
    s = waitIrq(spec::irq::RX, timeoutMs, irq);
    if (s == Status::IrqTimeout || s == Status::IrqPinError) {
        // Stop the receiver so the next exchange starts clean.  DS Table 75 (IDLE/StopCom)
        const Status i = idle();
        if (!ok(i)) {
            return i;
        }
        return s == Status::IrqTimeout ? Status::NoResponse : s;
    }
    if (!ok(s)) {
        return s;
    }
    if ((irq & spec::irq::GENERAL_ERROR) != 0) {  // DS Table 77 bit 17
        (void)clearIrq(spec::irq::GENERAL_ERROR);  // report ChipError regardless
        return Status::ChipError;
    }

    uint32_t st = 0;
    s = readRegister(spec::reg::RX_STATUS, st);  // DS Table 93
    if (!ok(s)) {
        return s;
    }
    lastRxStatus_ = st;
    const size_t n = st & spec::rxstatus::NUM_BYTES_MASK;
    rxLastBits = static_cast<uint8_t>((st >> spec::rxstatus::NUM_LAST_BITS_SHIFT) &
                                      spec::rxstatus::NUM_LAST_BITS_MASK);
    if (n > rxCap) {
        return Status::BufferTooSmall;
    }
    if (n > 0) {
        s = readData(rx, n);  // DS Table 17
        if (!ok(s)) {
            return s;
        }
    }
    rxLen = n;
    // A collision usually also breaks parity, so it is reported first.  DS Table 93
    if ((st & spec::rxstatus::COLLISION_DETECTED) != 0) {
        return Status::Collision;
    }
    if ((st & spec::rxstatus::PROTOCOL_ERROR) != 0) {
        return Status::ProtocolError;
    }
    if ((st & spec::rxstatus::DATA_INTEGRITY_ERROR) != 0) {
        return Status::IntegrityError;
    }
    return Status::Ok;
}

Status Pn5180::readVersions(Versions& out) {
    // Product, firmware and EEPROM versions are contiguous (0x10..0x15).  DS Table 51
    uint8_t raw[3 * spec::eeprom::kVersionBytes] = {};
    const Status s = readEeprom(spec::eeprom::PRODUCT_VERSION, raw, sizeof(raw));
    if (!ok(s)) {
        return s;
    }
    Version* const v[3] = {&out.product, &out.firmware, &out.eeprom};
    for (size_t i = 0; i < 3; ++i) {
        const uint8_t* p = &raw[i * spec::eeprom::kVersionBytes];
        v[i]->minor = p[spec::eeprom::kVersionMinorOffset];
        v[i]->major = p[spec::eeprom::kVersionMajorOffset];
    }
    fw_ = out.firmware;
    fwKnown_ = true;
    return Status::Ok;
}

}  // namespace pn5180
