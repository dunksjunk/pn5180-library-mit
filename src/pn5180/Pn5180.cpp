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
    Status s = checkReady();
    if (!ok(s)) {
        return s;
    }

    // RESET_N low for at least t(reset) = 10 us.  DS 11.2.2, Table 129
    hal_.setReset(true);
    hal_.delayUs(spec::kResetLowMinUs);
    hal_.setReset(false);
    irqShadow_ = 0;  // all registers return to defaults  DS 11.2.1.2
    fwKnown_ = false;

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
    return readVersions(v);  // records the firmware version for gating
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
