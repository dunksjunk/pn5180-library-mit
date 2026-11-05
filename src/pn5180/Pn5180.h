// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PN5180 driver: host interface commands on top of Transport.
// Portable core: no Arduino.h, no OS headers, no heap, no exceptions.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Hal.h"
#include "Spec.h"
#include "Status.h"
#include "Transport.h"

namespace pn5180 {

struct Version {
    uint8_t major = 0;
    uint8_t minor = 0;
};

struct Versions {
    Version product;   // EEPROM 0x10  DS Table 51
    Version firmware;  // EEPROM 0x12  DS Table 51
    Version eeprom;    // EEPROM 0x14  DS Table 51
};

// Thread safety: none. A Pn5180 and every protocol object built on it
// (Iso14443a, Iso15693, Type2Tag, MemTest, ...) form one single-owner unit. Protocol operations
// span several chip commands and temporarily change chip registers (e.g.
// TX_CONFIG, CRC settings) that they restore afterwards, so two tasks must
// never use the same chip concurrently. If several tasks need the reader,
// the application serialises access (e.g. one mutex held around each whole
// protocol operation, from fieldOn() to fieldOff()).
class Pn5180 {
public:
    explicit Pn5180(Hal& hal) : hal_(hal), transport_(hal) {}

    // Initialise the HAL. Must succeed before any other call.
    Status begin();

    // Hardware reset via RESET_N, then wait for the IDLE IRQ that signals the
    // end of start-up (DS 11.2.2). The IDLE IRQ is cleared afterwards.
    // Returns IrqTimeout if IDLE_IRQ_STAT is not seen within `bootTimeoutMs`
    // (the boot time depends on EEPROM XTAL_BOOT_TIME, DS Table 141).
    Status hardReset(uint32_t bootTimeoutMs = 50);

    // Command error checking (default on). After each command the driver reads
    // IRQ_STATUS; if GENERAL_ERROR_IRQ is set (command exception, DS 11.4.1,
    // 11.4.3.1) it clears that flag and returns ChipError. Other IRQ flags seen
    // by this check are kept and merged into the next readIrqStatus(), so they
    // are not lost when the EEPROM selects auto-clear-on-read (DS Table 51,
    // IRQ_PIN_CONFIG bit 1).
    void setCommandErrorCheck(bool enabled) { errorCheck_ = enabled; }
    bool commandErrorCheck() const { return errorCheck_; }

    // Register access (DS 11.4.3.3). `reg` must be a documented register
    // address (spec::reg::isDocumented), otherwise InvalidArgument. Writes to
    // read-only registers return ReadOnly. FELICA_EMD_CONTROL returns
    // Unsupported when the firmware is known (after hardReset() or
    // readVersions()) to be older than 4.1.
    Status writeRegister(uint8_t reg, uint32_t value);
    Status writeRegisterOrMask(uint8_t reg, uint32_t orMask);
    Status writeRegisterAndMask(uint8_t reg, uint32_t andMask);
    Status readRegister(uint8_t reg, uint32_t& value);

    // EEPROM access (DS Table 12, Table 13). Address 0..254, 1..255 bytes,
    // and the range must not extend beyond address 254.
    Status readEeprom(uint8_t address, uint8_t* data, size_t len);

    // EEPROM writes are refused (WriteProtected) until explicitly enabled,
    // and never allowed into the documented read-only area 0x00..0x15
    // (DS Table 51).
    void setEepromWriteEnabled(bool enabled) { eepromWriteEnabled_ = enabled; }
    bool eepromWriteEnabled() const { return eepromWriteEnabled_; }
    Status writeEeprom(uint8_t address, const uint8_t* data, size_t len);

    // SEND_DATA: 0..260 bytes, `validBitsInLastByte` 0..7 (0 = all bits).
    // Precondition (caller): SYSTEM_CONFIG.COMMAND set to Transceive.  DS Table 15
    Status sendData(const uint8_t* data, size_t len, uint8_t validBitsInLastByte = 0);

    // READ_DATA: read `len` (1..508) bytes from the reception buffer.  DS Table 17
    Status readData(uint8_t* data, size_t len);

    // LOAD_RF_CONFIG: tx 0x00..0x1C, rx 0x80..0x9C, or 0xFF to keep.  DS Table 31
    Status loadRfConfig(uint8_t txConfig, uint8_t rxConfig);

    // RF_ON with flags from spec::kRfOn* (DS Table 37); RF_OFF (DS Table 38).
    Status rfOn(uint8_t flags = 0);
    Status rfOff();

    // RF_ON, then poll until RF_STATUS.TX_RF_STATUS shows the field is on
    // (DS Table 103). Returns RfActiveError if RF_ACTIVE_ERROR_IRQ is raised
    // because an external field was detected (DS 11.8.2, Table 77), or
    // RfFieldTimeout if the field is not on within `timeoutMs` (host policy;
    // the data sheet gives no maximum).
    Status rfOnAndWait(uint8_t flags = 0, uint32_t timeoutMs = 20);

    // Enable/disable CRC generation (Tx) and checking (Rx) by toggling only
    // bit 0 of CRC_TX_CONFIG / CRC_RX_CONFIG; the CRC preset and type come
    // from the loaded RF configuration.  DS Table 92, Table 99; AN12650 4.1.3
    Status setCrc(bool txEnabled, bool rxEnabled);

    // SYSTEM_CONFIG.COMMAND = IDLE: stop any ongoing RF communication.  DS Table 75
    Status idle();

    // One RF exchange (DS Table 15 precondition; AN12650 4.1.1 lines 5-9):
    // IDLE, clear IRQs, COMMAND = Transceive, SEND_DATA, then poll IRQ_STATUS
    // for RX_IRQ for up to `timeoutMs`, read RX_STATUS and READ_DATA.
    //   txLastBits: valid bits of the last tx byte, 0 = all 8 (DS Table 16)
    //   rx/rxCap:   receive buffer (may be nullptr if rxCap == 0)
    //   rxLen:      bytes received (also set on Collision/IntegrityError/
    //               ProtocolError so callers can inspect partial data)
    //   rxLastBits: valid bits of the last rx byte, 0 = all (DS Table 93)
    // Returns NoResponse if RX_IRQ does not appear in time (the transceiver
    // is then set back to IDLE), Collision / IntegrityError / ProtocolError
    // per RX_STATUS, BufferTooSmall if more than rxCap bytes arrived.
    Status transceive(const uint8_t* tx, size_t txLen, uint8_t txLastBits, uint8_t* rx,
                      size_t rxCap, size_t& rxLen, uint8_t& rxLastBits, uint32_t timeoutMs);

    // RX_STATUS read by the last transceive(); 0 if it received no frame.
    uint32_t lastRxStatus() const { return lastRxStatus_; }

    // Convenience helpers.
    // IRQ_STATUS as read now, merged with flags observed by the command error
    // check since the last call (see setCommandErrorCheck()).
    Status readIrqStatus(uint32_t& status);
    Status clearIrq(uint32_t mask);
    // Reads the versions; also records the firmware version for gating.
    Status readVersions(Versions& out);

    Transport& transport() { return transport_; }

    // Host delay through the HAL (for protocol layers built on this driver).
    void delayMs(uint32_t ms) { hal_.delayMs(ms); }

private:
    Status checkReady() const { return begun_ ? Status::Ok : Status::NotInitialized; }
    Status checkRegister(uint8_t reg, bool write) const;
    Status registerCommand(uint8_t command, uint8_t reg, uint32_t value);
    Status rawWriteRegister(uint8_t command, uint8_t reg, uint32_t value);
    Status rawReadRegister(uint8_t reg, uint32_t& value);
    // Runs the GENERAL_ERROR check after a command whose transport status is `s`.
    Status finish(Status s);

    Hal& hal_;
    Transport transport_;
    bool begun_ = false;
    bool eepromWriteEnabled_ = false;
    bool errorCheck_ = true;
    uint32_t irqShadow_ = 0;
    uint32_t lastRxStatus_ = 0;
    bool fwKnown_ = false;
    Version fw_;
    // Largest command frame: SEND_DATA = code + valid-bits + 260 data bytes.
    uint8_t txBuf_[2 + spec::kSendDataMaxBytes] = {};
};

}  // namespace pn5180
