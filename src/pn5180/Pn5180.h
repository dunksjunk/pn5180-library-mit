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

    // CRC_RX_CONFIG.RX_BIT_ALIGN (bits 8:6): the first received bit is stored
    // at bit position `bits` (0..7) of the first rx byte, for bit-oriented
    // frames. Only these bits are changed. InvalidArgument if bits > 7.
    // RX_COLL_POS then includes this offset.  DS Table 92, Table 93
    Status setRxBitAlign(uint8_t bits);

    // SYSTEM_CONFIG.COMMAND = IDLE: stop any ongoing RF communication.  DS Table 75
    Status idle();

    // One RF exchange (DS Table 15 precondition; AN12650 4.1.1 lines 5-9):
    // IDLE, clear IRQs, COMMAND = Transceive, SEND_DATA, then wait up to
    // `timeoutMs` for RX_IRQ - on the IRQ pin when one is in use
    // (irqPinInUse(); IRQ_ENABLE routes RX_IRQ to it, DS Table 76), otherwise
    // by polling IRQ_STATUS over SPI - and read RX_STATUS and READ_DATA.
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

    // ---- IRQ pin (optional; Hal::hasIrq()) --------------------------------
    // When the HAL has an IRQ line, hardReset() reads IRQ_PIN_CONFIG (EEPROM
    // 0x1A, DS Table 51) for the pin polarity, and transceive() then waits
    // for RX_IRQ on the pin (IRQ_ENABLE routes only RX_IRQ, DS Table 76;
    // GENERAL_ERROR is non-maskable, DS 11.3.2.2) instead of polling
    // IRQ_STATUS over SPI. Timeouts, the IRQ clear sequence and the returned
    // Status values are identical to the polling path; the result is always
    // read from IRQ_STATUS / RX_STATUS. If RX_IRQ is found set in IRQ_STATUS
    // after a timeout although the pin never became active, IrqPinError is
    // returned instead of NoResponse. Never rewrites the EEPROM.
    bool irqPinInUse() const { return hal_.hasIrq() && irqConfigKnown_ && irqPinEnabled_; }
    // Force polling even when the HAL has an IRQ line (default: use the pin).
    void setUseIrqPin(bool enabled) { irqPinEnabled_ = enabled; }
    // IRQ_PIN_CONFIG as read by hardReset() / checkIrqPin(); valid when
    // irqPinConfigKnown().
    bool irqPinConfigKnown() const { return irqConfigKnown_; }
    bool irqPinActiveHigh() const { return irqActiveHigh_; }
    // Current level of the IRQ line translated to "an enabled IRQ is pending".
    bool irqPinActive();

    // Self-test of the IRQ line: with IRQ_ENABLE routing only TIMER2_IRQ the
    // pin must be inactive, become active when Timer2 expires (100 us,
    // DS Table 87, 90) and inactive again after IRQ_CLEAR (DS 11.3.2.1).
    // Returns Unsupported without an IRQ line, IrqPinError if the pin is
    // stuck / miswired / of the wrong polarity, IrqTimeout if TIMER2_IRQ
    // never appears in IRQ_STATUS. Leaves Timer2 stopped and IRQ_ENABLE = 0.
    Status checkIrqPin();

    // IRQ_ENABLE write (bits 16:0, DS Table 76), skipped if the cached value
    // already matches. hardReset() resets the cache to the IRQ_ENABLE reset
    // value; any writeRegister*() on IRQ_ENABLE invalidates it.
    Status setIrqEnable(uint32_t mask);

    // ---- Low-power modes (used by Lpcd) ----------------------------------
    // SWITCH_MODE LPCD (DS Table 18, Table 21). After success the chip must
    // not receive any instruction (DS Table 18); every command returns
    // LowPowerActive until leaveLowPower() or hardReset().
    Status switchModeLpcd(uint16_t wakeupCounterMs);
    bool lowPowerActive() const { return lowPower_; }
    // To be called once the IRQ pin signalled the end of LPCD/standby (normal
    // mode re-entered by the chip, DS Table 18 description): reads and
    // clears IRQ_STATUS into `irqStatus`, then hard-resets the chip so all
    // registers are in their defined default state (DS 11.8.10: register
    // configurations are not restored after wake-up).
    Status leaveLowPower(uint32_t& irqStatus);

    Hal& hal() { return hal_; }

    // Convenience helpers.
    // IRQ_STATUS as read now, merged with flags observed by the command error
    // check since the last call (see setCommandErrorCheck()).
    Status readIrqStatus(uint32_t& status);
    Status clearIrq(uint32_t mask);
    // Reads the versions; also records the firmware version for gating.
    Status readVersions(Versions& out);

    // Raw host-interface access, bypassing every check of this class. Must
    // not be used while lowPowerActive(): any NSS activity ends LPCD
    // (DS 11.8.10 item 3) without this object noticing. The Transport
    // therefore refuses frames with LowPowerActive while LPCD/standby is
    // armed (until leaveLowPower() or hardReset()).
    Transport& transport() { return transport_; }

    // Host delay through the HAL (for protocol layers built on this driver).
    void delayMs(uint32_t ms) { hal_.delayMs(ms); }

private:
    Status checkReady() const {
        return !begun_ ? Status::NotInitialized : (lowPower_ ? Status::LowPowerActive : Status::Ok);
    }
    Status readIrqPinConfig();
    // Waits until IRQ_STATUS has a bit of `mask` or GENERAL_ERROR (returned
    // in `irq`), on the IRQ pin if in use, else by polling. IrqTimeout after
    // `timeoutMs`; IrqPinError if the pin missed a set bit.
    Status waitIrq(uint32_t mask, uint32_t timeoutMs, uint32_t& irq);
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
    bool irqConfigKnown_ = false;
    bool irqActiveHigh_ = false;
    bool irqPinEnabled_ = true;
    bool irqEnableKnown_ = false;
    uint32_t irqEnable_ = 0;
    bool lowPower_ = false;
    // Largest command frame: SEND_DATA = code + valid-bits + 260 data bytes.
    uint8_t txBuf_[2 + spec::kSendDataMaxBytes] = {};
};

}  // namespace pn5180
