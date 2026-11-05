// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 15693 INVENTORY. Portable core: no Arduino.h.
#include "Iso15693.h"

#include "Spec.h"

namespace pn5180 {

using namespace iso15693;

namespace {

// Statuses meaning "a slot held an answer, but not a usable one". Anything
// else that is not Ok/NoResponse is a device error and aborts the inventory.
bool isGarbled(Status s) {
    return s == Status::Collision || s == Status::IntegrityError || s == Status::ProtocolError ||
           s == Status::CardError || s == Status::BufferTooSmall;
}

}  // namespace

Status Iso15693::fieldOn(uint32_t guardMs) {
    // ISO 15693 ASK100 26 kbit/s Tx 0x0D / Rx 0x8D.  DS Table 32; AN12650 4.2.3 line 1
    Status s = nfc_.loadRfConfig(spec::kRfConfigIso15693Ask100Tx, spec::kRfConfigIso15693Ask100Rx);
    if (ok(s)) {
        s = nfc_.rfOnAndWait();  // DS Table 37, Table 103; AN12650 4.2.3 line 2
    }
    if (ok(s)) {
        // Every ISO 15693 frame carries a CRC.  ISO/IEC 15693-3 (CRC); DS Tables 92, 99
        s = nfc_.setCrc(true, true);
    }
    if (ok(s) && guardMs > 0) {
        // Cards need >= kPowerUpMs of unmodulated field.  ISO/IEC 15693-3 (power-up)
        nfc_.delayMs(guardMs < kPowerUpMs ? kPowerUpMs : guardMs);
    }
    return s;
}

Status Iso15693::fieldOff() {
    return nfc_.rfOff();  // DS Table 38; AN12650 4.2.1 line 16
}

Status Iso15693::exchange(const uint8_t* tx, size_t txLen, CardV& card) {
    // Slack beyond the expected length so an overlong (garbled) frame is
    // still read and reported as CardError.
    uint8_t rx[kInventoryResponseBytes + kCrcBytes + 4] = {};
    size_t n = 0;
    uint8_t lastBits = 0;
    const Status s = nfc_.transceive(tx, txLen, 0, rx, sizeof(rx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;
    }
    // The PN5180 checks and removes the CRC (RX_CRC_ENABLE, DS Table 92):
    // on the test bed exactly 10 bytes arrive. 12 bytes would mean the CRC
    // check is off, which must not pass silently.
    if (n != kInventoryResponseBytes || lastBits != 0 || (rx[0] & RESP_FLAG_ERROR) != 0) {
        return Status::CardError;
    }
    // rx: flags, DSFID, UID LSB first.  ISO/IEC 15693-3 (inventory response, UID)
    if (rx[2 + kUidBytes - 1] != kUidMsb) {
        return Status::CardError;
    }
    card.dsfid = rx[1];
    for (size_t i = 0; i < kUidBytes; ++i) {
        card.uid[i] = rx[2 + kUidBytes - 1 - i];  // store MSB first
    }
    return Status::Ok;
}

Status Iso15693::inventory(CardV& card) {
    // flags 26h, INVENTORY, mask length 0.  ISO/IEC 15693-3 (inventory)
    const uint8_t req[3] = {kRequestFlags1Slot, CMD_INVENTORY, kMaskLengthNone};
    return exchange(req, sizeof(req), card);
}

Status Iso15693::runSlots(InventoryResult& result) {
    // Slot 0 is opened by the request itself.  AN12650 4.2.1 line 6
    const uint8_t req[3] = {kRequestFlags16Slots, CMD_INVENTORY, kMaskLengthNone};
    for (size_t slot = 0; slot < kSlots; ++slot) {
        CardV card;
        // Slots 1..15: zero-length SEND_DATA = EOF only (TX_CONFIG set by
        // inventory16()).  AN12650 4.2.1 line 15; DS Table 15 condition
        const Status s = slot == 0 ? exchange(req, sizeof(req), card) : exchange(nullptr, 0, card);
        if (ok(s)) {
            result.cards[result.count++] = card;
        } else if (isGarbled(s)) {
            result.collisionSlots = static_cast<uint16_t>(result.collisionSlots | (1u << slot));
        } else if (s != Status::NoResponse) {
            return s;  // device error
        }
        if (s != Status::NoResponse) {
            // A card answered: wait t2 before the next EOF.  ISO/IEC 15693-3 (t2)
            nfc_.delayMs(kT2Ms);
        }
        if (slot == 0) {
            // From now on send EOF only: clear TX_START_SYMBOL and
            // TX_DATA_ENABLE, keep the stop symbol.  AN12650 4.2.3 line 11; DS Table 98
            const Status m = nfc_.writeRegisterAndMask(spec::reg::TX_CONFIG,
                                                       spec::txconfig::kEofOnlyAndMask);
            if (!ok(m)) {
                return m;
            }
        }
    }
    return Status::Ok;
}

Status Iso15693::inventory16(InventoryResult& result) {
    result = InventoryResult{};
    uint32_t saved = 0;
    Status s = nfc_.readRegister(spec::reg::TX_CONFIG, saved);  // DS Table 98
    if (!ok(s)) {
        return s;
    }
    s = runSlots(result);
    // Restore TX_CONFIG on every path, so the next request carries SOF and data again.
    const Status r = nfc_.writeRegister(spec::reg::TX_CONFIG, saved);
    if (!ok(s)) {
        return s;
    }
    if (!ok(r)) {
        return r;
    }
    if (result.count > 0) {
        return Status::Ok;
    }
    return result.collisionSlots != 0 ? Status::Collision : Status::NoResponse;
}

size_t Iso15693::putHeader(uint8_t* req, uint8_t flags, uint8_t cmd, const uint8_t* uid) {
    // flags, command code, UID LSB first.  ISO/IEC 15693-3 (request format, UID)
    req[0] = flags;
    req[1] = cmd;
    for (size_t i = 0; i < kUidBytes; ++i) {
        req[2 + i] = uid[kUidBytes - 1 - i];
    }
    return kAddressedHeaderBytes;
}

Status Iso15693::addressed(const uint8_t* req, size_t reqLen, uint8_t* rx, size_t rxCap, size_t& n,
                           uint32_t timeoutMs) {
    lastErrorCode_ = 0;
    uint8_t lastBits = 0;
    // CRC on both ways was set by fieldOn(); the PN5180 strips the received
    // CRC (DS Table 92), so `n` excludes it.
    const Status s = nfc_.transceive(req, reqLen, 0, rx, rxCap, n, lastBits, timeoutMs);
    if (!ok(s)) {
        return s;
    }
    if (n < 1 || lastBits != 0) {
        return Status::CardError;
    }
    // Error flag set: one error code byte follows.  ISO/IEC 15693-3 (response flags, error codes)
    if ((rx[0] & RESP_FLAG_ERROR) != 0) {
        lastErrorCode_ = n == 2 ? rx[1] : 0;
        return Status::CardError;
    }
    return Status::Ok;
}

Status Iso15693::getSystemInfo(const uint8_t uid[kUidBytes], SystemInfo15693& info) {
    if (uid == nullptr) {
        return Status::InvalidArgument;
    }
    uint8_t req[kAddressedHeaderBytes] = {};
    const size_t len = putHeader(req, kRequestFlagsAddressed, CMD_GET_SYSTEM_INFO, uid);
    uint8_t rx[kSystemInfoMaxBytes + 4] = {};
    size_t n = 0;
    const Status s = addressed(req, len, rx, sizeof(rx), n, kSystemInfoTimeoutMs);
    if (!ok(s)) {
        return s;
    }
    // flags, info flags, UID (LSB first), then the optional fields in this
    // order: DSFID, AFI, memory size (2), IC reference.
    //                                     ISO/IEC 15693-3 (get system information response)
    if (n < 2 + kUidBytes) {
        return Status::CardError;
    }
    SystemInfo15693 out;
    out.infoFlags = rx[1];
    for (size_t i = 0; i < kUidBytes; ++i) {
        out.uid[i] = rx[2 + kUidBytes - 1 - i];
        if (out.uid[i] != uid[i]) {
            return Status::CardError;  // another card answered
        }
    }
    size_t p = 2 + kUidBytes;
    const size_t need = p + ((out.infoFlags & INFO_DSFID) ? 1 : 0) +
                        ((out.infoFlags & INFO_AFI) ? 1 : 0) +
                        ((out.infoFlags & INFO_MEMORY_SIZE) ? 2 : 0) +
                        ((out.infoFlags & INFO_IC_REFERENCE) ? 1 : 0);
    if (n != need) {
        return Status::CardError;
    }
    if (out.infoFlags & INFO_DSFID) {
        out.dsfid = rx[p++];
    }
    if (out.infoFlags & INFO_AFI) {
        out.afi = rx[p++];
    }
    if (out.infoFlags & INFO_MEMORY_SIZE) {
        // Both fields hold "value - 1".  ISO/IEC 15693-3 (VICC memory size)
        out.blockCount = static_cast<uint16_t>(rx[p] + 1u);
        out.blockSize = static_cast<uint8_t>((rx[p + 1] & kBlockSizeMask) + 1u);
        p += 2;
    }
    if (out.infoFlags & INFO_IC_REFERENCE) {
        out.icReference = rx[p];
    }
    info = out;
    return Status::Ok;
}

Status Iso15693::readBlock(const uint8_t uid[kUidBytes], uint8_t block, uint8_t* out,
                           size_t blockSize, bool withSecurity, uint8_t* security) {
    if (uid == nullptr || out == nullptr || blockSize == 0 || blockSize > kMaxBlockBytes ||
        (withSecurity && security == nullptr)) {
        return Status::InvalidArgument;
    }
    uint8_t req[kAddressedHeaderBytes + 1] = {};
    const uint8_t flags =
        static_cast<uint8_t>(kRequestFlagsAddressed | (withSecurity ? FLAG_OPTION : 0));
    size_t len = putHeader(req, flags, CMD_READ_SINGLE_BLOCK, uid);
    req[len++] = block;  // ISO/IEC 15693-3 (read single block)
    uint8_t rx[2 + kMaxBlockBytes + 4] = {};
    size_t n = 0;
    const Status s = addressed(req, len, rx, sizeof(rx), n, kReadBlockTimeoutMs);
    if (!ok(s)) {
        return s;
    }
    // flags [, block security status], data.  ISO/IEC 15693-3 (read single block response)
    const size_t head = withSecurity ? 2 : 1;
    if (n != head + blockSize) {
        return Status::CardError;
    }
    if (withSecurity) {
        *security = rx[1];
    }
    for (size_t i = 0; i < blockSize; ++i) {
        out[i] = rx[head + i];
    }
    return Status::Ok;
}

Status Iso15693::writeBlock(const uint8_t uid[kUidBytes], uint8_t block, const uint8_t* data,
                            size_t blockSize) {
    if (uid == nullptr || data == nullptr || blockSize == 0 || blockSize > kMaxBlockBytes) {
        return Status::InvalidArgument;
    }
    uint8_t req[kAddressedHeaderBytes + 1 + kMaxBlockBytes] = {};
    // Option flag 0: answer after programming.  ISO/IEC 15693-3 (write alternatives)
    size_t len = putHeader(req, kRequestFlagsAddressed, CMD_WRITE_SINGLE_BLOCK, uid);
    req[len++] = block;  // ISO/IEC 15693-3 (write single block)
    for (size_t i = 0; i < blockSize; ++i) {
        req[len++] = data[i];
    }
    uint8_t rx[4] = {};
    size_t n = 0;
    const Status s = addressed(req, len, rx, sizeof(rx), n, kWriteBlockTimeoutMs);
    if (!ok(s)) {
        return s;
    }
    return n == 1 ? Status::Ok : Status::CardError;  // flags only on success
}

}  // namespace pn5180
