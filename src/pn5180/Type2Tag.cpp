// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NFC Forum Type 2 Tag READ / WRITE / Capability Container. Portable core.
#include "Type2Tag.h"

#include "Spec.h"

namespace pn5180 {

using namespace t2t;

Status Type2Tag::read(uint8_t page, uint8_t out[kReadBytes]) {
    if (out == nullptr) {
        return Status::InvalidArgument;
    }
    // READ carries CRC_A both ways.  T2T (READ); ISO/IEC 14443-3 6.2.4; DS Tables 92, 99
    Status s = nfc_.setCrc(true, true);
    if (!ok(s)) {
        return s;
    }
    const uint8_t cmd[2] = {CMD_READ, page};
    // Slack so an overlong frame is read and reported, not truncated.
    uint8_t rx[kReadBytes + 4] = {};
    size_t n = 0;
    uint8_t lastBits = 0;
    s = nfc_.transceive(cmd, sizeof(cmd), 0, rx, sizeof(rx), n, lastBits, kReadTimeoutMs);
    // A NAK is a 4-bit frame; with CRC checking on the chip may also flag it
    // as a CRC error. Either way the tag refused the READ.  T2T (ACK and NAK)
    if ((ok(s) || s == Status::IntegrityError) && n == 1 && lastBits == kAckNakBits) {
        return Status::CardError;
    }
    if (!ok(s)) {
        return s;
    }
    // The PN5180 checks and removes CRC_A (RX_CRC_ENABLE, DS Table 92), so
    // exactly 16 bytes remain; 18 would mean CRC checking is off.
    if (n != kReadBytes || lastBits != 0) {
        return Status::CardError;
    }
    for (size_t i = 0; i < kReadBytes; ++i) {
        out[i] = rx[i];
    }
    return Status::Ok;
}

Status Type2Tag::capability(T2tCapability& cc) {
    uint8_t buf[kReadBytes] = {};
    const Status s = read(kCcPage, buf);  // pages 3..6, CC is the first 4 bytes
    if (!ok(s)) {
        return s;
    }
    T2tCapability c;
    for (size_t i = 0; i < kPageBytes; ++i) {
        c.raw[i] = buf[i];
    }
    // T2T (Capability Container): CC0 magic, CC1 version, CC2 size/8, CC3 access.
    if (c.raw[0] != kCcMagic || c.raw[2] == 0) {
        ccValid_ = false;
        return Status::CardError;
    }
    c.version = c.raw[1];
    c.dataBytes = static_cast<uint16_t>(c.raw[2] * kCcSizeUnitBytes);
    // Data area = pages 4 .. 4 + size/4 - 1; CC2 <= 255 keeps it below page 255.
    c.lastPage = static_cast<uint8_t>(kFirstDataPage + c.dataBytes / kPageBytes - 1);
    c.readOnly = (c.raw[3] & kCcWriteAccessMask) != kCcAccessGranted;
    cc_ = c;
    ccValid_ = true;
    cc = c;
    return Status::Ok;
}

Status Type2Tag::write(uint8_t page, const uint8_t in[kPageBytes]) {
    if (in == nullptr) {
        return Status::InvalidArgument;
    }
    if (!ccValid_) {
        T2tCapability c;
        const Status s = capability(c);
        if (!ok(s)) {
            return s;
        }
    }
    // Never write the UID/lock/CC pages or beyond the declared data area.
    if (page < cc_.firstPage || page > cc_.lastPage) {
        return Status::InvalidArgument;
    }
    if (cc_.readOnly) {
        return Status::ReadOnly;
    }
    const uint8_t frame[2 + kPageBytes] = {CMD_WRITE, page, in[0], in[1], in[2], in[3]};
    return writeExchange(frame, sizeof(frame));
}

Status Type2Tag::writeExchange(const uint8_t* frame, size_t len) {
    lastAckNak_ = 0xFF;
    // WRITE goes out with CRC_A; the 4-bit ACK/NAK comes back without one.
    // T2T (WRITE, ACK and NAK); DS Table 99 TX_CRC_ENABLE, Table 92 RX_CRC_ENABLE
    uint32_t savedRx = 0;
    Status s = nfc_.readRegister(spec::reg::CRC_RX_CONFIG, savedRx);
    if (!ok(s)) {
        return s;
    }
    s = nfc_.setCrc(true, false);
    size_t n = 0;
    uint8_t lastBits = 0;
    uint8_t rx[4] = {};
    if (ok(s)) {
        s = nfc_.transceive(frame, len, 0, rx, sizeof(rx), n, lastBits, kWriteTimeoutMs);
    }
    // Restore CRC_RX_CONFIG on every path.  DS Table 92
    const Status r = nfc_.writeRegister(spec::reg::CRC_RX_CONFIG, savedRx);
    if (!ok(s)) {
        return s;
    }
    if (!ok(r)) {
        return r;
    }
    // RX_STATUS: 1 byte with RX_NUM_LAST_BITS = 4.  DS Table 93; T2T (ACK and NAK)
    if (n != 1 || lastBits != kAckNakBits) {
        return Status::CardError;
    }
    lastAckNak_ = static_cast<uint8_t>(rx[0] & kAckNakMask);
    return lastAckNak_ == kAck ? Status::Ok : Status::CardError;
}

}  // namespace pn5180
