// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 14443-3 Type A activation. Portable core: no Arduino.h.
#include "Iso14443a.h"

#include "Spec.h"

namespace pn5180 {

using namespace iso14443a;

Status Iso14443a::fieldOn(uint32_t guardMs) {
    // ISO 14443-A 106 kbit/s Tx 0x00 / Rx 0x80.  DS Table 32; AN12650 4.1.3 line 1
    Status s = nfc_.loadRfConfig(spec::kRfConfigIso14443a106Tx, spec::kRfConfigIso14443a106Rx);
    if (ok(s)) {
        s = nfc_.rfOnAndWait();  // DS Table 37, Table 103; AN12650 4.1.3 line 2
    }
    if (ok(s)) {
        s = setupReceiver();
    }
    if (ok(s) && guardMs > 0) {
        nfc_.delayMs(guardMs);  // card power-up  ISO/IEC 14443-3 5
    }
    return s;
}

Status Iso14443a::setupReceiver() {
    // The receiver values loaded by LOAD_RF_CONFIG 0x80 can be overridden by
    // the EEPROM ARC table (DS 11.8.6). On the reference module they come out
    // as RX_GAIN 3 / MIN_LEVEL 4 (RF_CONTROL_RX 0x1F, SIGPRO_RM_CONFIG
    // 0x440DC), which on hardware produced false SOF detections and garbage
    // frames flagged as collisions instead of a clean ATQA. RX_GAIN 2 with the
    // documented SIGPRO_RM_CONFIG default (MIN_LEVEL 3) reads cards reliably.
    // DS Table 108 also warns that RX_GAIN 3 with a low MIN_LEVEL may raise no IRQ.
    // ARC applies its table after RF_ON, so these writes must follow RF_ON:
    // written before, they do not stick.
    Status s = nfc_.writeRegisterAndMask(spec::reg::RF_CONTROL_RX, ~spec::kRfControlRxGainMask);
    if (ok(s)) {
        s = nfc_.writeRegisterOrMask(spec::reg::RF_CONTROL_RX, spec::kRxGainIso14443a106);
    }
    if (ok(s)) {
        s = nfc_.writeRegister(spec::reg::SIGPRO_RM_CONFIG, spec::kSigproRmConfigIso14443a106);
    }
    return s;
}

Status Iso14443a::fieldOff() {
    return nfc_.rfOff();  // DS Table 38; AN12650 4.1.3 line 11
}

Status Iso14443a::request(uint16_t& atqa, bool wakeup) {
    // Short frame: no CRC either direction.  ISO/IEC 14443-3 6.2.3.1, 6.4.1
    Status s = nfc_.setCrc(false, false);  // AN12650 4.1.3 lines 3-4
    if (!ok(s)) {
        return s;
    }
    const uint8_t cmd = wakeup ? WUPA : REQA;
    uint8_t rx[2] = {};
    size_t n = 0;
    uint8_t lastBits = 0;
    // 7 valid bits in the only byte.  DS Table 16; AN12650 4.1.3 line 8
    s = nfc_.transceive(&cmd, 1, kShortFrameBits, rx, sizeof(rx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;
    }
    if (n != 2 || lastBits != 0) {  // ATQA is 16 bits  ISO/IEC 14443-3 6.5.2
        return Status::CardError;
    }
    atqa = static_cast<uint16_t>(rx[0] | (rx[1] << 8));  // LSB first  ISO/IEC 14443-3 6.5.2
    return Status::Ok;
}

Status Iso14443a::selectLevel(uint8_t sel, uint8_t clUid[4], uint8_t& sak) {
    // ANTICOLLISION with NVB = 20h, no CRC.  ISO/IEC 14443-3 6.4.2, 6.5.3
    Status s = nfc_.setCrc(false, false);
    if (!ok(s)) {
        return s;
    }
    const uint8_t ac[2] = {sel, NVB_ANTICOLLISION};
    uint8_t rx[5] = {};  // UID CLn (4 bytes) + BCC
    size_t n = 0;
    uint8_t lastBits = 0;
    s = nfc_.transceive(ac, sizeof(ac), 0, rx, sizeof(rx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;  // Collision is passed through unresolved (single-card scope)
    }
    if (n != 5 || lastBits != 0) {
        return Status::CardError;
    }
    // BCC = XOR of the 4 UID CLn bytes.  ISO/IEC 14443-3 6.5.3.2
    if (static_cast<uint8_t>(rx[0] ^ rx[1] ^ rx[2] ^ rx[3]) != rx[4]) {
        return Status::IntegrityError;
    }

    // SELECT with NVB = 70h, CRC_A both ways.  ISO/IEC 14443-3 6.4.2, 6.5.3.3
    s = nfc_.setCrc(true, true);
    if (!ok(s)) {
        return s;
    }
    const uint8_t selCmd[7] = {sel, NVB_SELECT, rx[0], rx[1], rx[2], rx[3], rx[4]};
    uint8_t sakRx[3] = {};  // SAK (+ CRC_A in case the PN5180 passes it on)
    s = nfc_.transceive(selCmd, sizeof(selCmd), 0, sakRx, sizeof(sakRx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;
    }
    // The PN5180 checks CRC_A (RX_CRC_ENABLE, DS Table 92); the data sheet does
    // not state whether the CRC bytes are kept in the buffer, so accept both.
    if ((n != 1 && n != 3) || lastBits != 0) {
        return Status::CardError;
    }
    for (size_t i = 0; i < 4; ++i) {
        clUid[i] = rx[i];
    }
    sak = sakRx[0];
    return Status::Ok;
}

Status Iso14443a::select(CardA& card) {
    static constexpr uint8_t kSel[kMaxCascadeLevels] = {SEL_CL1, SEL_CL2, SEL_CL3};
    card.uidLen = 0;
    card.sak = 0;
    for (size_t level = 0; level < kMaxCascadeLevels; ++level) {
        uint8_t cl[4] = {};
        uint8_t sak = 0;
        const Status s = selectLevel(kSel[level], cl, sak);
        if (!ok(s)) {
            return s;
        }
        if ((sak & SAK_CASCADE_BIT) != 0) {
            // UID incomplete: CLn = CT + 3 UID bytes.  ISO/IEC 14443-3 6.5.4
            if (cl[0] != CASCADE_TAG || level + 1 == kMaxCascadeLevels) {
                return Status::CardError;
            }
            for (size_t i = 1; i < 4; ++i) {
                card.uid[card.uidLen++] = cl[i];
            }
            continue;
        }
        for (size_t i = 0; i < 4; ++i) {
            card.uid[card.uidLen++] = cl[i];
        }
        card.sak = sak;
        return Status::Ok;
    }
    return Status::CardError;  // unreachable: last level always returns
}

Status Iso14443a::activate(CardA& card, bool wakeup) {
    uint16_t atqa = 0;
    Status s = request(atqa, wakeup);
    if (!ok(s)) {
        return s;
    }
    card.atqa = atqa;
    return select(card);
}

Status Iso14443a::halt() {
    // HLTA = 50h 00h + CRC_A.  ISO/IEC 14443-3 6.4.3
    Status s = nfc_.setCrc(true, false);
    if (!ok(s)) {
        return s;
    }
    const uint8_t cmd[2] = {HLTA_0, HLTA_1};
    uint8_t rx[4] = {};
    size_t n = 0;
    uint8_t lastBits = 0;
    s = nfc_.transceive(cmd, sizeof(cmd), 0, rx, sizeof(rx), n, lastBits, timeoutMs_);
    if (s == Status::NoResponse) {
        return Status::Ok;  // the card is HALTed and silent
    }
    return ok(s) ? Status::CardError : s;
}

}  // namespace pn5180
