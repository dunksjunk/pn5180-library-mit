// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 14443-3 Type A activation. Portable core: no Arduino.h.
#include "Iso14443a.h"

#include "Spec.h"

namespace pn5180 {

using namespace iso14443a;

namespace {

// Errors after which a new activation can succeed: no answer or a garbled one.
bool retryable(Status s) {
    return s == Status::NoResponse || s == Status::Collision || s == Status::IntegrityError ||
           s == Status::ProtocolError || s == Status::CardError || s == Status::BufferTooSmall;
}

uint8_t clampAttempts(uint8_t attempts) {
    if (attempts < 1) {
        return 1;
    }
    return attempts > ActivateRetry::kMaxAttempts ? ActivateRetry::kMaxAttempts : attempts;
}

// UID bit `b` of a cascade level: bytes in transmission order, each byte
// LSB first.  ISO/IEC 14443-3 6.2.3.1, 6.5.3.2
bool getBit(const uint8_t* p, unsigned b) { return ((p[b / 8] >> (b % 8)) & 1u) != 0; }
void setBit(uint8_t* p, unsigned b, bool v) {
    const uint8_t m = static_cast<uint8_t>(1u << (b % 8));
    p[b / 8] = static_cast<uint8_t>(v ? (p[b / 8] | m) : (p[b / 8] & ~m));
}

bool sameUid(const CardA& a, const CardA& b) {
    if (a.uidLen != b.uidLen) {
        return false;
    }
    for (size_t i = 0; i < a.uidLen; ++i) {
        if (a.uid[i] != b.uid[i]) {
            return false;
        }
    }
    return true;
}

constexpr uint8_t kSel[kMaxCascadeLevels] = {SEL_CL1, SEL_CL2, SEL_CL3};

}  // namespace

Status Iso14443a::fieldOn(uint32_t guardMs) {
    guardMs_ = guardMs;
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
    // frames flagged as collisions instead of a clean ATQA. RX_GAIN 2 with
    // MIN_LEVEL 5 (Spec.h kSigproRmConfigIso14443a106: measured 0/110
    // failures with three cards on the antenna, vs 7/110 at the DS Table 172
    // default MIN_LEVEL 3) keeps noise below the subcarrier threshold.
    // DS Table 102 also warns that RX_GAIN 3 with a low MIN_LEVEL may raise no IRQ.
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
    const bool fullAtqa = n == 2 && lastBits == 0;  // ATQA is 16 bits  ISO/IEC 14443-3 6.5.2
    if (s == Status::Collision) {
        // All PICCs in IDLE answer REQA at the same time; differing ATQAs
        // collide and the PCD goes on with anticollision.  ISO/IEC 14443-3 6.5.2
        // RX_NUM_BYTES stops at the byte holding the collided bit (measured,
        // see anticollisionLoop), so a real ATQA collision has
        // RX_COLL_POS < 16 and exactly COLL_POS / 8 + 1 bytes; any other
        // collision-flagged frame is noise.  DS Table 93
        const uint32_t pos = (nfc_.lastRxStatus() >> spec::rxstatus::COLL_POS_SHIFT) &
                             spec::rxstatus::COLL_POS_MASK;
        if (pos >= 16 || n != pos / 8 + 1) {
            return Status::IntegrityError;
        }
        // Bits from the collision on are not meaningful (cleared).
        atqa = static_cast<uint16_t>(rx[0] | (rx[1] << 8));
        return Status::Ok;
    }
    if (!ok(s)) {
        return s;
    }
    if (!fullAtqa) {
        return Status::CardError;
    }
    atqa = static_cast<uint16_t>(rx[0] | (rx[1] << 8));  // LSB first  ISO/IEC 14443-3 6.5.2
    return Status::Ok;
}

Status Iso14443a::anticollision(uint8_t sel, uint8_t cl[kClBytes]) {
    bool aligned = false;
    Status s = anticollisionLoop(sel, cl, aligned);
    if (aligned) {
        // Every other exchange expects byte-aligned reception.  DS Table 92
        const Status a = nfc_.setRxBitAlign(0);
        if (ok(s) || requiresRecovery(a)) {
            s = a;  // a device error outranks a card-level one
        }
    }
    return s;
}

Status Iso14443a::anticollisionLoop(uint8_t sel, uint8_t cl[kClBytes], bool& aligned) {
    for (size_t i = 0; i < kClBytes; ++i) {
        cl[i] = 0;
    }
    // Anticollision frames carry no CRC.  ISO/IEC 14443-3 6.5.3.1
    Status s = nfc_.setCrc(false, false);  // DS Table 92, Table 99
    if (!ok(s)) {
        return s;
    }
    unsigned known = 0;       // UID CLn bits already fixed (cl bits 0..known-1)
    unsigned collisions = 0;  // collisions resolved at this cascade level
    bool flipped = false;     // the last chosen bit was already switched to 0
    for (;;) {
        // Bit-oriented frame: SEL, NVB, the `known` UID CLn bits; a partial
        // last byte sends only its low `extra` bits (LSB first). The card
        // continues with bit `known` in the same byte.  ISO/IEC 14443-3 6.2.3.3, 6.5.3.2
        const unsigned whole = known / 8;
        const unsigned extra = known % 8;
        uint8_t tx[2 + kClBytes] = {sel, nvb(static_cast<uint8_t>(known))};
        const size_t uidBytes = whole + (extra != 0 ? 1 : 0);
        for (size_t i = 0; i < uidBytes; ++i) {
            tx[2 + i] = cl[i];  // bits >= known are 0 and not sent
        }
        // Store the first answer bit at position `extra` of rx[0].  DS Table 92
        s = nfc_.setRxBitAlign(static_cast<uint8_t>(extra));
        if (!ok(s)) {
            return s;
        }
        aligned = extra != 0;
        uint8_t rx[kClBytes] = {};
        size_t n = 0;
        uint8_t lastBits = 0;
        // TX_LAST_BITS = extra (0 = whole last byte).  DS Table 15, Table 16
        s = nfc_.transceive(tx, 2 + uidBytes, static_cast<uint8_t>(extra), rx, sizeof(rx), n,
                            lastBits, timeoutMs_);
        if (s == Status::NoResponse && collisions > 0 && !flipped) {
            // No card answered with the chosen 1 at the collided bit: the
            // card(s) with a 1 there missed the frame (they stay READY,
            // ISO/IEC 14443-3 6.3). Continue with 0 there once, so the
            // other branch is still resolved.
            setBit(cl, known - 1, false);
            flipped = true;
            continue;
        }
        if (s != Status::Ok && s != Status::Collision) {
            return s;  // no answer or a garbled one (parity/framing, no collision)
        }
        const size_t expected = kClBytes - whole;  // bytes in the receive buffer
        unsigned end = kClBits;  // first bit not taken from this answer
        if (s == Status::Ok) {
            // The answer completes the 40 bits: (40 - known) bits stored from
            // bit `extra` end exactly on a byte boundary.  DS Table 93 RX_NUM_LAST_BITS
            if (n != expected || lastBits != 0) {
                return Status::CardError;
            }
        } else {
            // RX_COLL_POS: first collided data bit, counted from 0 in the
            // receive buffer including RX_BIT_ALIGN (DS Table 93). Measured
            // on the reference module (DS silent on both points): the base is
            // 0, and RX_NUM_BYTES_RECEIVED stops at the byte holding the
            // collided bit (e.g. RX_STATUS 0x840003: COLL_POS 16, 3 bytes).
            const uint32_t pos = (nfc_.lastRxStatus() >> spec::rxstatus::COLL_POS_SHIFT) &
                                 spec::rxstatus::COLL_POS_MASK;
            const uint32_t bit = 8u * whole + pos;
            // A collision flag is trusted only if the frame reaches the
            // collided bit and is not longer than the answer can be;
            // otherwise it is noise. Cards with the same 32 UID CLn bits
            // have the same BCC, so a real first collision lies in the UID
            // bits 0..31.  ISO/IEC 14443-3 6.5.3.2
            if (pos < extra || bit >= kUidClBits || n < pos / 8 + 1 || n > expected) {
                return Status::IntegrityError;
            }
            if (collisions >= kMaxCollisionRounds) {
                return Status::Collision;  // bound reached: not resolved
            }
            end = static_cast<unsigned>(bit);
        }
        for (unsigned b = known; b < end; ++b) {
            setBit(cl, b, getBit(rx, b - 8 * whole));
        }
        if (s == Status::Ok) {
            break;
        }
        // Policy: the collided bit is taken as 1.  ISO/IEC 14443-3 6.5.3.2
        setBit(cl, end, true);
        known = end + 1;
        ++collisions;
        flipped = false;
    }
    // BCC = XOR of the 4 UID CLn bytes.  ISO/IEC 14443-3 6.5.3.2
    if (static_cast<uint8_t>(cl[0] ^ cl[1] ^ cl[2] ^ cl[3]) != cl[4]) {
        return Status::IntegrityError;
    }
    return Status::Ok;
}

Status Iso14443a::selectCl(uint8_t sel, const uint8_t cl[kClBytes], uint8_t& sak) {
    // SELECT with NVB = 70h, CRC_A both ways.  ISO/IEC 14443-3 6.4.2, 6.5.3.3
    Status s = nfc_.setCrc(true, true);
    if (!ok(s)) {
        return s;
    }
    const uint8_t selCmd[2 + kClBytes] = {sel, NVB_SELECT, cl[0], cl[1], cl[2], cl[3], cl[4]};
    uint8_t sakRx[3] = {};  // SAK (+ CRC_A in case the PN5180 passes it on)
    size_t n = 0;
    uint8_t lastBits = 0;
    s = nfc_.transceive(selCmd, sizeof(selCmd), 0, sakRx, sizeof(sakRx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;
    }
    // The PN5180 checks CRC_A (RX_CRC_ENABLE, DS Table 92); the data sheet does
    // not state whether the CRC bytes are kept in the buffer, so accept both.
    if ((n != 1 && n != 3) || lastBits != 0) {
        return Status::CardError;
    }
    sak = sakRx[0];
    return Status::Ok;
}

Status Iso14443a::select(CardA& card) {
    card.uidLen = 0;
    card.sak = 0;
    for (size_t level = 0; level < kMaxCascadeLevels; ++level) {
        uint8_t cl[kClBytes] = {};
        uint8_t sak = 0;
        Status s = anticollision(kSel[level], cl);
        if (ok(s)) {
            s = selectCl(kSel[level], cl, sak);
        }
        if (!ok(s)) {
            card.uidLen = 0;
            return s;
        }
        if ((sak & SAK_CASCADE_BIT) != 0) {
            // UID incomplete: CLn = CT + 3 UID bytes.  ISO/IEC 14443-3 6.5.4
            if (cl[0] != CASCADE_TAG || level + 1 == kMaxCascadeLevels) {
                card.uidLen = 0;
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

Status Iso14443a::selectUid(const uint8_t* uid, size_t uidLen, CardA& card) {
    // Single 4, double 7, triple 10 bytes.  ISO/IEC 14443-3 6.5.4
    size_t levels = 0;
    if (uidLen == 4) {
        levels = 1;
    } else if (uidLen == 7) {
        levels = 2;
    } else if (uidLen == 10) {
        levels = 3;
    }
    if (uid == nullptr || levels == 0) {
        return Status::InvalidArgument;
    }
    uint8_t copy[10] = {};  // `uid` may point into `card`
    for (size_t i = 0; i < uidLen; ++i) {
        copy[i] = uid[i];
    }
    size_t next = 0;
    uint8_t sak = 0;
    for (size_t level = 0; level < levels; ++level) {
        const bool last = level + 1 == levels;
        // Not the last level: CT + 3 UID bytes.  ISO/IEC 14443-3 6.5.4
        uint8_t cl[kClBytes] = {};
        size_t i = 0;
        if (!last) {
            cl[i++] = CASCADE_TAG;
        }
        while (i < 4) {
            cl[i++] = copy[next++];
        }
        cl[4] = static_cast<uint8_t>(cl[0] ^ cl[1] ^ cl[2] ^ cl[3]);  // BCC  ISO/IEC 14443-3 6.5.3.2
        const Status s = selectCl(kSel[level], cl, sak);
        if (!ok(s)) {
            return s;
        }
        // SAK b3 set exactly while the UID is incomplete.  ISO/IEC 14443-3 6.5.3.4
        if (((sak & SAK_CASCADE_BIT) != 0) == last) {
            return Status::CardError;
        }
    }
    for (size_t i = 0; i < uidLen; ++i) {
        card.uid[i] = copy[i];
    }
    card.uidLen = static_cast<uint8_t>(uidLen);
    card.sak = sak;
    return Status::Ok;
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

Status Iso14443a::activateUid(const uint8_t* uid, size_t uidLen, CardA& card, bool wakeup) {
    if (uid == nullptr) {
        return Status::InvalidArgument;
    }
    uint16_t atqa = 0;
    Status s = request(atqa, wakeup);
    if (ok(s)) {
        s = selectUid(uid, uidLen, card);
    }
    if (ok(s)) {
        card.atqa = atqa;
    }
    return s;
}

Status Iso14443a::enumerate(CardA* cards, size_t max, size_t& count, bool wakeup,
                            ActivateRetry policy) {
    count = 0;
    if (cards == nullptr && max != 0) {
        return Status::InvalidArgument;
    }
    const uint8_t attempts = clampAttempts(policy.attempts);
    if (wakeup) {
        // Field reset: every PICC goes POWER-OFF -> IDLE, HALTed ones too.
        // (WUPA would not do: a card woken from HALT returns to HALT when it
        // loses the SELECT, so REQA would miss it.)  ISO/IEC 14443-3 5, 6.3
        Status r = fieldOff();
        if (ok(r)) {
            nfc_.delayMs(kFieldResetOffMs);
            r = fieldOn(guardMs_);
        }
        if (!ok(r)) {
            return r;
        }
    }
    unsigned failures = 0;
    Status lastError = Status::Ok;
    // Each step finds a new card, a duplicate (lost HLTA) or fails; bounded
    // even if HLTA never works.
    const size_t maxSteps = (max + 2) * (static_cast<size_t>(attempts) + 1);
    for (size_t step = 0; step < maxSteps; ++step) {
        CardA c;
        uint16_t atqa = 0;
        // REQA: HALTed cards stay silent.  ISO/IEC 14443-3 6.4
        Status s = request(atqa, false);
        if (s == Status::NoResponse && failures == 0) {
            return Status::Ok;  // no (more) cards
        }
        if (ok(s)) {
            c.atqa = atqa;
            s = select(c);
        }
        if (!ok(s)) {
            if (!retryable(s)) {
                return s;
            }
            // The failed card dropped back to IDLE (or HALT if WUPA woke it)
            // and answers a new request.  ISO/IEC 14443-3 6.3
            lastError = s;
            if (++failures >= attempts) {
                return lastError;
            }
            continue;
        }
        failures = 0;
        bool duplicate = false;
        for (size_t i = 0; i < count && !duplicate; ++i) {
            duplicate = sameUid(cards[i], c);
        }
        if (!duplicate && count >= max) {
            return Status::BufferTooSmall;  // one more card than fits; left ACTIVE
        }
        // ACTIVE -> HALT: it ignores REQA from now on.  ISO/IEC 14443-3 6.4.3
        s = halt();
        if (!ok(s) && !retryable(s)) {
            return s;
        }
        // A CardError here (an answer to HLTA) leaves its state unknown; if
        // it is still awake it is found again as a duplicate.
        if (!duplicate) {
            cards[count++] = c;
        }
    }
    return lastError != Status::Ok ? lastError : Status::CardError;
}

Status Iso14443a::activateWithRetry(CardA& card, bool wakeup, ActivateRetry policy) {
    const uint8_t attempts = clampAttempts(policy.attempts);
    Status s = activate(card, wakeup);
    for (uint8_t i = 1; i < attempts && retryable(s); ++i) {
        if (policy.resetField) {
            // POWER-OFF -> IDLE for every PICC.  ISO/IEC 14443-3 6.3
            Status r = fieldOff();
            if (ok(r)) {
                nfc_.delayMs(kFieldResetOffMs);
                r = fieldOn(guardMs_);
            }
            if (!ok(r)) {
                return r;
            }
        }
        // The PICC is back in IDLE (or HALT): restart with REQA/WUPA.  ISO/IEC 14443-3 6.3
        s = activate(card, wakeup);
    }
    return s;
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
