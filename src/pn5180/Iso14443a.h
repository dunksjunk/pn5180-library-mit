// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 14443-3 Type A activation (REQA/WUPA, anticollision, SELECT,
// HLTA) on top of Pn5180::transceive(). Portable core: no Arduino.h, no heap.
//
// Card-side protocol per the public standard ISO/IEC 14443-3; PN5180 setup
// per the NXP data sheet (DS) and AN12650.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Pn5180.h"
#include "Status.h"

namespace pn5180 {

/// An activated ISO/IEC 14443A card.
struct CardA {
    uint8_t uid[10] = {};  ///< 4, 7 or 10 bytes             ISO/IEC 14443-3 6.5.4
    uint8_t uidLen = 0;    ///< valid bytes in uid
    uint16_t atqa = 0;     ///< first received byte = LSB    ISO/IEC 14443-3 6.5.2
    uint8_t sak = 0;       ///< SAK of the last cascade level ISO/IEC 14443-3 6.5.3.4
};

/// Card-side ISO/IEC 14443-3 constants (internal).
namespace iso14443a {
// Card commands                                   ISO/IEC 14443-3 6.4, 6.5
constexpr uint8_t REQA = 0x26;          // short frame, 7 bits
constexpr uint8_t WUPA = 0x52;          // short frame, 7 bits
constexpr uint8_t SEL_CL1 = 0x93;
constexpr uint8_t SEL_CL2 = 0x95;
constexpr uint8_t SEL_CL3 = 0x97;
constexpr uint8_t NVB_ANTICOLLISION = 0x20;  // 2 bytes sent, no UID bits
constexpr uint8_t NVB_SELECT = 0x70;         // 7 bytes sent (full UID CLn + BCC)
constexpr uint8_t HLTA_0 = 0x50;
constexpr uint8_t HLTA_1 = 0x00;
constexpr uint8_t CASCADE_TAG = 0x88;        // CT: UID continues at next level
constexpr uint8_t SAK_CASCADE_BIT = 0x04;    // SAK b3: UID not complete
constexpr uint8_t kShortFrameBits = 7;       // REQA/WUPA are 7-bit short frames
constexpr size_t kMaxCascadeLevels = 3;
// Anticollision bit counts per cascade level            ISO/IEC 14443-3 6.5.3
constexpr uint8_t kClBytes = 5;              // UID CLn (4 bytes) + BCC
constexpr uint8_t kClBits = 40;
constexpr uint8_t kUidClBits = 32;           // UID CLn without BCC
// Bound on collision rounds per cascade level: each resolves >= 1 more of
// the 32 UID CLn bits.
constexpr uint8_t kMaxCollisionRounds = 32;
// NVB: high nibble = whole bytes sent incl. SEL and NVB, low nibble = extra
// bits.                                                 ISO/IEC 14443-3 6.5.3.2
constexpr uint8_t nvb(uint8_t knownUidBits) {
    return static_cast<uint8_t>(((2u + knownUidBits / 8u) << 4) | (knownUidBits % 8u));
}
}  // namespace iso14443a

/// Bounded retry for activation (Iso14443a::activateWithRetry, enumerate).
struct ActivateRetry {
    /// Total activations tried, including the first (clamped to 1..kMaxAttempts).
    uint8_t attempts = 3;
    /// Before each retry, switch the field off and on again (fieldOff(),
    /// kFieldResetOffMs, fieldOn() with the guard of the last fieldOn()).
    /// This also wakes HALTed cards: a field reset puts every PICC into
    /// POWER-OFF and then IDLE.  ISO/IEC 14443-3 6.3
    bool resetField = false;
    /// Upper bound of `attempts`.
    static constexpr uint8_t kMaxAttempts = 10;
};

/// ISO/IEC 14443-3 Type A reader: field control, REQA/WUPA, anticollision,
/// SELECT, HLTA and enumeration of all cards in the field.
///
/// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
class Iso14443a {
public:
    /// @param nfc driver; must outlive this object
    explicit Iso14443a(Pn5180& nfc) : nfc_(nfc) {}

    /// Host-side receive timeout per exchange. Type A answers come within a
    /// few hundred microseconds (ISO/IEC 14443-3 6.2.1.1 frame delay time);
    /// the default leaves margin for millisecond clock granularity.
    void setResponseTimeoutMs(uint32_t ms) { timeoutMs_ = ms; }

    /// LOAD_RF_CONFIG for ISO 14443-A 106 kbit/s (DS Table 32), RF_ON and
    /// wait for the field (DS Table 37, Table 103), set the receiver to
    /// RX_GAIN 2 / SIGPRO_RM_CONFIG 0x450DC = DS default with MIN_LEVEL 5,
    /// measured, see Spec.h (DS Tables 102, 108, 172; must be
    /// after RF_ON because of ARC, DS 11.8.6), then wait @p guardMs with
    /// unmodulated field so the card can power up before the first command
    /// (ISO/IEC 14443-3 5: >= 5 ms).
    Status fieldOn(uint32_t guardMs = 5);

    /// RF_OFF (DS Table 38).
    Status fieldOff();

    /// REQA (or WUPA when @p wakeup), returns ATQA.  ISO/IEC 14443-3 6.5.2
    /// Several cards answer at once, so a bit collision inside the 16-bit
    /// ATQA is accepted (Ok; bits from the collision on are not meaningful);
    /// a collision flag on a frame whose length does not match RX_COLL_POS
    /// is noise (IntegrityError).
    Status request(uint16_t& atqa, bool wakeup = false);

    /// Anticollision + SELECT over cascade levels 1..3 (ISO/IEC 14443-3 6.5.3).
    /// Bit collisions between several cards in READY are resolved: the known
    /// bits up to the collision position (RX_STATUS.RX_COLL_POS, DS Table 93)
    /// are kept and the collided bit is set to 1 (policy: the card with a 1
    /// there wins), then ANTICOLLISION is repeated as a bit-oriented frame
    /// (NVB, partial last TX byte, RX_BIT_ALIGN). Exactly one card ends up
    /// ACTIVE; the others drop back to IDLE (or HALT) at its SELECT.
    /// A collision flag is only trusted on a frame of the expected length
    /// with RX_COLL_POS inside the UID bits; anything else is a garbled frame
    /// (IntegrityError), as are parity/framing errors without a collision.
    /// At most 32 collisions per cascade level.
    /// Does not change card.atqa.
    Status select(CardA& card);

    /// SELECT a known card directly with its full UID (4, 7 or 10 bytes),
    /// cascade level by level, without anticollision (ISO/IEC 14443-3 6.5.3.3,
    /// 6.5.4). Precondition: the card is in READY (request() just answered).
    /// Fills card.uid/uidLen/sak; card.atqa is not changed. InvalidArgument
    /// for another length, CardError if the SAK cascade bit disagrees with the
    /// UID length, NoResponse if no card with that UID answered.
    Status selectUid(const uint8_t* uid, size_t uidLen, CardA& card);

    /// request() + select(). NoResponse means no card in the field.
    Status activate(CardA& card, bool wakeup = false);

    /// request() + selectUid(): re-activate a known card, e.g. after HLTA
    /// (wakeup = true sends WUPA, which also wakes HALTed cards, ISO/IEC 14443-3 6.4).
    Status activateUid(const uint8_t* uid, size_t uidLen, CardA& card, bool wakeup = true);

    /// Find every Type A card in the field, at most @p max.
    ///
    /// REQA, anticollision + SELECT one card, HLTA it, repeat with REQA.
    /// With @p wakeup the field is reset first (fieldOff, kFieldResetOffMs,
    /// fieldOn with the last guard time) so HALTed cards are found too;
    /// WUPA cannot be used for that, because a card woken from HALT goes
    /// back to HALT when another card wins the SELECT (ISO/IEC 14443-3 6.3)
    /// and would be missed by the following REQA. HALTed cards do not
    /// answer REQA (ISO/IEC 14443-3 6.4), cards that lost the SELECT went
    /// back to IDLE and answer again; ends when no card answers.
    ///
    /// cards[0..count) hold the cards in the order found, each HALTed; card.atqa
    /// is the ATQA of the REQA that started its activation: when several cards
    /// answered with different ATQAs it is their superposition (collided bits).
    /// Afterwards all of them stay HALTed: only WUPA (e.g. activateUid) or a
    /// field reset (fieldOff/fieldOn, enumerate(.., wakeup = true)) wakes them.
    /// A transient error (as for activateWithRetry) is retried with a new
    /// request, at most policy.attempts - 1 times per call (policy.resetField is
    /// ignored: a field reset would wake the cards already found). A card
    /// found twice (its HLTA was lost) is halted again and listed once.
    /// @return Ok (count may be 0), BufferTooSmall if one more card answered
    /// after @p max were found (cards[0..max) valid, the extra card is left
    /// SELECTed / ACTIVE), the last transient error once the retries are used
    /// up (cards[0..count) still valid), or a device error at once.
    Status enumerate(CardA* cards, size_t max, size_t& count, bool wakeup = false,
                     ActivateRetry policy = {});

    /// activate() with a bounded retry after a transient error: no answer,
    /// or a garbled ATQA / anticollision / SELECT answer (NoResponse,
    /// Collision, IntegrityError, ProtocolError, CardError, BufferTooSmall).
    /// Collision stays in this set: select() resolves real bit collisions, so
    /// a Collision that still reaches the caller comes from a frame that must
    /// have one sender (SAK) or a collision flag on a garbled frame, i.e. noise.
    /// A PICC that receives an erroneous frame, or misses one, returns from
    /// READY to IDLE (to HALT if it was woken from HALT), so each retry
    /// restarts the whole sequence with the same REQA/WUPA.  ISO/IEC 14443-3 6.3
    /// Any other status (device errors, InvalidArgument) is returned at once.
    /// Returns the status of the last attempt. With no card in the field this
    /// costs `attempts` request timeouts (plus field resets if enabled).
    Status activateWithRetry(CardA& card, bool wakeup = false, ActivateRetry policy = {});

    /// Pause between fieldOff() and fieldOn() for ActivateRetry::resetField:
    /// PCD field reset, field off for at least t_RESET = 5 ms.  ISO/IEC 14443-3 5
    static constexpr uint32_t kFieldResetOffMs = 5;

    /// HLTA. A card must not answer (ISO/IEC 14443-3 6.4.3), so no response
    /// within the timeout is success; any answer is CardError.
    Status halt();

private:
    // Receiver settings written after RF_ON (see Iso14443a.cpp).
    // DS 11.8.6, Tables 102, 108, 172
    Status setupReceiver();
    // ANTICOLLISION loop of one cascade level: cl = UID CLn + BCC (checked).
    Status anticollision(uint8_t sel, uint8_t cl[iso14443a::kClBytes]);
    Status anticollisionLoop(uint8_t sel, uint8_t cl[iso14443a::kClBytes], bool& aligned);
    // SELECT (NVB 70h, CRC_A) with UID CLn + BCC; returns the SAK.
    Status selectCl(uint8_t sel, const uint8_t cl[iso14443a::kClBytes], uint8_t& sak);

    Pn5180& nfc_;
    uint32_t timeoutMs_ = 5;
    uint32_t guardMs_ = 5;  // guard of the last fieldOn(), reused by field resets
};

}  // namespace pn5180
