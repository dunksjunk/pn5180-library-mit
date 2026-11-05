// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 15693 (vicinity cards) INVENTORY and addressed memory commands
// (GET SYSTEM INFORMATION, READ/WRITE SINGLE BLOCK) on top of
// Pn5180::transceive().
// Portable core: no Arduino.h, no heap.
//
// Card-side protocol per the public standard ISO/IEC 15693-2/-3 (clauses are
// named rather than numbered because the numbering differs between
// editions); PN5180 setup per the NXP data sheet (DS) and AN12650 4.2.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Pn5180.h"
#include "Spec.h"
#include "Status.h"

namespace pn5180 {

struct CardV {
    // UID, most significant byte first: uid[0] is the E0h prefix, uid[1] the
    // IC manufacturer code. On air the UID is sent LSB first.  ISO/IEC 15693-3 (UID)
    uint8_t uid[8] = {};
    uint8_t dsfid = 0;  // data storage format identifier    ISO/IEC 15693-3 (DSFID)
};

namespace iso15693 {
// Request flags (b1 = LSB).                       ISO/IEC 15693-3 (request flags)
constexpr uint8_t FLAG_DATA_RATE_HIGH = 0x02;  // b2: high data rate
constexpr uint8_t FLAG_INVENTORY = 0x04;       // b3: inventory flag
constexpr uint8_t FLAG_NB_SLOTS_ONE = 0x20;    // b6 (inventory set): 1 slot, clear = 16
constexpr uint8_t FLAG_ADDRESS = 0x20;         // b6 (inventory clear): addressed, UID follows
constexpr uint8_t FLAG_OPTION = 0x40;          // b7: command-specific option
// Response flags                                  ISO/IEC 15693-3 (response flags)
constexpr uint8_t RESP_FLAG_ERROR = 0x01;  // b1: error, an error code follows
// Command codes                                   ISO/IEC 15693-3 (command codes)
constexpr uint8_t CMD_INVENTORY = 0x01;
constexpr uint8_t CMD_READ_SINGLE_BLOCK = 0x20;
constexpr uint8_t CMD_WRITE_SINGLE_BLOCK = 0x21;
constexpr uint8_t CMD_GET_SYSTEM_INFO = 0x2B;
// Addressed request at high data rate: flags 22h.  ISO/IEC 15693-3 (request flags)
constexpr uint8_t kRequestFlagsAddressed = FLAG_DATA_RATE_HIGH | FLAG_ADDRESS;
// GET SYSTEM INFORMATION info flags.               ISO/IEC 15693-3 (get system information)
constexpr uint8_t INFO_DSFID = 0x01;
constexpr uint8_t INFO_AFI = 0x02;
constexpr uint8_t INFO_MEMORY_SIZE = 0x04;  // 2 bytes: blocks - 1, block size - 1 (b5..b1)
constexpr uint8_t INFO_IC_REFERENCE = 0x08;
constexpr uint8_t kBlockSizeMask = 0x1F;    // block size in bytes - 1, 5 bits
// Block security status (READ with option flag): b1 = block locked.
//                                                  ISO/IEC 15693-3 (block security status)
constexpr uint8_t SECURITY_LOCKED = 0x01;
constexpr size_t kMaxBlockBytes = 32;       // 5-bit block size field -> 1..32 bytes
constexpr uint8_t kMaskLengthNone = 0x00;  // mask length 0: every card takes part

constexpr uint8_t kRequestFlags1Slot = FLAG_DATA_RATE_HIGH | FLAG_INVENTORY | FLAG_NB_SLOTS_ONE;  // 26h
constexpr uint8_t kRequestFlags16Slots = FLAG_DATA_RATE_HIGH | FLAG_INVENTORY;  // 06h  AN12650 4.2.3 line 6

constexpr size_t kUidBytes = 8;
constexpr size_t kSlots = 16;  // 16-slot inventory    ISO/IEC 15693-3 (anticollision)
// INVENTORY response without CRC: flags, DSFID, UID.  ISO/IEC 15693-3 (inventory)
constexpr size_t kInventoryResponseBytes = 1 + 1 + kUidBytes;
constexpr size_t kCrcBytes = 2;  // CRC-16 per ISO/IEC 13239   ISO/IEC 15693-3 (CRC)
constexpr uint8_t kUidMsb = 0xE0;  // UID MSB is always E0h  ISO/IEC 15693-3 (UID)

// ---- Timing (fc = 13.56 MHz) ----
constexpr uint64_t kFcHz = 13560000u;
// t1: VICC response delay, nominal 4352/fc (~321 us).   ISO/IEC 15693-3 (t1)
constexpr uint32_t kT1Ns = static_cast<uint32_t>(4352u * 1000000000ull / kFcHz);
// t2: minimum time from a VICC response to the VCD's next EOF, 4192/fc
// (~309 us).                                            ISO/IEC 15693-3 (t2)
constexpr uint32_t kT2Ns = static_cast<uint32_t>(4192u * 1000000000ull / kFcHz);
// VICC SOF and EOF each last 4 bit durations.       ISO/IEC 15693-2 (VICC SOF/EOF)
constexpr uint32_t kDelimiterBits = 2u * 4u;
// Longest request on air (flags, command, mask length, CRC) with SOF/EOF.
// Reader->card 1-out-of-4 runs at the same 26.48 kbit/s (DS Table 64:
// 302.08 us per byte = 37.76 us per bit) as card->reader (DS Table 65).
constexpr uint32_t kRequestBits = static_cast<uint32_t>((3u + kCrcBytes) * 8u) + kDelimiterBits;
constexpr uint32_t kResponseBits =
    static_cast<uint32_t>((kInventoryResponseBytes + kCrcBytes) * 8u) + kDelimiterBits;
// SEND_DATA returns when transmission starts (DS Table 15 note), so one slot
// needs request + t1 + full response before RX_IRQ.  ~6.1 ms.
constexpr uint32_t kSlotNs = (kRequestBits + kResponseBits) * spec::kIso15693BitNs + kT1Ns;
// Host receive timeout per slot: rounded up to whole ms, +1 ms for the
// granularity of the millisecond clock.  -> 8 ms
constexpr uint32_t kSlotTimeoutMs = (kSlotNs + 999999u) / 1000000u + 1u;
// Pause after a slot in which a card answered, before the next EOF; t2
// rounded up to the host's millisecond resolution.
constexpr uint32_t kT2Ms = (kT2Ns + 999999u) / 1000000u;
// VICC power-up: a card must be ready for a request within 1 ms of the field
// being switched on.                                ISO/IEC 15693-3 (power-up)
constexpr uint32_t kPowerUpMs = 1;

// Write alternative with option flag 0: the VICC answers once programming
// is done, at the latest 20 ms after the request's EOF.
//                                                  ISO/IEC 15693-3 (write alternatives)
constexpr uint32_t kWriteMaxNs = 20000000u;

// Host receive timeout for a request of `reqBytes` and a response of up to
// `respBytes` (both without CRC): request + t1 + `extraNs` + response on air,
// rounded up to whole ms, +1 ms for the millisecond clock granularity. Same
// model as kSlotNs above.
constexpr uint32_t exchangeTimeoutMs(size_t reqBytes, size_t respBytes, uint32_t extraNs = 0) {
    return static_cast<uint32_t>(
               ((((reqBytes + kCrcBytes) * 8u + kDelimiterBits) +
                 ((respBytes + kCrcBytes) * 8u + kDelimiterBits)) *
                    spec::kIso15693BitNs +
                kT1Ns + extraNs + 999999u) /
               1000000u) +
           1u;
}
// Requests: flags, command, UID [, block [, data]].
constexpr size_t kAddressedHeaderBytes = 2 + kUidBytes;
// Responses: flags, info flags, UID, DSFID, AFI, memory size (2), IC ref.
constexpr size_t kSystemInfoMaxBytes = 2 + kUidBytes + 1 + 1 + 2 + 1;
constexpr uint32_t kSystemInfoTimeoutMs =
    exchangeTimeoutMs(kAddressedHeaderBytes, kSystemInfoMaxBytes);
constexpr uint32_t kReadBlockTimeoutMs =
    exchangeTimeoutMs(kAddressedHeaderBytes + 1, 2 + kMaxBlockBytes);
constexpr uint32_t kWriteBlockTimeoutMs =
    exchangeTimeoutMs(kAddressedHeaderBytes + 1 + kMaxBlockBytes, 2, kWriteMaxNs);
}  // namespace iso15693

// GET SYSTEM INFORMATION answer.                  ISO/IEC 15693-3 (get system information)
struct SystemInfo15693 {
    uint8_t infoFlags = 0;    // iso15693::INFO_*: which fields below are valid
    uint8_t uid[8] = {};      // MSB first, as CardV
    uint8_t dsfid = 0;
    uint8_t afi = 0;
    uint16_t blockCount = 0;  // number of blocks (0 if INFO_MEMORY_SIZE absent)
    uint8_t blockSize = 0;    // bytes per block (0 if INFO_MEMORY_SIZE absent)
    uint8_t icReference = 0;
};

struct InventoryResult {
    CardV cards[iso15693::kSlots];
    uint8_t count = 0;            // valid entries in cards[]
    uint16_t collisionSlots = 0;  // bit n: slot n had an unusable (garbled) answer
};

// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
class Iso15693 {
public:
    explicit Iso15693(Pn5180& nfc) : nfc_(nfc) {}

    // Host-side receive timeout per slot (default iso15693::kSlotTimeoutMs).
    void setResponseTimeoutMs(uint32_t ms) { timeoutMs_ = ms; }

    // LOAD_RF_CONFIG 0x0D/0x8D (ISO 15693 ASK100, 26 kbit/s; DS Table 32,
    // AN12650 4.2.3 line 1), RF_ON and wait for the field (DS Table 37,
    // Table 103), CRC on in both directions (DS Tables 92, 99), then wait
    // `guardMs` of unmodulated field so cards can power up.
    Status fieldOn(uint32_t guardMs = 5);

    // RF_OFF (DS Table 38; AN12650 4.2.1 line 16).
    Status fieldOff();

    // Single-slot INVENTORY (flags 26h, mask length 0).
    //   Ok             card filled in
    //   NoResponse     no card answered
    //   Collision      several cards answered at once (PN5180 RX_STATUS)
    //   IntegrityError CRC error / ProtocolError framing error (RX_STATUS);
    //                  with several cards in the field this is a collision too
    //   CardError      error flag set or wrong length / UID prefix
    Status inventory(CardV& card);

    // 16-slot INVENTORY (flags 06h, mask length 0) as in AN12650 4.2: the
    // request opens slot 0, an EOF-only frame (TX_CONFIG with TX_DATA_ENABLE
    // and the start symbol cleared, DS Table 98) opens each of slots 1..15.
    // TX_CONFIG is restored on every path.
    //   Ok         at least one card read (result.collisionSlots may be != 0)
    //   Collision  no card read cleanly, but some slots had garbled answers
    //   NoResponse no answer in any slot
    //   other      device error; the inventory was aborted
    Status inventory16(InventoryResult& result);

    // ---- Addressed memory commands (flags 22h + UID, LSB first on air) ----
    // `uid` is MSB first (CardV::uid). For all of them:
    //   CardError  the card set the error flag (lastErrorCode() holds the
    //              ISO/IEC 15693-3 error code) or answered malformed
    //   NoResponse no answer (wrong UID, card gone)
    // The field must be on (fieldOn()); each command uses its own timeout.

    // GET SYSTEM INFORMATION 2Bh. The UID in the answer must match `uid`.
    Status getSystemInfo(const uint8_t uid[iso15693::kUidBytes], SystemInfo15693& info);

    // READ SINGLE BLOCK 20h. `blockSize` = bytes per block (1..32, e.g. from
    // getSystemInfo); the answer must be exactly that long. With
    // `withSecurity` the option flag is set and the block security status
    // byte is stored in `security` (required then, may be nullptr otherwise).
    Status readBlock(const uint8_t uid[iso15693::kUidBytes], uint8_t block, uint8_t* out,
                     size_t blockSize, bool withSecurity = false, uint8_t* security = nullptr);

    // WRITE SINGLE BLOCK 21h, option flag 0: the card answers after
    // programming (timeout iso15693::kWriteBlockTimeoutMs).
    Status writeBlock(const uint8_t uid[iso15693::kUidBytes], uint8_t block, const uint8_t* data,
                      size_t blockSize);

    // ISO/IEC 15693-3 error code of the last addressed command that came
    // back with the error flag; 0 after a command without a card error.
    uint8_t lastErrorCode() const { return lastErrorCode_; }

private:
    // Sends `req` and checks the response flags. On success `rx`/`n` hold
    // the response without CRC (flags byte included).
    Status addressed(const uint8_t* req, size_t reqLen, uint8_t* rx, size_t rxCap, size_t& n,
                     uint32_t timeoutMs);
    // Writes flags, command and the UID (LSB first); returns the length.
    static size_t putHeader(uint8_t* req, uint8_t flags, uint8_t cmd, const uint8_t* uid);

    // One exchange; maps the answer to `card` (see inventory()).
    Status exchange(const uint8_t* tx, size_t txLen, CardV& card);
    Status runSlots(InventoryResult& result);

    Pn5180& nfc_;
    uint32_t timeoutMs_ = iso15693::kSlotTimeoutMs;
    uint8_t lastErrorCode_ = 0;
};

}  // namespace pn5180
