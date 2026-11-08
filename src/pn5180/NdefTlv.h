// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// TLV layer shared by the NFC Forum Type 2 Tag and Type 5 Tag mappings: find
// the NDEF Message TLV in a tag's data area, read it, and write a message as
// NDEF Message TLV + Terminator TLV, plus a backup/restore helper for the
// units a write touches. Portable core: no Arduino.h, no heap.
//
// Sources: NFC Forum Type 2 Tag Technical Specification ("T2T", clauses
// named: "TLV blocks", "NDEF write procedure") and NFC Forum Type 5 Tag
// Technical Specification ("T5T", same TLV format after the CC). Both use:
//   T (1 byte), L (1 byte 00h..FEh, or FFh + 2 bytes MSB first for
//   00FFh..FFFEh), V (L bytes). NULL and Terminator TLVs have no L/V.
//
// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
#pragma once

#include <cstddef>
#include <cstdint>

#include "Status.h"

namespace pn5180 {

namespace tlv {
// TLV block types.                                   T2T (TLV blocks), T5T (TLV blocks)
constexpr uint8_t T_NULL = 0x00;            // single byte, no L/V
constexpr uint8_t T_LOCK_CONTROL = 0x01;    // T2T only: dynamic lock bits position
constexpr uint8_t T_MEMORY_CONTROL = 0x02;  // T2T only: reserved memory position
constexpr uint8_t T_NDEF = 0x03;            // NDEF Message TLV
constexpr uint8_t T_PROPRIETARY = 0xFD;
constexpr uint8_t T_TERMINATOR = 0xFE;      // single byte, last TLV
// Length field.                                       T2T (TLV blocks: length field)
constexpr uint8_t kLength3Marker = 0xFF;    // FFh + 2 bytes = 3-byte format
constexpr size_t kMaxShortLength = 0xFE;    // 1-byte format: 00h..FEh
constexpr size_t kMaxLength = 0xFFFE;       // 3-byte format: 00FFh..FFFEh
// Lock / Memory Control TLV value (3 bytes).         T2T (Lock Control TLV, Memory Control TLV)
//   V0: b7..b4 page address, b3..b0 byte offset in that page
//   V1: size (Lock: number of lock bits, Memory: reserved bytes; 0 = 256)
//   V2: b3..b0 log2 of the page size in bytes (b7..b4: bytes locked per lock bit)
//   Byte address = page address * 2^(V2 b3..b0) + byte offset.
constexpr size_t kControlValueBytes = 3;
// Host limits (not from a spec).
constexpr size_t kMaxUnitBytes = 32;  // biggest ISO/IEC 15693 block
constexpr size_t kMaxReserved = 4;    // Lock/Memory Control TLVs remembered
}  // namespace tlv

// The data area of one card as an array of equal read/write units (T2T: a
// 4-byte page, ISO/IEC 15693: a block). Addresses are absolute byte
// addresses in the card memory: unit u covers [u * unitSize(), (u+1) * unitSize()).
class NdefMemory {
public:
    virtual size_t unitSize() const = 0;   // 1..tlv::kMaxUnitBytes, 0 = no valid CC
    virtual size_t areaStart() const = 0;  // first byte of the TLV area (after the CC)
    virtual size_t areaEnd() const = 0;    // one past the last byte of the TLV area
    virtual bool readOnly() const = 0;     // the CC refuses writes
    virtual Status readUnit(size_t unit, uint8_t* out) = 0;
    virtual Status writeUnit(size_t unit, const uint8_t* in) = 0;

protected:
    ~NdefMemory() = default;
};

// Result of scanning the TLV area.
struct NdefTlvScan {
    struct Range {
        size_t start = 0;  // absolute byte address
        size_t end = 0;    // exclusive
    };
    bool found = false;     // NDEF Message TLV present
    size_t tlvStart = 0;    // its T byte; if !found: where a new one would go
    size_t valueStart = 0;  // first byte of the message (found only)
    size_t length = 0;      // message length (found only)
    // Areas declared by Lock/Memory Control TLVs before the NDEF TLV; NDEF
    // data must not be stored there.        T2T (Lock Control TLV, Memory Control TLV)
    Range reserved[tlv::kMaxReserved];
    size_t reservedCount = 0;
};

// Walks the TLVs from areaStart(): NULL TLVs and Lock/Memory Control and
// proprietary TLVs are skipped, the scan stops at the first NDEF Message TLV
// or at a Terminator TLV / the end of the area.
//   Ok             `scan` filled (found or not)
//   NotFormatted   the memory has no valid data area (unitSize() 0 or area empty)
//   MalformedData  a TLV length runs past the area, or a control TLV has a
//                  value length other than 3
//   Unsupported    more than tlv::kMaxReserved control TLVs
//   other          read error of the card
Status scanNdefTlv(NdefMemory& mem, NdefTlvScan& scan);

// Copies the NDEF message (V of the first NDEF Message TLV) into `buf`.
// `len` = message length; 0 if there is no NDEF TLV or it is empty.
//   BufferTooSmall `len` is set to the needed size, nothing copied
//   Unsupported    the message overlaps a reserved (lock/memory control) area
//   + the results of scanNdefTlv()
Status readNdefMessage(NdefMemory& mem, uint8_t* buf, size_t capacity, size_t& len);

// Where a message of `msgLength` bytes would be written: at the existing NDEF
// Message TLV, else after the last TLV before the Terminator. A Terminator
// TLV follows the message if a byte is left in the area.
struct NdefWritePlan {
    size_t tlvStart = 0;      // T byte of the NDEF Message TLV
    size_t headerBytes = 0;   // T + L: 2 (L <= FEh) or 4 (3-byte L)
    size_t msgLength = 0;
    bool terminator = false;  // a Terminator TLV is written after the message
    size_t firstUnit = 0;     // first and last unit written (inclusive)
    size_t lastUnit = 0;
    size_t end() const { return tlvStart + headerBytes + msgLength + (terminator ? 1u : 0u); }
};
//   ReadOnly       the CC marks the area read-only
//   BufferTooSmall the TLV does not fit into the area (or msgLength > FFFEh)
//   Unsupported    the TLV would overlap a reserved area, or would have to
//                  rewrite a unit that also holds the CC
//   + the results of scanNdefTlv()
Status planNdefWrite(NdefMemory& mem, size_t msgLength, NdefWritePlan& plan);

// Writes `msg` (may be empty: an empty NDEF TLV) per planNdefWrite(). Order,
// so that an interrupted write leaves a valid TLV area:
//   1. the unit(s) holding T and L, with L = 00h (an empty NDEF Message TLV),
//   2. the remaining units (message, Terminator),
//   3. the unit(s) holding T and L again, now with the real length.
// T2T (NDEF write procedure), T5T (NDEF write procedure). Units only partly
// covered are read first and their other bytes written back unchanged.
// The message content itself is not checked (see validateNdefMessage()).
Status writeNdefMessage(NdefMemory& mem, const uint8_t* msg, size_t len);

// Copy of the units a write per `plan` touches.
struct NdefBackup {
    static constexpr size_t kMaxBytes = 512;  // host limit
    bool valid = false;
    size_t firstUnit = 0;
    size_t units = 0;
    size_t unitSize = 0;
    size_t tlvStart = 0;
    uint8_t data[kMaxBytes] = {};
};
//   BufferTooSmall the units do not fit into NdefBackup::kMaxBytes
//   other          read error (backup.valid stays false)
Status backupNdefArea(NdefMemory& mem, const NdefWritePlan& plan, NdefBackup& backup);

// Writes the backed-up units back (first the T/L unit(s) as an empty NDEF
// Message TLV, then the other units, then the T/L unit(s) from the backup,
// mirroring writeNdefMessage()) and reads every unit back.
//   CardError      a unit read back differs from the backup
//   InvalidArgument backup not valid / from another memory geometry
//   other          write or read error
Status restoreNdefArea(NdefMemory& mem, const NdefBackup& backup);

}  // namespace pn5180
