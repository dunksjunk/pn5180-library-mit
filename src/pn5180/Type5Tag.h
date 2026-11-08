// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NFC Forum Type 5 Tag NDEF mapping on an ISO/IEC 15693 card: Capability
// Container in block 0 and the NDEF TLV area after it, accessed with the
// addressed READ/WRITE SINGLE BLOCK commands of Iso15693. Portable core: no
// Arduino.h, no heap. The ISO 15693 field must be on (Iso15693::fieldOn()).
//
// Source: NFC Forum Type 5 Tag Technical Specification ("T5T", clauses
// named: "Capability Container", "NDEF detection procedure", "NDEF write
// procedure"); block size and count from GET SYSTEM INFORMATION
// (ISO/IEC 15693-3). No vendor-specific commands.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Iso15693.h"
#include "NdefTlv.h"
#include "Status.h"

namespace pn5180 {

namespace t5t {
// Capability Container, first byte.                  T5T (Capability Container)
constexpr uint8_t kCcMagic1 = 0xE1;  // memory addressable with 1-byte block numbers
constexpr uint8_t kCcMagic2 = 0xE2;  // 2-byte block numbers needed beyond 256 blocks
// CC1: version and access conditions.                T5T (Capability Container)
constexpr uint8_t kMajorShift = 6;         // b7..b6 major version
constexpr uint8_t kMinorShift = 4;         // b5..b4 minor version
constexpr uint8_t kReadAccessShift = 2;    // b3..b2 read access
constexpr uint8_t kAccessMask = 0x03;      // b1..b0 write access
constexpr uint8_t kAccessGranted = 0x00;   // 00b: always
constexpr uint8_t kSupportedMajor = 1;     // mapping version 1.x
// CC2: MLEN, T5T area size in 8-byte units; 00h = 8-byte CC with a 2-byte
// MLEN in CC6..CC7 (MSB first).                      T5T (Capability Container)
constexpr size_t kMlenUnitBytes = 8;
constexpr size_t kCcShortBytes = 4;
constexpr size_t kCcLongBytes = 8;
// READ/WRITE SINGLE BLOCK carry a 1-byte block number.  ISO/IEC 15693-3 (read single block)
constexpr size_t kMaxBlocks1Byte = 256;
}  // namespace t5t

// Decoded Capability Container.                      T5T (Capability Container)
struct T5tCapability {
    uint8_t raw[t5t::kCcLongBytes] = {};  // first 8 bytes of the card (CC0..CC7 if 8-byte CC)
    uint8_t ccBytes = 0;                  // 4 or 8
    uint8_t versionMajor = 0;
    uint8_t versionMinor = 0;
    uint8_t readAccess = 0;               // CC1 b3..b2
    uint8_t writeAccess = 0;              // CC1 b1..b0
    uint8_t features = 0;                 // CC3
    uint32_t mlenBytes = 0;               // MLEN * 8
    // TLV area actually used: from ccBytes to min(ccBytes + MLEN * 8, the
    // memory size from GET SYSTEM INFORMATION, limited to 256 blocks).
    uint32_t areaStart = 0;
    uint32_t areaEnd = 0;
    bool readOnly = true;                 // write access != 00b
};

// One object per card (UID) per session. Not thread-safe; shares the
// single-owner rule of Pn5180 (see Pn5180.h).
class Type5Tag {
public:
    // `uid` (MSB first, as CardV::uid) must stay valid while the object is used.
    Type5Tag(Iso15693& reader, const uint8_t uid[iso15693::kUidBytes])
        : reader_(reader), uid_(uid) {}
    Type5Tag(const Type5Tag&) = delete;  // ndefMemory() refers to this object
    Type5Tag& operator=(const Type5Tag&) = delete;

    // GET SYSTEM INFORMATION (once) and the CC from the first blocks.
    //   Ok            `cc` filled and cached
    //   Unsupported   no memory size in the system information, block size
    //                 above 32, mapping major version != 1 or read access
    //                 not granted
    //   NotFormatted  no E1h/E2h magic, or MLEN 0
    //   + Iso15693 command results (NoResponse, CardError, ...)
    Status capability(T5tCapability& cc);

    // System information as read by capability().
    const SystemInfo15693& systemInfo() const { return info_; }

    // As Type2Tag::readNdef() / writeNdef(); the CC block(s) are never written.
    Status readNdef(uint8_t* buf, size_t capacity, size_t& len);
    Status writeNdef(const uint8_t* msg, size_t len);

    // The T5T area as NdefMemory (unit = block). Empty until capability() succeeded.
    NdefMemory& ndefMemory() { return mem_; }

private:
    class Memory final : public NdefMemory {
    public:
        explicit Memory(Type5Tag& tag) : tag_(tag) {}
        size_t unitSize() const override { return tag_.ccValid_ ? tag_.info_.blockSize : 0; }
        size_t areaStart() const override { return tag_.cc_.areaStart; }
        size_t areaEnd() const override { return tag_.ccValid_ ? tag_.cc_.areaEnd : 0; }
        bool readOnly() const override { return tag_.cc_.readOnly; }
        Status readUnit(size_t unit, uint8_t* out) override;
        Status writeUnit(size_t unit, const uint8_t* in) override;

    private:
        Type5Tag& tag_;
    };

    Status ensureCapability();
    size_t blocks() const;  // addressable blocks

    Iso15693& reader_;
    const uint8_t* uid_;
    SystemInfo15693 info_;
    bool infoValid_ = false;
    T5tCapability cc_;
    bool ccValid_ = false;
    Memory mem_{*this};
};

}  // namespace pn5180
