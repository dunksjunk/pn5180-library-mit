// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NFC Forum Type 2 Tag memory access (READ 30h, WRITE A2h, Capability
// Container) on an ISO/IEC 14443-3 Type A card that Iso14443a has already
// activated (field on, card selected). Portable core: no Arduino.h, no heap.
//
// Card-side protocol per the public NFC Forum Type 2 Tag specification
// (cited "T2T") and ISO/IEC 14443-3; PN5180 setup per the NXP data sheet (DS).
// Only the generic T2T commands are used, no vendor-specific ones.
#pragma once

#include <cstddef>
#include <cstdint>

#include "NdefTlv.h"
#include "Pn5180.h"
#include "Status.h"

namespace pn5180 {

namespace t2t {
// Commands                                         T2T (command set)
constexpr uint8_t CMD_READ = 0x30;   // READ: page address -> 16 bytes (4 pages)
constexpr uint8_t CMD_WRITE = 0xA2;  // WRITE: page address + 4 bytes -> ACK/NAK
constexpr size_t kPageBytes = 4;     // T2T (memory structure: 4-byte blocks)
constexpr size_t kReadBytes = 16;    // READ returns 4 pages   T2T (READ)
// ACK/NAK: a 4-bit frame without CRC_A.             T2T (ACK and NAK)
constexpr uint8_t kAckNakBits = 4;
constexpr uint8_t kAck = 0x0A;       // ACK = 1010b; any other value is a NAK
constexpr uint8_t kAckNakMask = 0x0F;

// Static memory layout                               T2T (memory structure)
constexpr uint8_t kCcPage = 3;         // Capability Container in page 3
constexpr uint8_t kFirstDataPage = 4;  // data area starts at page 4
// Capability Container bytes                         T2T (Capability Container)
constexpr uint8_t kCcMagic = 0xE1;            // CC0: NDEF magic number
constexpr size_t kCcSizeUnitBytes = 8;        // CC2: data area size / 8
constexpr uint8_t kCcReadAccessShift = 4;     // CC3 b7..b4: read access
constexpr uint8_t kCcWriteAccessMask = 0x0F;  // CC3 b3..b0: write access
constexpr uint8_t kCcAccessGranted = 0x0;     // 0h: access granted without security
constexpr uint8_t kCcMajorVersionShift = 4;   // CC1 b7..b4: major version
// Highest mapping major version this driver implements (1.x); a reader must
// not access a tag with a higher major version.  T2T (NDEF detection procedure)
constexpr uint8_t kSupportedMajorVersion = 1;
constexpr size_t kDataAreaStartByte = kFirstDataPage * kPageBytes;  // byte 16

// Host receive timeouts (host policy, not from a cited spec clause). READ
// answers within the ISO/IEC 14443-3 frame delay time (well under 1 ms of
// airtime here). WRITE answers after the tag has programmed its memory.
// Measured on the reference test bed (NTAG215, ESP32 at 1 MHz SPI,
// 2026-11-04, 20 writes): the whole Type2Tag::write() call, including the
// SPI traffic around the exchange, took 7.2..12.8 ms, so the ACK came less
// than 12.8 ms after the frame. 20 ms leaves about 1.5x margin over that
// upper bound, including 1 ms for the granularity of the millisecond clock.
constexpr uint32_t kReadTimeoutMs = 5;
constexpr uint32_t kWriteTimeoutMs = 20;
}  // namespace t2t

// Decoded Capability Container (page 3).            T2T (Capability Container)
struct T2tCapability {
    uint8_t raw[4] = {};       // CC0..CC3 as read
    uint8_t version = 0;       // CC1: mapping version (major b7..b4, minor b3..b0)
    uint16_t dataBytes = 0;    // CC2 * 8: size of the data area in bytes
    uint8_t firstPage = t2t::kFirstDataPage;
    uint8_t lastPage = 0;      // last page of the data area (inclusive)
    bool readOnly = true;      // CC3 write access != 0h
    bool readable = false;     // CC3 read access == 0h
};

// One object per activated card: it caches that card's Capability Container.
// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
class Type2Tag {
public:
    explicit Type2Tag(Pn5180& nfc) : nfc_(nfc) {}
    Type2Tag(const Type2Tag&) = delete;  // ndefMemory() refers to this object
    Type2Tag& operator=(const Type2Tag&) = delete;

    // READ 30h: 16 bytes from `page` onward (4 pages), CRC_A on both
    // directions (the PN5180 checks and strips it, DS Tables 92, 99).
    //   Ok, NoResponse, IntegrityError (CRC/parity), CardError (NAK or wrong
    //   length), or a device error.
    Status read(uint8_t page, uint8_t out[t2t::kReadBytes]);

    // Reads and decodes the Capability Container; caches it for write().
    //   NotFormatted if CC0 is not E1h or CC2 is 0 (no NDEF-formatted T2T).
    Status capability(T2tCapability& cc);

    // ---- NDEF (NFC Forum Type 2 Tag NDEF mapping) ----
    // Both read the CC first if needed (capability()).
    //   NotFormatted   no valid CC
    //   Unsupported    CC major version > 1 or read access not granted
    //   + readNdefMessage() / writeNdefMessage() results (NdefTlv.h);
    //   writeNdef: ReadOnly when CC3 refuses writes. Pages 0-3 are never written.
    // readNdef: `len` = message length, 0 if the tag holds no NDEF message.
    Status readNdef(uint8_t* buf, size_t capacity, size_t& len);
    Status writeNdef(const uint8_t* msg, size_t len);

    // The data area (pages 4 .. lastPage) as NdefMemory, for planNdefWrite()
    // / backupNdefArea() / restoreNdefArea(). Empty (unitSize() 0) until
    // capability() succeeded. Reads go through a 4-page cache that is
    // dropped by every write and by readNdef()/writeNdef().
    NdefMemory& ndefMemory() { return mem_; }

    // WRITE A2h: 4 bytes into `page`. Allowed only inside the data area that
    // the CC declares (reads the CC first if needed):
    //   InvalidArgument  page outside the data area (pages 0..3 never written)
    //   ReadOnly         the CC marks the tag read-only
    // The ACK/NAK is a 4-bit frame without CRC, so CRC checking on receive is
    // switched off for this exchange and CRC_RX_CONFIG is restored to its
    // previous value on every path (DS Table 92). NAK -> CardError.
    Status write(uint8_t page, const uint8_t in[t2t::kPageBytes]);

    // ACK/NAK nibble of the last WRITE answer (0xFF if none).
    uint8_t lastAckNak() const { return lastAckNak_; }

private:
    class Memory final : public NdefMemory {
    public:
        explicit Memory(Type2Tag& tag) : tag_(tag) {}
        size_t unitSize() const override { return tag_.ccValid_ ? t2t::kPageBytes : 0; }
        size_t areaStart() const override { return t2t::kDataAreaStartByte; }
        size_t areaEnd() const override {
            return tag_.ccValid_ ? t2t::kDataAreaStartByte + tag_.cc_.dataBytes : 0;
        }
        bool readOnly() const override { return tag_.cc_.readOnly; }
        Status readUnit(size_t unit, uint8_t* out) override;
        Status writeUnit(size_t unit, const uint8_t* in) override;
        void invalidate() { cached_ = false; }

    private:
        Type2Tag& tag_;
        bool cached_ = false;
        uint8_t cachePage_ = 0;
        uint8_t cache_[t2t::kReadBytes] = {};
    };

    Status writeExchange(const uint8_t* frame, size_t len);
    // capability() if needed, then the NDEF access checks (version, read access).
    Status ndefReady();

    Pn5180& nfc_;
    T2tCapability cc_;
    bool ccValid_ = false;
    uint8_t lastAckNak_ = 0xFF;
    Memory mem_{*this};
};

}  // namespace pn5180
