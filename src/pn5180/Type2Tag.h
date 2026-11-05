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

// Host receive timeouts. READ answers within the 14443-3 frame delay time
// (well under 1 ms of airtime here); WRITE answers after the tag has
// programmed its memory. The T2T specification bounds the WRITE response
// time in the low-millisecond range; 10 ms is a conservative host policy
// above it (see the questions in the M4 notes) and includes 1 ms for the
// granularity of the millisecond clock.
constexpr uint32_t kReadTimeoutMs = 5;
constexpr uint32_t kWriteTimeoutMs = 10;
}  // namespace t2t

// Decoded Capability Container (page 3).            T2T (Capability Container)
struct T2tCapability {
    uint8_t raw[4] = {};       // CC0..CC3 as read
    uint8_t version = 0;       // CC1: mapping version (major b7..b4, minor b3..b0)
    uint16_t dataBytes = 0;    // CC2 * 8: size of the data area in bytes
    uint8_t firstPage = t2t::kFirstDataPage;
    uint8_t lastPage = 0;      // last page of the data area (inclusive)
    bool readOnly = true;      // CC3 write access != 0h
};

// One object per activated card: it caches that card's Capability Container.
// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
class Type2Tag {
public:
    explicit Type2Tag(Pn5180& nfc) : nfc_(nfc) {}

    // READ 30h: 16 bytes from `page` onward (4 pages), CRC_A on both
    // directions (the PN5180 checks and strips it, DS Tables 92, 99).
    //   Ok, NoResponse, IntegrityError (CRC/parity), CardError (NAK or wrong
    //   length), or a device error.
    Status read(uint8_t page, uint8_t out[t2t::kReadBytes]);

    // Reads and decodes the Capability Container; caches it for write().
    //   CardError if CC0 is not E1h or CC2 is 0 (no NDEF-formatted T2T).
    Status capability(T2tCapability& cc);

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
    Status writeExchange(const uint8_t* frame, size_t len);

    Pn5180& nfc_;
    T2tCapability cc_;
    bool ccValid_ = false;
    uint8_t lastAckNak_ = 0xFF;
};

}  // namespace pn5180
