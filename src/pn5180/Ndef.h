// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NDEF message parser and builder, plus the NFC Forum Well-known URI ("U")
// and Text ("T") record helpers. Portable core: no Arduino.h, no heap, no
// card access; works on caller-provided buffers only.
//
// Sources (public NFC Forum specifications, cited by clause):
//   "NDEF"     NFC Data Exchange Format (NDEF) Technical Specification 1.0
//   "RTD"      NFC Record Type Definition (RTD) 1.0 (well-known type names)
//   "RTD URI"  URI Record Type Definition 1.0
//   "RTD Text" Text Record Type Definition 1.0
//
// Pure functions on caller buffers; objects are not shared between threads.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Status.h"

namespace pn5180 {

namespace ndef {
// Record header flags (first octet).                 NDEF 3.2.1 .. 3.2.6
constexpr uint8_t FLAG_MB = 0x80;   // Message Begin           NDEF 3.2.1
constexpr uint8_t FLAG_ME = 0x40;   // Message End             NDEF 3.2.2
constexpr uint8_t FLAG_CF = 0x20;   // Chunk Flag              NDEF 3.2.3
constexpr uint8_t FLAG_SR = 0x10;   // Short Record            NDEF 3.2.4
constexpr uint8_t FLAG_IL = 0x08;   // ID_LENGTH present       NDEF 3.2.5
constexpr uint8_t TNF_MASK = 0x07;  // Type Name Format        NDEF 3.2.6

// TNF values.                                        NDEF 3.2.6
constexpr uint8_t TNF_EMPTY = 0x00;
constexpr uint8_t TNF_WELL_KNOWN = 0x01;   // NFC Forum RTD type
constexpr uint8_t TNF_MEDIA = 0x02;        // RFC 2046 media type
constexpr uint8_t TNF_ABSOLUTE_URI = 0x03; // RFC 3986 absolute URI
constexpr uint8_t TNF_EXTERNAL = 0x04;     // NFC Forum external type
constexpr uint8_t TNF_UNKNOWN = 0x05;
constexpr uint8_t TNF_UNCHANGED = 0x06;    // only in middle/terminating chunks
constexpr uint8_t TNF_RESERVED = 0x07;

// Short record: PAYLOAD_LENGTH is 1 octet (0..255), else 4 octets.  NDEF 3.2.4, 3.2.9
constexpr size_t kShortPayloadMax = 255;
// TYPE_LENGTH and ID_LENGTH are single octets.        NDEF 3.2.7, 3.2.8
constexpr size_t kMaxTypeLength = 255;
constexpr size_t kMaxIdLength = 255;

// Well-known types (single-character record type names).   RTD URI 3.1, RTD Text 3.1
constexpr uint8_t RTD_URI = 'U';
constexpr uint8_t RTD_TEXT = 'T';

// Text record status byte.                            RTD Text 3.2.1
constexpr uint8_t TEXT_UTF16 = 0x80;      // b7: 0 = UTF-8, 1 = UTF-16
constexpr uint8_t TEXT_RFU = 0x40;        // b6: reserved, must be 0
constexpr uint8_t TEXT_LANG_MASK = 0x3F;  // b5..b0: length of the IANA language code
constexpr size_t kMaxLangLength = 63;

// Highest URI identifier code defined (23h "urn:nfc:"); 24h..FFh are RFU.  RTD URI 3.2.2
constexpr uint8_t kUriCodeMax = 0x23;
}  // namespace ndef

// One record of a parsed message. Pointers point into the message buffer.
struct NdefRecord {
    uint8_t header = 0;           // raw first octet (flags + TNF)
    uint8_t tnf = 0;              // header & ndef::TNF_MASK
    const uint8_t* type = nullptr;
    size_t typeLength = 0;
    const uint8_t* id = nullptr;
    size_t idLength = 0;
    const uint8_t* payload = nullptr;
    size_t payloadLength = 0;
    bool mb() const { return (header & ndef::FLAG_MB) != 0; }
    bool me() const { return (header & ndef::FLAG_ME) != 0; }
    bool shortRecord() const { return (header & ndef::FLAG_SR) != 0; }
    // True for TNF Well-known with the single-character type `t` (e.g. 'U').
    bool isWellKnown(uint8_t t) const {
        return tnf == ndef::TNF_WELL_KNOWN && typeLength == 1 && type != nullptr && type[0] == t;
    }
};

// Forward iterator over a raw NDEF message. Never reads outside
// [msg, msg + len). Usage:
//   NdefParser p(buf, len);
//   NdefRecord r;
//   while (!p.done()) { Status s = p.next(r); if (!ok(s)) break; ... }
// An empty message (len 0, e.g. an NDEF TLV of length 0) has no records and
// is done() at once. A null `msg` with a non-zero `len` is invalid input, not
// an empty message: done() is false and next() returns InvalidArgument.
class NdefParser {
public:
    NdefParser(const uint8_t* msg, size_t len) : msg_(msg), len_(len) {}

    // True after the record with ME set, for an empty message, or after an
    // error (the parser does not continue past malformed data).
    bool done() const { return done_; }

    // Next record:
    //   Ok             `rec` filled
    //   MalformedData  a length runs past the end of the message, MB missing
    //                  on the first record or set on a later one, the
    //                  message ends without ME, or a record violates the TNF
    //                  rules (Empty with non-zero lengths, Unknown/Unchanged
    //                  with a type, Unchanged outside a chunk)
    //   Unsupported    chunked record (CF = 1): not supported by this driver
    //   InvalidArgument called when done(), or null `msg` with len > 0
    Status next(NdefRecord& rec);

    // Number of records returned so far.
    size_t count() const { return count_; }

private:
    const uint8_t* msg_;
    size_t len_;
    size_t pos_ = 0;
    size_t count_ = 0;
    bool done_ = (len_ == 0);
};

// Parses the whole message; `records` = number of records (0 for len 0).
// Returns the first error of NdefParser::next(), else Ok.
Status validateNdefMessage(const uint8_t* msg, size_t len, size_t& records);

// Appends records to a caller buffer. MB is set on the first record and ME on
// the last one appended (the previous record's ME is cleared on each
// append). Short-record format is used when the payload is <= 255 bytes.
// A failed append leaves the buffer content unchanged.
class NdefBuilder {
public:
    NdefBuilder(uint8_t* buf, size_t capacity) : buf_(buf), cap_(buf == nullptr ? 0 : capacity) {}

    // Generic record (no chunking).
    //   BufferTooSmall  record does not fit into the remaining capacity
    //   InvalidArgument TNF Unchanged/Reserved, type/ID longer than 255,
    //                   Empty with any length != 0, Unknown with a type,
    //                   null pointer with non-zero length
    Status addRecord(uint8_t tnf, const uint8_t* type, size_t typeLength, const uint8_t* id,
                     size_t idLength, const uint8_t* payload, size_t payloadLength);

    // Well-known URI record: the longest matching URI identifier code is
    // used and the rest of `uri` (NUL-terminated) stored after it.  RTD URI 3.2
    Status addUri(const char* uri);

    // Well-known Text record, UTF-8: status byte (language length), the
    // language code (e.g. "en", 1..63 bytes) and the text.  RTD Text 3.2
    Status addText(const char* text, const char* lang = "en");

    const uint8_t* data() const { return buf_; }
    size_t size() const { return size_; }
    size_t records() const { return records_; }

private:
    // Record whose payload is `head` followed by `body`.
    Status addParts(uint8_t tnf, const uint8_t* type, size_t typeLength, const uint8_t* id,
                    size_t idLength, const uint8_t* head, size_t headLength, const uint8_t* body,
                    size_t bodyLength);

    uint8_t* buf_;
    size_t cap_;
    size_t size_ = 0;
    size_t records_ = 0;
    size_t lastHeader_ = 0;  // offset of the last record's header octet
};

// URI identifier code table.                         RTD URI 3.2.2
// Prefix string for `code` ("" for 00h), nullptr for RFU codes (24h..FFh).
const char* ndefUriPrefix(uint8_t code);
// Code of the longest prefix in the table that `uri` starts with (00h if
// none) and that prefix's length.
uint8_t ndefUriCodeFor(const char* uri, size_t& prefixLength);

// Decoded URI record: prefix (from the code) + rest (bytes of the payload).
struct NdefUri {
    const char* prefix = "";
    const uint8_t* rest = nullptr;
    size_t restLength = 0;
};
//   Ok, InvalidArgument (not a Well-known "U" record), MalformedData (empty
//   payload), Unsupported (RFU identifier code).
Status decodeNdefUri(const NdefRecord& rec, NdefUri& out);
// True if prefix + rest equals `uri` (NUL-terminated).
bool ndefUriEquals(const NdefUri& u, const char* uri);

// Decoded Text record (pointers into the payload).
struct NdefText {
    bool utf16 = false;
    const uint8_t* lang = nullptr;
    size_t langLength = 0;
    const uint8_t* text = nullptr;
    size_t textLength = 0;
};
//   Ok, InvalidArgument (not a Well-known "T" record), MalformedData (empty
//   payload, language length past the payload, or RFU bit b6 set).
Status decodeNdefText(const NdefRecord& rec, NdefText& out);

}  // namespace pn5180
