// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NDEF message parser / builder and RTD URI / Text helpers. Portable core.
#include "Ndef.h"

namespace pn5180 {

using namespace ndef;

namespace {

size_t cstrLength(const char* s) {
    size_t n = 0;
    while (s[n] != '\0') {
        ++n;
    }
    return n;
}

// URI identifier codes 00h..23h.                      RTD URI 3.2.2 (Table 3)
constexpr const char* kUriPrefixes[kUriCodeMax + 1] = {
    "",                            // 00h: no prepending
    "http://www.",                 // 01h
    "https://www.",                // 02h
    "http://",                     // 03h
    "https://",                    // 04h
    "tel:",                        // 05h
    "mailto:",                     // 06h
    "ftp://anonymous:anonymous@",  // 07h
    "ftp://ftp.",                  // 08h
    "ftps://",                     // 09h
    "sftp://",                     // 0Ah
    "smb://",                      // 0Bh
    "nfs://",                      // 0Ch
    "ftp://",                      // 0Dh
    "dav://",                      // 0Eh
    "news:",                       // 0Fh
    "telnet://",                   // 10h
    "imap:",                       // 11h
    "rtsp://",                     // 12h
    "urn:",                        // 13h
    "pop:",                        // 14h
    "sip:",                        // 15h
    "sips:",                       // 16h
    "tftp:",                       // 17h
    "btspp://",                    // 18h
    "btl2cap://",                  // 19h
    "btgoep://",                   // 1Ah
    "tcpobex://",                  // 1Bh
    "irdaobex://",                 // 1Ch
    "file://",                     // 1Dh
    "urn:epc:id:",                 // 1Eh
    "urn:epc:tag:",                // 1Fh
    "urn:epc:pat:",                // 20h
    "urn:epc:raw:",                // 21h
    "urn:epc:",                    // 22h
    "urn:nfc:",                    // 23h
};

// TNF-specific length rules.                         NDEF 3.2.6
//   Empty: TYPE_LENGTH, ID_LENGTH and PAYLOAD_LENGTH are 0.
//   Unknown, Unchanged: TYPE_LENGTH is 0.
//   Well-known, Media, Absolute URI, External: a TYPE is present.
bool tnfLengthsValid(uint8_t tnf, size_t typeLength, size_t idLength, size_t payloadLength) {
    switch (tnf) {
        case TNF_EMPTY:
            return typeLength == 0 && idLength == 0 && payloadLength == 0;
        case TNF_UNKNOWN:
        case TNF_UNCHANGED:
            return typeLength == 0;
        case TNF_WELL_KNOWN:
        case TNF_MEDIA:
        case TNF_ABSOLUTE_URI:
        case TNF_EXTERNAL:
            return typeLength != 0;
        default:
            // Reserved (07h): a parser treats it like Unknown.  NDEF 3.2.6
            return true;
    }
}

}  // namespace

// ---------------------------------------------------------------- parser

Status NdefParser::next(NdefRecord& rec) {
    if (done_) {
        return Status::InvalidArgument;
    }
    // From here on any failure ends the iteration.
    done_ = true;
    if (msg_ == nullptr) {  // len_ > 0 here (len 0 starts done)
        return Status::InvalidArgument;
    }
    size_t avail = len_ - pos_;
    const uint8_t* p = msg_ + pos_;
    // Header octet + TYPE_LENGTH are always present.  NDEF 3.2 (record layout)
    if (avail < 2) {
        return Status::MalformedData;
    }
    const uint8_t h = p[0];
    if ((h & FLAG_CF) != 0) {
        return Status::Unsupported;  // chunked payloads not supported  NDEF 3.2.3
    }
    // MB on the first record only.                     NDEF 3.2.1
    const bool mb = (h & FLAG_MB) != 0;
    if (mb != (count_ == 0)) {
        return Status::MalformedData;
    }
    const size_t typeLength = p[1];
    size_t off = 2;
    avail -= 2;
    size_t payloadLength = 0;
    if ((h & FLAG_SR) != 0) {  // 1-octet PAYLOAD_LENGTH  NDEF 3.2.4
        if (avail < 1) {
            return Status::MalformedData;
        }
        payloadLength = p[off];
        off += 1;
        avail -= 1;
    } else {  // 4-octet PAYLOAD_LENGTH, MSB first      NDEF 3.2.9
        if (avail < 4) {
            return Status::MalformedData;
        }
        const uint32_t pl = (static_cast<uint32_t>(p[off]) << 24) |
                            (static_cast<uint32_t>(p[off + 1]) << 16) |
                            (static_cast<uint32_t>(p[off + 2]) << 8) | p[off + 3];
        off += 4;
        avail -= 4;
        // Compare in 64 bits: a 32-bit size_t must not truncate the length.
        if (static_cast<uint64_t>(pl) > static_cast<uint64_t>(avail)) {
            return Status::MalformedData;
        }
        payloadLength = static_cast<size_t>(pl);
    }
    size_t idLength = 0;
    if ((h & FLAG_IL) != 0) {  // ID_LENGTH present      NDEF 3.2.5, 3.2.8
        if (avail < 1) {
            return Status::MalformedData;
        }
        idLength = p[off];
        off += 1;
        avail -= 1;
    }
    // TYPE, ID, PAYLOAD in this order.               NDEF 3.2 (record layout)
    if (typeLength > avail) {
        return Status::MalformedData;
    }
    avail -= typeLength;
    if (idLength > avail) {
        return Status::MalformedData;
    }
    avail -= idLength;
    if (payloadLength > avail) {
        return Status::MalformedData;
    }
    const uint8_t tnf = static_cast<uint8_t>(h & TNF_MASK);
    // Unchanged is only valid in middle/terminating chunks, which are not
    // supported (CF rejected above).                  NDEF 3.2.6
    if (tnf == TNF_UNCHANGED || !tnfLengthsValid(tnf, typeLength, idLength, payloadLength)) {
        return Status::MalformedData;
    }
    const bool me = (h & FLAG_ME) != 0;
    const size_t end = pos_ + off + typeLength + idLength + payloadLength;
    // ME marks the last record: nothing may follow it, and the message must
    // not end without it.                             NDEF 3.2.2
    if (me != (end == len_)) {
        return Status::MalformedData;
    }
    rec.header = h;
    rec.tnf = tnf;
    rec.type = p + off;
    rec.typeLength = typeLength;
    rec.id = p + off + typeLength;
    rec.idLength = idLength;
    rec.payload = p + off + typeLength + idLength;
    rec.payloadLength = payloadLength;
    pos_ = end;
    ++count_;
    done_ = me;
    return Status::Ok;
}

Status validateNdefMessage(const uint8_t* msg, size_t len, size_t& records) {
    records = 0;
    if (msg == nullptr && len != 0) {
        return Status::InvalidArgument;
    }
    NdefParser parser(msg, len);
    NdefRecord rec;
    while (!parser.done()) {
        const Status s = parser.next(rec);
        if (!ok(s)) {
            return s;
        }
    }
    records = parser.count();
    return Status::Ok;
}

// ---------------------------------------------------------------- builder

Status NdefBuilder::addRecord(uint8_t tnf, const uint8_t* type, size_t typeLength,
                              const uint8_t* id, size_t idLength, const uint8_t* payload,
                              size_t payloadLength) {
    return addParts(tnf, type, typeLength, id, idLength, payload, payloadLength, nullptr, 0);
}

Status NdefBuilder::addParts(uint8_t tnf, const uint8_t* type, size_t typeLength,
                             const uint8_t* id, size_t idLength, const uint8_t* head,
                             size_t headLength, const uint8_t* body, size_t bodyLength) {
    if ((type == nullptr && typeLength != 0) || (id == nullptr && idLength != 0) ||
        (head == nullptr && headLength != 0) || (body == nullptr && bodyLength != 0)) {
        return Status::InvalidArgument;
    }
    if (headLength > SIZE_MAX - bodyLength) {
        return Status::InvalidArgument;
    }
    const size_t payloadLength = headLength + bodyLength;
    // Unchanged (chunks only) and Reserved must not be written.  NDEF 3.2.6
    if (tnf > TNF_UNKNOWN || typeLength > kMaxTypeLength || idLength > kMaxIdLength ||
        // Must fit the 4-octet field (two 16-bit shifts: no warning or UB
        // where size_t has 32 bits).                   NDEF 3.2.9
        ((payloadLength >> 16) >> 16) != 0 ||
        !tnfLengthsValid(tnf, typeLength, idLength, payloadLength)) {
        return Status::InvalidArgument;
    }
    const bool sr = payloadLength <= kShortPayloadMax;
    const bool il = idLength != 0;
    const size_t fixed = 2 + (sr ? 1u : 4u) + (il ? 1u : 0u);
    // Fit check without overflow: each part against what is left.
    size_t left = cap_ - size_;
    if (fixed > left) {
        return Status::BufferTooSmall;
    }
    left -= fixed;
    if (typeLength > left) {
        return Status::BufferTooSmall;
    }
    left -= typeLength;
    if (idLength > left) {
        return Status::BufferTooSmall;
    }
    left -= idLength;
    if (payloadLength > left) {
        return Status::BufferTooSmall;
    }

    uint8_t h = static_cast<uint8_t>(FLAG_ME | tnf);
    if (records_ == 0) {
        h |= FLAG_MB;
    }
    if (sr) {
        h |= FLAG_SR;
    }
    if (il) {
        h |= FLAG_IL;
    }
    size_t w = size_;
    buf_[w++] = h;
    buf_[w++] = static_cast<uint8_t>(typeLength);
    if (sr) {
        buf_[w++] = static_cast<uint8_t>(payloadLength);
    } else {
        const uint32_t pl = static_cast<uint32_t>(payloadLength);
        buf_[w++] = static_cast<uint8_t>(pl >> 24);
        buf_[w++] = static_cast<uint8_t>(pl >> 16);
        buf_[w++] = static_cast<uint8_t>(pl >> 8);
        buf_[w++] = static_cast<uint8_t>(pl);
    }
    if (il) {
        buf_[w++] = static_cast<uint8_t>(idLength);
    }
    for (size_t i = 0; i < typeLength; ++i) {
        buf_[w++] = type[i];
    }
    for (size_t i = 0; i < idLength; ++i) {
        buf_[w++] = id[i];
    }
    for (size_t i = 0; i < headLength; ++i) {
        buf_[w++] = head[i];
    }
    for (size_t i = 0; i < bodyLength; ++i) {
        buf_[w++] = body[i];
    }
    // The previous record is no longer the last one.   NDEF 3.2.2
    if (records_ != 0) {
        buf_[lastHeader_] = static_cast<uint8_t>(buf_[lastHeader_] & ~FLAG_ME);
    }
    lastHeader_ = size_;
    size_ = w;
    ++records_;
    return Status::Ok;
}

Status NdefBuilder::addUri(const char* uri) {
    if (uri == nullptr) {
        return Status::InvalidArgument;
    }
    size_t prefixLength = 0;
    const uint8_t code = ndefUriCodeFor(uri, prefixLength);
    const uint8_t type = RTD_URI;
    // Payload: identifier code, then the URI field.   RTD URI 3.2.1
    return addParts(TNF_WELL_KNOWN, &type, 1, nullptr, 0, &code, 1,
                    reinterpret_cast<const uint8_t*>(uri + prefixLength),
                    cstrLength(uri + prefixLength));
}

Status NdefBuilder::addText(const char* text, const char* lang) {
    if (text == nullptr || lang == nullptr) {
        return Status::InvalidArgument;
    }
    const size_t langLength = cstrLength(lang);
    if (langLength == 0 || langLength > kMaxLangLength) {
        return Status::InvalidArgument;
    }
    // Payload: status byte (UTF-8, language length), language code, text.  RTD Text 3.2
    uint8_t head[1 + kMaxLangLength] = {};
    head[0] = static_cast<uint8_t>(langLength);
    for (size_t i = 0; i < langLength; ++i) {
        head[1 + i] = static_cast<uint8_t>(lang[i]);
    }
    const uint8_t type = RTD_TEXT;
    return addParts(TNF_WELL_KNOWN, &type, 1, nullptr, 0, head, 1 + langLength,
                    reinterpret_cast<const uint8_t*>(text), cstrLength(text));
}

// ---------------------------------------------------------------- URI / Text

const char* ndefUriPrefix(uint8_t code) {
    return code <= kUriCodeMax ? kUriPrefixes[code] : nullptr;
}

uint8_t ndefUriCodeFor(const char* uri, size_t& prefixLength) {
    prefixLength = 0;
    uint8_t best = 0;
    if (uri == nullptr) {
        return best;
    }
    for (uint8_t code = 1; code <= kUriCodeMax; ++code) {
        const char* p = kUriPrefixes[code];
        size_t n = 0;
        while (p[n] != '\0' && uri[n] == p[n]) {
            ++n;
        }
        if (p[n] == '\0' && n > prefixLength) {  // whole prefix matched, longer than best
            best = code;
            prefixLength = n;
        }
    }
    return best;
}

Status decodeNdefUri(const NdefRecord& rec, NdefUri& out) {
    if (!rec.isWellKnown(RTD_URI)) {
        return Status::InvalidArgument;
    }
    if (rec.payloadLength < 1 || rec.payload == nullptr) {
        return Status::MalformedData;
    }
    const char* prefix = ndefUriPrefix(rec.payload[0]);
    if (prefix == nullptr) {
        return Status::Unsupported;  // RFU identifier code  RTD URI 3.2.2
    }
    out.prefix = prefix;
    out.rest = rec.payload + 1;
    out.restLength = rec.payloadLength - 1;
    return Status::Ok;
}

bool ndefUriEquals(const NdefUri& u, const char* uri) {
    if (uri == nullptr || u.prefix == nullptr) {
        return false;
    }
    size_t i = 0;
    for (; u.prefix[i] != '\0'; ++i) {
        if (uri[i] != u.prefix[i]) {
            return false;
        }
    }
    for (size_t k = 0; k < u.restLength; ++k, ++i) {
        if (uri[i] == '\0' || static_cast<uint8_t>(uri[i]) != u.rest[k]) {
            return false;
        }
    }
    return uri[i] == '\0';
}

Status decodeNdefText(const NdefRecord& rec, NdefText& out) {
    if (!rec.isWellKnown(RTD_TEXT)) {
        return Status::InvalidArgument;
    }
    if (rec.payloadLength < 1 || rec.payload == nullptr) {
        return Status::MalformedData;
    }
    const uint8_t status = rec.payload[0];  //       RTD Text 3.2.1
    const size_t langLength = status & TEXT_LANG_MASK;
    if ((status & TEXT_RFU) != 0 || langLength > rec.payloadLength - 1) {
        return Status::MalformedData;
    }
    out.utf16 = (status & TEXT_UTF16) != 0;
    out.lang = rec.payload + 1;
    out.langLength = langLength;
    out.text = rec.payload + 1 + langLength;
    out.textLength = rec.payloadLength - 1 - langLength;
    return Status::Ok;
}

}  // namespace pn5180
