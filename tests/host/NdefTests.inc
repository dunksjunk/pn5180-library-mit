// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host tests for the NDEF core, the TLV layer and the Type 2 / Type 5 Tag
// NDEF mappings. Included by test_core.cpp (uses its EXPECT macro and rigs).

// Test memory: `size` bytes, unit `unit`, TLV area [start, end).
struct FakeMem final : NdefMemory {
    uint8_t bytes[1024] = {};
    size_t unit = 4;
    size_t start = 16;
    size_t end = 16 + 496;
    bool ro = false;
    int writes = 0;
    int failAfterWrites = -1;  // >= 0: writes beyond this count fail (NoResponse)
    int weakWrites = 0;        // n > 0: the next n writes store byte 0 ^ 01h
    size_t unitSize() const override { return unit; }
    size_t areaStart() const override { return start; }
    size_t areaEnd() const override { return end; }
    bool readOnly() const override { return ro; }
    Status readUnit(size_t u, uint8_t* out) override {
        if ((u + 1) * unit > sizeof(bytes)) return Status::InvalidArgument;
        std::memcpy(out, bytes + u * unit, unit);
        return Status::Ok;
    }
    Status writeUnit(size_t u, const uint8_t* in) override {
        if ((u + 1) * unit > sizeof(bytes) || u * unit < start) return Status::InvalidArgument;
        if (failAfterWrites >= 0 && writes >= failAfterWrites) return Status::NoResponse;
        std::memcpy(bytes + u * unit, in, unit);
        if (weakWrites > 0) {
            bytes[u * unit] ^= 0x01;
            --weakWrites;
        }
        ++writes;
        return Status::Ok;
    }
};

size_t buildTestMessage(uint8_t* buf, size_t cap, const char* uri, const char* text) {
    NdefBuilder b(buf, cap);
    EXPECT(b.addUri(uri) == Status::Ok);
    EXPECT(b.addText(text, "en") == Status::Ok);
    return b.size();
}

bool sameBytes(const uint8_t* a, size_t na, const uint8_t* b, size_t nb) {
    return na == nb && (na == 0 || std::memcmp(a, b, na) == 0);
}

void testNdefRecords() {
    // URI + Text round trip, MB/ME, short records.
    {
        uint8_t msg[128] = {};
        const size_t n = buildTestMessage(msg, sizeof(msg), "https://www.example.com/a", "hi");
        // URI record: MB + SR, not ME: 91h; type 'U', identifier code 02h.
        EXPECT(msg[0] == 0x91 && msg[1] == 1 && msg[2] == 1 + 13 && msg[3] == 'U' && msg[4] == 0x02);
        size_t count = 0;
        EXPECT(validateNdefMessage(msg, n, count) == Status::Ok && count == 2);
        NdefParser p(msg, n);
        NdefRecord r;
        EXPECT(p.next(r) == Status::Ok && r.mb() && !r.me() && r.shortRecord());
        NdefUri u;
        EXPECT(decodeNdefUri(r, u) == Status::Ok);
        EXPECT(ndefUriEquals(u, "https://www.example.com/a"));
        EXPECT(!ndefUriEquals(u, "https://www.example.com/ab"));
        EXPECT(!ndefUriEquals(u, "https://www.example.com/"));
        EXPECT(p.next(r) == Status::Ok && !r.mb() && r.me() && p.done());
        NdefText t;
        EXPECT(decodeNdefText(r, t) == Status::Ok && !t.utf16 && t.langLength == 2);
        EXPECT(t.textLength == 2 && std::memcmp(t.text, "hi", 2) == 0);
        EXPECT(std::memcmp(t.lang, "en", 2) == 0);
        EXPECT(decodeNdefUri(r, u) == Status::InvalidArgument);  // not a URI record
        EXPECT(p.next(r) == Status::InvalidArgument);           // past the end
    }
    // URI prefix table: longest match, edges.
    {
        size_t pl = 0;
        EXPECT(ndefUriCodeFor("https://www.x", pl) == 0x02 && pl == 12);
        EXPECT(ndefUriCodeFor("https://x", pl) == 0x04 && pl == 8);
        EXPECT(ndefUriCodeFor("http://", pl) == 0x03 && pl == 7);
        EXPECT(ndefUriCodeFor("urn:epc:id:1", pl) == 0x1E && pl == 11);
        EXPECT(ndefUriCodeFor("urn:epc:x", pl) == 0x22);
        EXPECT(ndefUriCodeFor("urn:x", pl) == 0x13);
        EXPECT(ndefUriCodeFor("urn:nfc:sn", pl) == 0x23);
        EXPECT(ndefUriCodeFor("ftp://anonymous:anonymous@h", pl) == 0x07);
        EXPECT(ndefUriCodeFor("ftp://ftp.h", pl) == 0x08);
        EXPECT(ndefUriCodeFor("ftp://h", pl) == 0x0D);
        EXPECT(ndefUriCodeFor("HTTP://x", pl) == 0x00 && pl == 0);  // case-sensitive
        EXPECT(ndefUriCodeFor("", pl) == 0x00 && pl == 0);
        EXPECT(std::strcmp(ndefUriPrefix(0x00), "") == 0);
        EXPECT(std::strcmp(ndefUriPrefix(0x23), "urn:nfc:") == 0);
        EXPECT(ndefUriPrefix(0x24) == nullptr && ndefUriPrefix(0xFF) == nullptr);
        // Prefix-only URI and RFU code on decode.
        uint8_t msg[16] = {};
        NdefBuilder b(msg, sizeof(msg));
        EXPECT(b.addUri("tel:") == Status::Ok);
        NdefParser p(msg, b.size());
        NdefRecord r;
        NdefUri u;
        EXPECT(p.next(r) == Status::Ok && decodeNdefUri(r, u) == Status::Ok);
        EXPECT(u.restLength == 0 && ndefUriEquals(u, "tel:"));
        msg[4] = 0x24;
        EXPECT(decodeNdefUri(r, u) == Status::Unsupported);
    }
    // Long record (payload > 255): 4-byte length, SR clear; and an ID.
    {
        static uint8_t payload[300];
        for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = uint8_t(i);
        uint8_t msg[400] = {};
        NdefBuilder b(msg, sizeof(msg));
        const uint8_t type[] = {'a', '/', 'b'};
        const uint8_t id[] = {'#', '1'};
        EXPECT(b.addRecord(ndef::TNF_MEDIA, type, 3, id, 2, payload, 300) == Status::Ok);
        EXPECT(b.size() == 1 + 1 + 4 + 1 + 3 + 2 + 300);
        EXPECT((msg[0] & ndef::FLAG_SR) == 0 && (msg[0] & ndef::FLAG_IL) != 0);
        EXPECT(msg[0] == (ndef::FLAG_MB | ndef::FLAG_ME | ndef::FLAG_IL | ndef::TNF_MEDIA));
        NdefParser p(msg, b.size());
        NdefRecord r;
        EXPECT(p.next(r) == Status::Ok && r.payloadLength == 300 && r.idLength == 2);
        EXPECT(std::memcmp(r.payload, payload, 300) == 0 && r.id[0] == '#' && p.done());
        // Exactly 255 stays short.
        NdefBuilder b2(msg, sizeof(msg));
        EXPECT(b2.addRecord(ndef::TNF_UNKNOWN, nullptr, 0, nullptr, 0, payload, 255) == Status::Ok);
        EXPECT((msg[0] & ndef::FLAG_SR) != 0 && b2.size() == 3 + 255);
    }
    // Builder: BufferTooSmall leaves the buffer unchanged; bad arguments.
    {
        uint8_t msg[24] = {};
        NdefBuilder b(msg, sizeof(msg));
        EXPECT(b.addUri("https://abc") == Status::Ok);
        uint8_t copy[24];
        std::memcpy(copy, msg, sizeof(msg));
        const size_t before = b.size();
        EXPECT(b.addText("this text is far too long", "en") == Status::BufferTooSmall);
        EXPECT(b.size() == before && std::memcmp(copy, msg, sizeof(msg)) == 0);
        size_t count = 0;
        EXPECT(validateNdefMessage(msg, b.size(), count) == Status::Ok && count == 1);
        EXPECT(b.addRecord(ndef::TNF_UNCHANGED, nullptr, 0, nullptr, 0, nullptr, 0) ==
               Status::InvalidArgument);
        const uint8_t t = 'X';
        EXPECT(b.addRecord(ndef::TNF_EMPTY, &t, 1, nullptr, 0, nullptr, 0) == Status::InvalidArgument);
        EXPECT(b.addRecord(ndef::TNF_WELL_KNOWN, nullptr, 0, nullptr, 0, nullptr, 0) ==
               Status::InvalidArgument);
        EXPECT(b.addText("x", "") == Status::InvalidArgument);
        NdefBuilder none(nullptr, 10);
        EXPECT(none.addUri("x") == Status::BufferTooSmall);
    }
    // Malformed messages.
    {
        uint8_t m[8] = {0xD1, 0x01, 0x01, 'T', 0x00};  // valid single Text record (lang len 0)
        size_t c = 0;
        EXPECT(validateNdefMessage(m, 5, c) == Status::Ok && c == 1);
        m[0] = 0xF1;  // CF set
        EXPECT(validateNdefMessage(m, 5, c) == Status::Unsupported);
        m[0] = 0x51;  // MB missing
        EXPECT(validateNdefMessage(m, 5, c) == Status::MalformedData);
        m[0] = 0x91;  // ME missing
        EXPECT(validateNdefMessage(m, 5, c) == Status::MalformedData);
        m[0] = 0xD1;
        EXPECT(validateNdefMessage(m, 6, c) == Status::MalformedData);  // trailing byte after ME
        m[0] = 0xD6;  // TNF Unchanged outside a chunk
        EXPECT(validateNdefMessage(m, 5, c) == Status::MalformedData);
        m[0] = 0xD0;  // Empty with a type
        EXPECT(validateNdefMessage(m, 5, c) == Status::MalformedData);
        const uint8_t empty[3] = {0xD0, 0x00, 0x00};
        EXPECT(validateNdefMessage(empty, 3, c) == Status::Ok && c == 1);
        EXPECT(validateNdefMessage(nullptr, 0, c) == Status::Ok && c == 0);
        EXPECT(validateNdefMessage(nullptr, 5, c) == Status::InvalidArgument && c == 0);
        {
            // Null buffer with a length is invalid input, not an empty message.
            NdefParser bad(nullptr, 5);
            NdefRecord br;
            EXPECT(!bad.done());
            EXPECT(bad.next(br) == Status::InvalidArgument);
            EXPECT(bad.done() && bad.count() == 0);
            NdefParser none(nullptr, 0);
            EXPECT(none.done());
        }
        // Long record with a huge 4-byte length.
        const uint8_t huge[8] = {0xC5, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
        EXPECT(validateNdefMessage(huge, 8, c) == Status::MalformedData);
        // Text: language length past the payload, RFU bit.
        const uint8_t badText[6] = {0xD1, 0x01, 0x02, 'T', 0x05, 'e'};
        NdefParser p(badText, 6);
        NdefRecord r;
        NdefText t;
        EXPECT(p.next(r) == Status::Ok && decodeNdefText(r, t) == Status::MalformedData);
    }
    // Fuzz-style: every truncation of valid messages fails cleanly, and the
    // parser only touches bytes inside the (exactly sized, heap-allocated)
    // buffer; every single-byte corruption of the header bytes never crashes.
    {
        uint8_t msgs[3][400] = {};
        size_t lens[3] = {};
        lens[0] = buildTestMessage(msgs[0], 400, "https://www.example.com/x", "Text 1");
        NdefBuilder b(msgs[1], 400);
        static uint8_t big[300] = {};
        const uint8_t type[] = {'x', '/', 'y'};
        const uint8_t id[] = {'i'};
        EXPECT(b.addRecord(ndef::TNF_EXTERNAL, type, 3, id, 1, big, 300) == Status::Ok);
        EXPECT(b.addUri("mailto:a@b") == Status::Ok);
        lens[1] = b.size();
        NdefBuilder b3(msgs[2], 400);
        EXPECT(b3.addRecord(ndef::TNF_EMPTY, nullptr, 0, nullptr, 0, nullptr, 0) == Status::Ok);
        lens[2] = b3.size();
        for (int k = 0; k < 3; ++k) {
            size_t c = 0;
            EXPECT(validateNdefMessage(msgs[k], lens[k], c) == Status::Ok);
            for (size_t n = 1; n < lens[k]; ++n) {
                uint8_t* exact = new uint8_t[n];
                std::memcpy(exact, msgs[k], n);
                EXPECT(validateNdefMessage(exact, n, c) == Status::MalformedData);
                delete[] exact;
            }
            for (size_t i = 0; i < lens[k] && i < 16; ++i) {
                for (int v = 0; v < 256; v += 7) {
                    uint8_t* exact = new uint8_t[lens[k]];
                    std::memcpy(exact, msgs[k], lens[k]);
                    exact[i] = uint8_t(v);
                    (void)validateNdefMessage(exact, lens[k], c);
                    delete[] exact;
                }
            }
        }
    }
}

void testNdefTlv() {
    uint8_t msg[400] = {};
    const size_t n = buildTestMessage(msg, sizeof(msg), "https://example.com/t", "tlv test");
    // NULL, Lock Control, Memory Control and proprietary TLVs before the NDEF TLV.
    {
        FakeMem m;
        const uint8_t pre[] = {0x00, 0x01, 0x03, 0xA0, 0x10, 0x44,  // lock: byte 160, 16 bits
                               0x02, 0x03, 0xF0, 0x04, 0x02,        // memory: byte 60, 4 bytes
                               0xFD, 0x01, 0x55, 0x00};
        std::memcpy(m.bytes + 16, pre, sizeof(pre));
        size_t at = 16 + sizeof(pre);
        m.bytes[at] = 0x03;
        m.bytes[at + 1] = uint8_t(n);
        std::memcpy(m.bytes + at + 2, msg, n);
        m.bytes[at + 2 + n] = 0xFE;
        NdefTlvScan scan;
        EXPECT(scanNdefTlv(m, scan) == Status::Ok && scan.found && scan.tlvStart == at);
        EXPECT(scan.reservedCount == 2);
        EXPECT(scan.reserved[0].start == 160 && scan.reserved[0].end == 162);
        EXPECT(scan.reserved[1].start == 60 && scan.reserved[1].end == 64);
        uint8_t out[400] = {};
        size_t len = 0;
        // Message 16+15..: overlaps the memory control area at 60..63.
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Unsupported);
        m.bytes[16 + 10] = 0x07;  // 128-byte pages: memory area at 15 * 128 = 1920, outside
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
        EXPECT(readNdefMessage(m, out, 10, len) == Status::BufferTooSmall && len == n);
        // Write keeps the control TLVs and goes to the same place.
        uint8_t msg2[64] = {};
        const size_t n2 = buildTestMessage(msg2, sizeof(msg2), "tel:1", "x");
        EXPECT(writeNdefMessage(m, msg2, n2) == Status::Ok);
        EXPECT(std::memcmp(m.bytes + 16, pre, 8) == 0 && m.bytes[at] == 0x03);
        EXPECT(size_t(m.bytes[at + 1]) == n2 && m.bytes[at + 2 + n2] == 0xFE);
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg2, n2));
    }
    // 3-byte length form, write and read; blank (all NULL) area; empty TLV.
    {
        FakeMem m;
        uint8_t out[600] = {};
        size_t len = 99;
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok && len == 0);
        static uint8_t big[300] = {};
        uint8_t msgBig[320] = {};
        for (size_t i = 0; i < sizeof(big); ++i) big[i] = uint8_t(i * 3);
        NdefBuilder b(msgBig, sizeof(msgBig));
        EXPECT(b.addRecord(ndef::TNF_UNKNOWN, nullptr, 0, nullptr, 0, big, 300) == Status::Ok);
        EXPECT(writeNdefMessage(m, msgBig, b.size()) == Status::Ok);
        EXPECT(m.bytes[16] == 0x03 && m.bytes[17] == 0xFF);
        EXPECT(m.bytes[18] == uint8_t(b.size() >> 8) && m.bytes[19] == uint8_t(b.size()));
        EXPECT(m.bytes[20 + b.size()] == 0xFE);
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok &&
               sameBytes(out, len, msgBig, b.size()));
        EXPECT(writeNdefMessage(m, nullptr, 0) == Status::Ok);
        EXPECT(m.bytes[16] == 0x03 && m.bytes[17] == 0x00 && m.bytes[18] == 0xFE);
        EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok && len == 0);
    }
    // Malformed TLV areas.
    {
        FakeMem m;
        m.bytes[16] = 0x03;
        m.bytes[17] = 0xFF;
        m.bytes[18] = 0x02;  // 512 bytes > area
        m.bytes[19] = 0x00;
        NdefTlvScan scan;
        EXPECT(scanNdefTlv(m, scan) == Status::MalformedData);
        m.bytes[16] = 0x01;  // lock control with length != 3
        m.bytes[17] = 0x02;
        EXPECT(scanNdefTlv(m, scan) == Status::MalformedData);
        m.end = 17;  // TLV header cut by the end of the area
        EXPECT(scanNdefTlv(m, scan) == Status::MalformedData);
        m.unit = 0;
        EXPECT(scanNdefTlv(m, scan) == Status::NotFormatted);
    }
    // Fit, read-only, write order (an interrupted write leaves a valid area).
    {
        FakeMem m;
        m.end = 16 + 8;
        uint8_t big[16] = {};
        EXPECT(writeNdefMessage(m, big, 7) == Status::BufferTooSmall);
        EXPECT(writeNdefMessage(m, big, 6) == Status::Ok && m.bytes[16 + 1] == 6);  // no Terminator room
        m.ro = true;
        EXPECT(writeNdefMessage(m, big, 1) == Status::ReadOnly);
    }
    {
        uint8_t oldMsg[64] = {};
        const size_t no = buildTestMessage(oldMsg, sizeof(oldMsg), "http://old", "old");
        for (int fail = 0; fail < 40; ++fail) {
            FakeMem m;
            EXPECT(writeNdefMessage(m, oldMsg, no) == Status::Ok);
            m.writes = 0;
            m.failAfterWrites = fail;
            const Status ws = writeNdefMessage(m, msg, n);
            m.failAfterWrites = -1;
            uint8_t out[400] = {};
            size_t len = 0;
            EXPECT(readNdefMessage(m, out, sizeof(out), len) == Status::Ok);
            const bool valid = len == 0 || sameBytes(out, len, oldMsg, no) || sameBytes(out, len, msg, n);
            EXPECT(valid);
            if (ok(ws)) EXPECT(sameBytes(out, len, msg, n));
            // Interrupted after the first write: the TLV reads as empty.
            if (fail == 1) EXPECT(len == 0);
        }
    }
    // Backup / restore, also after a failed verify of the written message.
    {
        FakeMem m;
        uint8_t oldMsg[64] = {};
        const size_t no = buildTestMessage(oldMsg, sizeof(oldMsg), "http://old", "old");
        EXPECT(writeNdefMessage(m, oldMsg, no) == Status::Ok);
        uint8_t before[1024];
        std::memcpy(before, m.bytes, sizeof(before));
        NdefWritePlan plan;
        EXPECT(planNdefWrite(m, n, plan) == Status::Ok && plan.tlvStart == 16 && plan.firstUnit == 4);
        static NdefBackup backup;
        EXPECT(backupNdefArea(m, plan, backup) == Status::Ok && backup.valid);
        EXPECT(writeNdefMessage(m, msg, n) == Status::Ok);
        m.bytes[30] ^= 0xFF;  // a weak cell in the message body
        uint8_t out[400] = {};
        size_t len = 0;
        // The read-back differs (verify is the caller's job); restore anyway.
        const Status rs = readNdefMessage(m, out, sizeof(out), len);
        EXPECT(!(ok(rs) && sameBytes(out, len, msg, n)));
        EXPECT(restoreNdefArea(m, backup) == Status::Ok);
        EXPECT(std::memcmp(before, m.bytes, sizeof(before)) == 0);
        EXPECT(writeNdefMessage(m, msg, n) == Status::Ok);
        m.bytes[plan.lastUnit * 4] ^= 0xFF;
        m.failAfterWrites = m.writes + 1;  // restore stops early
        EXPECT(restoreNdefArea(m, backup) == Status::NoResponse);
        m.failAfterWrites = -1;
        EXPECT(restoreNdefArea(m, backup) == Status::Ok);
        EXPECT(std::memcmp(before, m.bytes, sizeof(before)) == 0);
        m.weakWrites = 100;
        EXPECT(restoreNdefArea(m, backup) == Status::CardError);
        NdefBackup invalid;
        EXPECT(restoreNdefArea(m, invalid) == Status::InvalidArgument);
    }
}

void testNdefT2t() {
    uint8_t msg[128] = {};
    const size_t n = buildTestMessage(msg, sizeof(msg), "https://example.com/04BB", "uid 04BB");
    {
        RigT2T r;
        uint8_t out[512] = {};
        size_t len = 9;
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Ok && len == 0);  // blank tag
        uint8_t p0[16];
        std::memcpy(p0, r.tag.mem, 16);
        EXPECT(r.t2.writeNdef(msg, n) == Status::Ok);
        EXPECT(std::memcmp(p0, r.tag.mem, 16) == 0);  // pages 0-3 untouched
        EXPECT(r.tag.mem[4][0] == 0x03 && size_t(r.tag.mem[4][1]) == n);
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
        // Backup / write / restore through ndefMemory().
        NdefWritePlan plan;
        EXPECT(planNdefWrite(r.t2.ndefMemory(), n, plan) == Status::Ok && plan.firstUnit == 4);
        static NdefBackup backup;
        EXPECT(backupNdefArea(r.t2.ndefMemory(), plan, backup) == Status::Ok);
        uint8_t msg2[128] = {};
        const size_t n2 = buildTestMessage(msg2, sizeof(msg2), "tel:+1", "other");
        EXPECT(r.t2.writeNdef(msg2, n2) == Status::Ok);
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg2, n2));
        EXPECT(restoreNdefArea(r.t2.ndefMemory(), backup) == Status::Ok);
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
        EXPECT(r.crcRxOn() && r.hal.locks == 0);
    }
    {
        RigT2T r;
        r.tag.mem[3][3] = 0x0F;  // read-only CC
        uint8_t out[16] = {};
        size_t len = 0;
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Ok && len == 0);
        EXPECT(r.t2.writeNdef(msg, n) == Status::ReadOnly && r.tag.writes == 0);
    }
    {
        RigT2T r;
        r.tag.mem[3][0] = 0x00;  // no CC
        uint8_t out[16] = {};
        size_t len = 0;
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::NotFormatted);
        EXPECT(r.t2.writeNdef(msg, n) == Status::NotFormatted && r.tag.writes == 0);
    }
    {
        RigT2T r;
        r.tag.mem[3][1] = 0x20;  // mapping version 2.0
        uint8_t out[16] = {};
        size_t len = 0;
        EXPECT(r.t2.readNdef(out, sizeof(out), len) == Status::Unsupported);
    }
}

void testNdefT5t() {
    const uint8_t uid[8] = {0xE0, 0x1D, 0xA1, 0xE8, 0x3F, 0x9F, 0x9E, 0xD9};
    uint8_t msg[128] = {};
    const size_t n = buildTestMessage(msg, sizeof(msg), "https://example.com/E01D", "uid E01D");
    auto rig = [&](Rig15693& r, FakeCardV& c) {
        std::memcpy(c.uidMsbFirst, uid, 8);
        r.hal.vcards[0] = &c;
        EXPECT(r.reader.fieldOn() == Status::Ok);
    };
    {  // no CC: not formatted, nothing written
        Rig15693 r;
        FakeCardV c;
        rig(r, c);
        Type5Tag tag(r.reader, uid);
        uint8_t out[16] = {};
        size_t len = 0;
        EXPECT(tag.readNdef(out, sizeof(out), len) == Status::NotFormatted);
        EXPECT(tag.writeNdef(msg, n) == Status::NotFormatted && c.writes == 0);
    }
    {  // 4-byte CC (MLEN 0Eh = 112 bytes, clamped to 28 x 4 - 4)
        Rig15693 r;
        FakeCardV c;
        rig(r, c);
        const uint8_t cc[4] = {0xE1, 0x40, 0x0E, 0x01};
        std::memcpy(c.blocks[0], cc, 4);
        c.blocks[1][0] = 0x03;  // empty NDEF TLV + Terminator
        c.blocks[1][2] = 0xFE;
        Type5Tag tag(r.reader, uid);
        T5tCapability cap;
        EXPECT(tag.capability(cap) == Status::Ok && cap.ccBytes == 4 && cap.versionMajor == 1);
        EXPECT(cap.areaStart == 4 && cap.areaEnd == 112 && !cap.readOnly);
        uint8_t out[256] = {};
        size_t len = 7;
        EXPECT(tag.readNdef(out, sizeof(out), len) == Status::Ok && len == 0);
        EXPECT(tag.writeNdef(msg, n) == Status::Ok);
        EXPECT(std::memcmp(c.blocks[0], cc, 4) == 0);
        EXPECT(tag.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
        uint8_t tooBig[120] = {};
        EXPECT(tag.writeNdef(tooBig, 107) == Status::BufferTooSmall);
        // Backup / restore.
        NdefWritePlan plan;
        EXPECT(planNdefWrite(tag.ndefMemory(), n, plan) == Status::Ok && plan.firstUnit == 1);
        static NdefBackup backup;
        EXPECT(backupNdefArea(tag.ndefMemory(), plan, backup) == Status::Ok);
        EXPECT(tag.writeNdef(nullptr, 0) == Status::Ok);
        EXPECT(restoreNdefArea(tag.ndefMemory(), backup) == Status::Ok);
        EXPECT(tag.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
        // A locked block in the way: the card refuses, reported as CardError.
        c.locked[3] = true;
        EXPECT(tag.writeNdef(msg, n) == Status::CardError);
    }
    {  // 8-byte CC with 2-byte MLEN
        Rig15693 r;
        FakeCardV c;
        rig(r, c);
        const uint8_t cc[8] = {0xE2, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D};
        std::memcpy(c.blocks[0], cc, 4);
        std::memcpy(c.blocks[1], cc + 4, 4);
        Type5Tag tag(r.reader, uid);
        T5tCapability cap;
        EXPECT(tag.capability(cap) == Status::Ok && cap.ccBytes == 8 && cap.mlenBytes == 104);
        EXPECT(cap.areaStart == 8 && cap.areaEnd == 112);
        EXPECT(tag.writeNdef(msg, n) == Status::Ok);
        EXPECT(std::memcmp(c.blocks[0], cc, 4) == 0 && std::memcmp(c.blocks[1], cc + 4, 4) == 0);
        EXPECT(c.blocks[2][0] == 0x03);
        uint8_t out[256] = {};
        size_t len = 0;
        EXPECT(tag.readNdef(out, sizeof(out), len) == Status::Ok && sameBytes(out, len, msg, n));
    }
    {  // read-only CC, unsupported version
        Rig15693 r;
        FakeCardV c;
        rig(r, c);
        const uint8_t cc[4] = {0xE1, 0x43, 0x0E, 0x00};
        std::memcpy(c.blocks[0], cc, 4);
        Type5Tag tag(r.reader, uid);
        EXPECT(tag.writeNdef(msg, n) == Status::ReadOnly && c.writes == 0);
        c.blocks[0][1] = 0x80;  // major version 2
        Type5Tag tag2(r.reader, uid);
        T5tCapability cap;
        EXPECT(tag2.capability(cap) == Status::Unsupported);
    }
}

void testNdef() {
    testNdefRecords();
    testNdefTlv();
    testNdefT2t();
    testNdefT5t();
}
