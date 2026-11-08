// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NDEF demo on NFC Forum Type 2 Tags (ISO/IEC 14443A) and Type 5 Tags
// (ISO/IEC 15693). Reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ 15 (strapping pin
//   MTDO: low at reset only silences the ROM boot log; the PN5180 IRQ idles
//   inactive, see the polarity printed by the Basic example, EEPROM 0x1A, DS Table 51).
//
// Polling and recovery as in MemoryTest (14443A with enumerate + activateUid, then
// a 16-slot ISO 15693 inventory, field off in between). Once per card
// presence:
//   - the existing NDEF message is printed record by record (or "NDEF none" /
//     "NDEF not formatted"),
//   - every unit (page/block) the test write touches is backed up, a test
//     message (one URI + one Text record derived from the UID) is written,
//     read back, decoded and compared,
//   - the backed-up units are written back and verified (also after a failed
//     step), and exactly one result line is printed:
//       T2T NDEF OK uid 04 BB 6A 04 CC 2A 81
//       ISO15693 NDEF OK uid E0 1D A1 E8 3F 9F 9E D9
//       ISO15693 NDEF SKIP not formatted uid E0 ...
//     or "... NDEF FAIL <step>: <Status> ...".
// A card without a Capability Container is never formatted by this demo.

#include <SPI.h>

#include <PN5180.h>
#include <PN5180ArduinoHal.h>

namespace {

constexpr int8_t PIN_SCK = 18;
constexpr int8_t PIN_MISO = 19;
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NSS = 16;
constexpr int8_t PIN_BUSY = 5;
constexpr int8_t PIN_RST = 17;
constexpr int8_t PIN_IRQ = 15;  // -1 = not connected: IRQ_STATUS is polled instead

// 1 MHz: well below the 7 Mbit/s maximum (DS 11.4.1).
constexpr uint32_t SPI_CLOCK_HZ = 1000000u;
constexpr uint32_t POLL_INTERVAL_MS = 500;
constexpr uint32_t INIT_RETRY_MS = 5000;
constexpr uint8_t FORGET_AFTER_POLLS = 3;
// SAK b6/b7 clear: Type 2 Tag platform (no ISO/IEC 14443-4).  ISO/IEC 14443-3 6.5.3.4
constexpr uint8_t SAK_NOT_T2T_MASK = 0x60;
// Transient RF errors: the whole step is repeated (after re-activation for
// a T2T), at most this many attempts.
constexpr int STEP_ATTEMPTS = 3;
constexpr size_t MSG_BYTES = 512;

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, SPI_CLOCK_HZ);
pn5180::Pn5180 nfc(hal);
pn5180::Iso14443a readerA(nfc);
pn5180::Iso15693 readerV(nfc);
bool ready = false;

// Large buffers kept off the loop task's stack.
uint8_t existingMsg[MSG_BYTES];
uint8_t testMsg[MSG_BYTES];
uint8_t readBack[MSG_BYTES];
pn5180::NdefBackup backup;

// ---- cards already handled (same bookkeeping as MemoryTest) ----
struct Seen {
    bool used = false;
    uint8_t uid[10] = {};
    uint8_t uidLen = 0;
    uint8_t misses = 0;
    bool present = false;
};
constexpr size_t MAX_CARDS_A = 4;  // Type A cards handled per poll
Seen seenA[MAX_CARDS_A];
Seen seenV[pn5180::iso15693::kSlots];

bool sameUid(const Seen& s, const uint8_t* uid, uint8_t len) {
    return s.used && s.uidLen == len && memcmp(s.uid, uid, len) == 0;
}

bool alreadyDone(Seen* list, size_t count, const uint8_t* uid, uint8_t len) {
    for (size_t i = 0; i < count; ++i) {
        if (sameUid(list[i], uid, len)) {
            list[i].present = true;
            list[i].misses = 0;
            return true;
        }
    }
    return false;
}

bool remember(Seen* list, size_t count, const uint8_t* uid, uint8_t len) {
    Seen* slot = nullptr;
    for (size_t i = 0; i < count && slot == nullptr; ++i) {
        if (!list[i].used) {
            slot = &list[i];
        }
    }
    for (size_t i = 0; i < count && slot == nullptr; ++i) {
        if (!list[i].present && (slot == nullptr || list[i].misses > slot->misses)) {
            slot = &list[i];
        }
    }
    if (slot == nullptr) {
        return false;
    }
    *slot = Seen{};
    slot->used = true;
    memcpy(slot->uid, uid, len);
    slot->uidLen = len;
    slot->present = true;
    return true;
}

// Drops `uid` from the list (it was not handled, e.g. activation failed).
void forget(Seen* list, size_t count, const uint8_t* uid, uint8_t len) {
    for (size_t i = 0; i < count; ++i) {
        if (sameUid(list[i], uid, len)) {
            list[i] = Seen{};
        }
    }
}

// WUPA + SELECT of one known card among HALTed ones, up to 3 tries after a
// garbled or missing answer.  ISO/IEC 14443-3 6.4, 6.5.3.3
pn5180::Status activateKnown(const uint8_t* uid, uint8_t uidLen, pn5180::CardA& active) {
    pn5180::Status s = pn5180::Status::NoResponse;
    for (uint8_t i = 0; i < 3; ++i) {
        s = readerA.activateUid(uid, uidLen, active, true);
        if (pn5180::ok(s) || pn5180::requiresRecovery(s)) {
            break;
        }
    }
    return s;
}

void agePoll(Seen* list, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (list[i].used && !list[i].present && ++list[i].misses >= FORGET_AFTER_POLLS) {
            list[i] = Seen{};
        }
        list[i].present = false;
    }
}

// ---- printing ----
void printHex2(uint8_t b) {
    if (b < 0x10) {
        Serial.print('0');
    }
    Serial.print(b, HEX);
}

void printBytes(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        Serial.print(' ');
        printHex2(p[i]);
    }
}

void printChars(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        Serial.print(static_cast<char>(p[i]));
    }
}

void printError(const char* what, pn5180::Status s) {
    Serial.print("ERROR ");
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
}

// Prints `msg` record by record.
void dumpMessage(const char* tag, const uint8_t* msg, size_t len) {
    pn5180::NdefParser parser(msg, len);
    pn5180::NdefRecord rec;
    while (!parser.done()) {
        const pn5180::Status s = parser.next(rec);
        if (!pn5180::ok(s)) {
            Serial.print(tag);
            Serial.print(" NDEF message invalid: ");
            Serial.println(pn5180::statusToString(s));
            return;
        }
        Serial.print(tag);
        Serial.print(" NDEF record ");
        Serial.print(parser.count());
        Serial.print(" TNF ");
        Serial.print(rec.tnf);
        Serial.print(" type \"");
        printChars(rec.type, rec.typeLength);
        Serial.print("\" ");
        pn5180::NdefUri uri;
        pn5180::NdefText text;
        if (rec.isWellKnown(pn5180::ndef::RTD_URI) && pn5180::ok(pn5180::decodeNdefUri(rec, uri))) {
            Serial.print("URI ");
            Serial.print(uri.prefix);
            printChars(uri.rest, uri.restLength);
        } else if (rec.isWellKnown(pn5180::ndef::RTD_TEXT) &&
                   pn5180::ok(pn5180::decodeNdefText(rec, text))) {
            Serial.print("Text [");
            printChars(text.lang, text.langLength);
            Serial.print("] ");
            if (text.utf16) {
                Serial.print("(UTF-16)");
            } else {
                printChars(text.text, text.textLength);
            }
        } else {
            Serial.print("payload");
            printBytes(rec.payload, rec.payloadLength < 32 ? rec.payloadLength : 32);
            if (rec.payloadLength > 32) {
                Serial.print(" ...");
            }
        }
        Serial.println();
    }
}

// ---- test message ----
char testUri[64];
char testText[64];

void makeTestMessage(const uint8_t* uid, size_t uidLen, size_t& len) {
    char hex[2 * 10 + 1] = {};
    for (size_t i = 0; i < uidLen && i < 10; ++i) {
        snprintf(hex + 2 * i, 3, "%02X", uid[i]);
    }
    // A per-run counter makes every write differ, so a stale value cannot pass.
    static uint16_t run = 0;
    ++run;
    snprintf(testUri, sizeof(testUri), "https://example.com/pn5180/%s/%u", hex, run);
    snprintf(testText, sizeof(testText), "PN5180-MIT test %s #%u", hex, run);
    pn5180::NdefBuilder b(testMsg, sizeof(testMsg));
    len = 0;
    if (pn5180::ok(b.addUri(testUri)) && pn5180::ok(b.addText(testText, "en"))) {
        len = b.size();
    }
}

// Read-back must hold exactly the two test records.
pn5180::Status checkReadBack(const uint8_t* msg, size_t len) {
    pn5180::NdefParser p(msg, len);
    pn5180::NdefRecord r;
    pn5180::NdefUri u;
    pn5180::NdefText t;
    pn5180::Status s = p.next(r);
    if (!pn5180::ok(s)) return s;
    if (!pn5180::ok(pn5180::decodeNdefUri(r, u)) || !pn5180::ndefUriEquals(u, testUri)) {
        return pn5180::Status::CardError;
    }
    s = p.next(r);
    if (!pn5180::ok(s)) return s;
    if (!pn5180::ok(pn5180::decodeNdefText(r, t)) || t.utf16 || t.langLength != 2 ||
        memcmp(t.lang, "en", 2) != 0 || t.textLength != strlen(testText) ||
        memcmp(t.text, testText, t.textLength) != 0 || !p.done()) {
        return pn5180::Status::CardError;
    }
    return pn5180::Status::Ok;
}

bool transient(pn5180::Status s) {
    return s == pn5180::Status::NoResponse || s == pn5180::Status::Collision ||
           s == pn5180::Status::IntegrityError || s == pn5180::Status::ProtocolError;
}

// Card-type specific pieces of the test.
struct Card {
    const char* tag;  // "T2T" / "ISO15693"
    const uint8_t* uid;
    uint8_t uidLen;
    pn5180::NdefMemory& mem;
    // read / write via the tag object, recover after a transient error
    pn5180::Status (*readNdef)(void* ctx, uint8_t* buf, size_t cap, size_t& len);
    pn5180::Status (*writeNdef)(void* ctx, const uint8_t* msg, size_t len);
    pn5180::Status (*recover)(void* ctx);
    void* ctx;
};

template <typename F>
pn5180::Status withRetry(Card& c, F step) {
    pn5180::Status s = step();
    for (int i = 1; i < STEP_ATTEMPTS && transient(s); ++i) {
        const pn5180::Status rc = c.recover(c.ctx);
        if (!pn5180::ok(rc) && !transient(rc)) {
            return rc;
        }
        if (pn5180::ok(rc)) {
            s = step();
        }
    }
    return s;
}

void printUidEnd(const Card& c) {
    Serial.print(" uid");
    printBytes(c.uid, c.uidLen);
    Serial.println();
}

// Runs the whole demo on one card. Returns the status that decides about a
// chip reset (worst of test and restore).
pn5180::Status runNdefTest(Card& c) {
    size_t len = 0;
    pn5180::Status s = withRetry(c, [&] { return c.readNdef(c.ctx, existingMsg, MSG_BYTES, len); });
    if (s == pn5180::Status::NotFormatted) {
        Serial.print(c.tag);
        Serial.println(" NDEF not formatted");
        Serial.print(c.tag);
        Serial.print(" NDEF SKIP not formatted");
        printUidEnd(c);
        return pn5180::Status::Ok;
    }
    if (!pn5180::ok(s)) {
        Serial.print(c.tag);
        Serial.print(" NDEF FAIL read existing: ");
        Serial.print(pn5180::statusToString(s));
        printUidEnd(c);
        return s;
    }
    if (len == 0) {
        Serial.print(c.tag);
        Serial.println(" NDEF none");
    } else {
        Serial.print(c.tag);
        Serial.print(" NDEF message ");
        Serial.print(len);
        Serial.println(" bytes");
        dumpMessage(c.tag, existingMsg, len);
    }
    if (c.mem.readOnly()) {
        Serial.print(c.tag);
        Serial.print(" NDEF SKIP read-only");
        printUidEnd(c);
        return pn5180::Status::Ok;
    }

    size_t msgLen = 0;
    makeTestMessage(c.uid, c.uidLen, msgLen);
    // The counter restarts at every boot, so a message left by an interrupted
    // earlier run can equal this one; take the next number so a write that
    // silently fails cannot pass the read-back check.
    if (msgLen != 0 && msgLen == len && memcmp(testMsg, existingMsg, len) == 0) {
        makeTestMessage(c.uid, c.uidLen, msgLen);
    }
    pn5180::NdefWritePlan plan;
    const char* step = "plan";
    s = msgLen == 0 ? pn5180::Status::BufferTooSmall : pn5180::planNdefWrite(c.mem, msgLen, plan);
    if (pn5180::ok(s)) {
        step = "backup";
        s = withRetry(c, [&] { return pn5180::backupNdefArea(c.mem, plan, backup); });
    }
    if (!pn5180::ok(s)) {  // nothing written yet
        Serial.print(c.tag);
        Serial.print(" NDEF FAIL ");
        Serial.print(step);
        Serial.print(": ");
        Serial.print(pn5180::statusToString(s));
        printUidEnd(c);
        return s;
    }
    Serial.print(c.tag);
    Serial.print(" NDEF backup units ");
    Serial.print(backup.firstUnit);
    Serial.print('-');
    Serial.print(backup.firstUnit + backup.units - 1);
    Serial.print(", writing ");
    Serial.print(msgLen);
    Serial.println(" bytes");

    step = "write";
    s = withRetry(c, [&] { return c.writeNdef(c.ctx, testMsg, msgLen); });
    if (pn5180::ok(s)) {
        step = "read back";
        size_t backLen = 0;
        s = withRetry(c, [&] { return c.readNdef(c.ctx, readBack, MSG_BYTES, backLen); });
        if (pn5180::ok(s)) {
            step = "verify";
            s = checkReadBack(readBack, backLen);
            if (pn5180::ok(s)) {
                dumpMessage(c.tag, readBack, backLen);
            }
        }
    }
    // Restore on every path once a write was attempted.
    const pn5180::Status rs = withRetry(c, [&] { return pn5180::restoreNdefArea(c.mem, backup); });
    if (pn5180::ok(s) && !pn5180::ok(rs)) {
        s = rs;
        step = "restore";
    }
    Serial.print(c.tag);
    if (pn5180::ok(s)) {
        Serial.print(" NDEF OK");
    } else {
        Serial.print(" NDEF FAIL ");
        Serial.print(step);
        Serial.print(": ");
        Serial.print(pn5180::statusToString(s));
        Serial.print(pn5180::ok(rs) ? ", original restored" : ", original NOT restored (");
        if (!pn5180::ok(rs)) {
            Serial.print(pn5180::statusToString(rs));
            Serial.print(')');
        }
    }
    printUidEnd(c);
    return pn5180::requiresRecovery(rs) ? rs : s;
}

// ---- recovery (same policy as MemoryTest) ----
bool initReader() {
    static bool begun = false;
    static bool announced = false;
    if (!begun) {
        const pn5180::Status s = nfc.begin();
        if (!pn5180::ok(s)) {
            printError("begin", s);
            return false;
        }
        begun = true;
    }
    const pn5180::Status s = nfc.hardReset();
    if (!pn5180::ok(s)) {
        printError("hard reset", s);
        return false;
    }
    ready = true;
    if (!announced) {
        announced = true;
        Serial.println("Waiting for NFC Forum Type 2 / Type 5 Tags...");
    }
    return true;
}

bool check(const char* what, pn5180::Status s) {
    if (pn5180::ok(s) || s == pn5180::Status::NoResponse) {
        return true;
    }
    printError(what, s);
    if (pn5180::requiresRecovery(s)) {
        (void)nfc.rfOff();
        ready = false;
        return false;
    }
    return true;
}

bool fieldOffBetweenPolls() {
    const pn5180::Status s = nfc.rfOff();
    if (!pn5180::ok(s)) {
        printError("RF off", s);
        ready = false;
        return false;
    }
    return true;
}

// ---- Type 2 Tag ----
struct T2Ctx {
    pn5180::Type2Tag* tag;
    const pn5180::CardA* card;
};

pn5180::Status t2Read(void* ctx, uint8_t* buf, size_t cap, size_t& len) {
    return static_cast<T2Ctx*>(ctx)->tag->readNdef(buf, cap, len);
}
pn5180::Status t2Write(void* ctx, const uint8_t* msg, size_t len) {
    return static_cast<T2Ctx*>(ctx)->tag->writeNdef(msg, len);
}
// A transmission error sends the PICC back to IDLE: field reset, WUPA
// activation and a check that the same card answered.  ISO/IEC 14443-3 6.3
pn5180::Status t2Recover(void* ctx) {
    const T2Ctx* t = static_cast<T2Ctx*>(ctx);
    pn5180::Status s = readerA.fieldOff();
    if (pn5180::ok(s)) {
        s = readerA.fieldOn();
    }
    if (!pn5180::ok(s)) {
        return s;
    }
    // SELECT by UID: another card in the field cannot answer instead.
    pn5180::CardA again;
    return activateKnown(t->card->uid, t->card->uidLen, again);
}

bool testT2t(const pn5180::CardA& card) {
    pn5180::Type2Tag tag(nfc);
    T2Ctx ctx{&tag, &card};
    Card c{"T2T", card.uid, card.uidLen, tag.ndefMemory(), t2Read, t2Write, t2Recover, &ctx};
    pn5180::T2tCapability cc;
    const pn5180::Status s = withRetry(c, [&] { return tag.capability(cc); });
    if (pn5180::ok(s)) {
        Serial.print("T2T CC");
        printBytes(cc.raw, sizeof(cc.raw));
        Serial.print(", data area ");
        Serial.print(cc.dataBytes);
        Serial.print(" bytes, pages ");
        Serial.print(cc.firstPage);
        Serial.print('-');
        Serial.print(cc.lastPage);
        Serial.println(cc.readOnly ? ", read-only" : ", writable");
    } else if (s != pn5180::Status::NotFormatted) {
        Serial.print("T2T NDEF FAIL capability container: ");
        Serial.print(pn5180::statusToString(s));
        printUidEnd(c);
        return !pn5180::requiresRecovery(s);
    }
    return !pn5180::requiresRecovery(runNdefTest(c));
}

void pollA() {
    pn5180::Status s = readerA.fieldOn();
    if (!pn5180::ok(s)) {
        if (check("RF on (14443A)", s)) {
            (void)fieldOffBetweenPolls();
        }
        return;
    }
    // All cards in the field (each left HALTed); a garbled or missing answer
    // is retried (bounded, see ActivateRetry).
    pn5180::CardA cards[MAX_CARDS_A];
    size_t count = 0;
    s = readerA.enumerate(cards, MAX_CARDS_A, count);
    if (s == pn5180::Status::BufferTooSmall) {
        // More cards answered than fit: only the first MAX_CARDS_A are handled,
        // and the same ones win every poll, so say so instead of skipping silently.
        Serial.print("WARNING: more than ");
        Serial.print(static_cast<unsigned>(MAX_CARDS_A));
        Serial.println(" Type A cards in the field; the others are not handled (raise MAX_CARDS_A)");
    } else if (!check("enumerate", s)) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        const pn5180::CardA& card = cards[i];
        if (alreadyDone(seenA, MAX_CARDS_A, card.uid, card.uidLen) ||
            !remember(seenA, MAX_CARDS_A, card.uid, card.uidLen)) {
            continue;
        }
        if ((card.sak & SAK_NOT_T2T_MASK) != 0) {
            Serial.print("14443A card is not a Type 2 Tag (SAK ");
            printHex2(card.sak);
            Serial.print("), uid");
            printBytes(card.uid, card.uidLen);
            Serial.println();
            continue;
        }
        // WUPA wakes the HALTed cards, SELECT with the UID picks this one
        // (the others drop back to IDLE).  ISO/IEC 14443-3 6.4, 6.5.3.3
        pn5180::CardA active;
        s = activateKnown(card.uid, card.uidLen, active);
        if (!pn5180::ok(s)) {
            if (!check("activate", s)) {
                return;
            }
            forget(seenA, MAX_CARDS_A, card.uid, card.uidLen);  // try again next poll
            continue;
        }
        if (!testT2t(card)) {
            (void)nfc.rfOff();
            ready = false;
            return;
        }
        if (!check("HLTA", readerA.halt())) {
            return;
        }
    }
    agePoll(seenA, MAX_CARDS_A);
    (void)fieldOffBetweenPolls();
}

// ---- ISO 15693 / Type 5 Tag ----
pn5180::Status t5Read(void* ctx, uint8_t* buf, size_t cap, size_t& len) {
    return static_cast<pn5180::Type5Tag*>(ctx)->readNdef(buf, cap, len);
}
pn5180::Status t5Write(void* ctx, const uint8_t* msg, size_t len) {
    return static_cast<pn5180::Type5Tag*>(ctx)->writeNdef(msg, len);
}
// An ISO 15693 card stays selected-less and addressable: just repeat.
pn5180::Status t5Recover(void*) { return pn5180::Status::Ok; }

bool testV(const pn5180::CardV& card) {
    pn5180::Type5Tag tag(readerV, card.uid);
    Card c{"ISO15693", card.uid, static_cast<uint8_t>(sizeof(card.uid)), tag.ndefMemory(),
           t5Read, t5Write, t5Recover, &tag};
    pn5180::T5tCapability cc;
    const pn5180::Status s = withRetry(c, [&] { return tag.capability(cc); });
    if (pn5180::ok(s)) {
        Serial.print("ISO15693 CC");
        printBytes(cc.raw, cc.ccBytes);
        Serial.print(", T5T area ");
        Serial.print(cc.areaStart);
        Serial.print('-');
        Serial.print(cc.areaEnd - 1);
        Serial.print(", ");
        Serial.print(tag.systemInfo().blockCount);
        Serial.print(" blocks x ");
        Serial.print(tag.systemInfo().blockSize);
        Serial.println(cc.readOnly ? " bytes, read-only" : " bytes, writable");
    } else if (s != pn5180::Status::NotFormatted) {
        Serial.print("ISO15693 NDEF FAIL capability container: ");
        Serial.print(pn5180::statusToString(s));
        printUidEnd(c);
        return !pn5180::requiresRecovery(s);
    }
    return !pn5180::requiresRecovery(runNdefTest(c));
}

void pollV() {
    pn5180::Status s = readerV.fieldOn();
    if (!pn5180::ok(s)) {
        if (check("RF on (15693)", s)) {
            (void)fieldOffBetweenPolls();
        }
        return;
    }
    pn5180::InventoryResult result;
    s = readerV.inventory16(result);
    if (!check("inventory", s)) {
        return;
    }
    bool fresh[pn5180::iso15693::kSlots] = {};
    for (uint8_t i = 0; i < result.count; ++i) {
        const pn5180::CardV& c = result.cards[i];
        fresh[i] = !alreadyDone(seenV, pn5180::iso15693::kSlots, c.uid, sizeof(c.uid));
    }
    for (uint8_t i = 0; i < result.count; ++i) {
        if (!fresh[i]) {
            continue;
        }
        const pn5180::CardV& c = result.cards[i];
        if (!remember(seenV, pn5180::iso15693::kSlots, c.uid, sizeof(c.uid))) {
            continue;
        }
        if (!testV(c)) {
            (void)nfc.rfOff();
            ready = false;
            return;
        }
    }
    agePoll(seenV, pn5180::iso15693::kSlots);
    (void)fieldOffBetweenPolls();
}

uint32_t nextActionMs = 0;

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("PN5180-MIT NdefDemo example");
    nextActionMs = millis();
}

void loop() {
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - nextActionMs) < 0) {
        delay(1);
        return;
    }
    if (!ready) {
        nextActionMs = initReader() ? now : now + INIT_RETRY_MS;
        return;
    }
    pollA();
    if (ready) {
        pollV();
    }
    nextActionMs = ready ? now + POLL_INTERVAL_MS : now;
}
