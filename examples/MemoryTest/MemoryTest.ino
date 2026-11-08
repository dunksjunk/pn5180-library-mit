// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Non-destructive memory test on NFC Forum Type 2 Tags (ISO/IEC 14443A) and
// ISO/IEC 15693 cards. Reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ 15 (strapping pin
//   MTDO: low at reset only silences the ROM boot log; the PN5180 IRQ idles
//   inactive, see the polarity printed by the Basic example, EEPROM 0x1A, DS Table 51).
//
// Every ~500 ms: a 14443A poll (all cards in the field, Iso14443a::enumerate;
// each untested Type 2 Tag is then re-selected by UID with activateUid;
// garbled or missing answers are retried, only an error that persists is
// printed), then an ISO 15693 poll
// (16-slot inventory), with the RF field off in between and after. Each card
// is tested ONCE until it has left the field (absent for FORGET_AFTER_POLLS
// polls), to save EEPROM write cycles:
//   - a read-only dump line (T2T: pages 0-3 and the CC; 15693: system info)
//   - the LAST page/block of the user area is read, a pattern derived from
//     the UID is written and verified, the original is written back and
//     verified (also after a failed verify),
//   - exactly one result line:
//       T2T MEMTEST OK page 127 uid 04 BB 6A 04 CC 2A 81
//       ISO15693 MEMTEST OK block 27 uid E0 1D A1 E8 3F 9F 9E D9
//     or "... MEMTEST FAIL <step>: <Status> ..." / "... MEMTEST SKIP ...".
// Pages 0-3 of a T2T and anything outside the user area are never written
// (Type2Tag enforces the CC data area). Device/transport errors (and a
// failed RF off) trigger a hardware reset; a failed start-up is retried
// every 5 s.
//
// WARNING: this sketch WRITES to the tags in the field. The original content
// is restored, but use it at your own risk - no warranty, see LICENSE.

#include <SPI.h>

#include <PN5180.h>
#include <PN5180ArduinoHal.h>

// Declared at global scope on purpose: PlatformIO's .ino converter would
// otherwise generate these prototypes inside the anonymous namespace below.
void setup();
void loop();

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
constexpr uint32_t INIT_RETRY_MS = 5000;  // retry after a failed start-up / reset
// A card counts as gone after this many consecutive polls without it (one
// missed inventory slot must not trigger a second write test).
constexpr uint8_t FORGET_AFTER_POLLS = 3;
// SAK b6/b7 clear: Type 2 Tag platform (no ISO/IEC 14443-4).  ISO/IEC 14443-3 6.5.3.4
constexpr uint8_t SAK_NOT_T2T_MASK = 0x60;

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, SPI_CLOCK_HZ);
pn5180::Pn5180 nfc(hal);
pn5180::Iso14443a readerA(nfc);
pn5180::Iso15693 readerV(nfc);
bool ready = false;

// ---- cards already tested ----
struct Seen {
    bool used = false;
    uint8_t uid[10] = {};
    uint8_t uidLen = 0;
    uint8_t misses = 0;
    bool present = false;  // seen in the current poll
};
constexpr size_t MAX_CARDS_A = 4;  // Type A cards handled per poll
Seen seenA[MAX_CARDS_A];
Seen seenV[pn5180::iso15693::kSlots];

bool sameUid(const Seen& s, const uint8_t* uid, uint8_t len) {
    return s.used && s.uidLen == len && memcmp(s.uid, uid, len) == 0;
}

// True if `uid` was tested already; marks it present either way.
bool alreadyTested(Seen* list, size_t count, const uint8_t* uid, uint8_t len) {
    for (size_t i = 0; i < count; ++i) {
        if (sameUid(list[i], uid, len)) {
            list[i].present = true;
            list[i].misses = 0;
            return true;
        }
    }
    return false;
}

// Records `uid` as tested. Uses a free entry, else the entry of the card
// absent longest (not seen in this poll). Returns false if every entry
// belongs to a card in the field: the caller must then NOT test the card,
// or it would be tested (written) again on every poll.
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

// Drops `uid` from the list (its test did not run, e.g. activation failed).
void forget(Seen* list, size_t count, const uint8_t* uid, uint8_t len) {
    for (size_t i = 0; i < count; ++i) {
        if (sameUid(list[i], uid, len)) {
            list[i] = Seen{};
        }
    }
}

// WUPA + SELECT of one known card among HALTed ones, up to 3 tries after a
// garbled or missing answer.  ISO/IEC 14443-3 6.4, 6.5.3.3
pn5180::Status activateKnown(const pn5180::CardA& card, pn5180::CardA& active) {
    pn5180::Status s = pn5180::Status::NoResponse;
    for (uint8_t i = 0; i < 3; ++i) {
        s = readerA.activateUid(card.uid, card.uidLen, active, true);
        if (pn5180::ok(s) || pn5180::requiresRecovery(s)) {
            break;
        }
    }
    return s;
}

// After a completed poll: age the cards that were not seen.
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

void printError(const char* what, pn5180::Status s) {
    Serial.print("ERROR ");
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
}

// "<tag> MEMTEST OK <unit> <n> uid ..." or "<tag> MEMTEST FAIL <step>: ..."
void printResult(const char* tag, const char* unit, unsigned n, const uint8_t* uid, uint8_t uidLen,
                 const pn5180::MemTestResult& r, uint8_t isoError) {
    Serial.print(tag);
    if (pn5180::ok(r.status)) {
        Serial.print(" MEMTEST OK ");
    } else {
        Serial.print(" MEMTEST FAIL ");
        Serial.print(pn5180::memTestStepToString(r.step));
        Serial.print(": ");
        Serial.print(pn5180::statusToString(r.status));
        if (isoError != 0) {
            Serial.print(" (error code ");
            printHex2(isoError);
            Serial.print(')');
        }
        Serial.print(r.restored ? ", original restored, " : ", original NOT restored (");
        if (!r.restored) {
            Serial.print(pn5180::statusToString(r.restoreStatus));
            Serial.print("), ");
        }
    }
    Serial.print(unit);
    Serial.print(' ');
    Serial.print(n);
    Serial.print(" uid");
    printBytes(uid, uidLen);
    Serial.println();
}

// ---- recovery (same policy as ReadUid) ----
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
        Serial.println("Waiting for Type 2 Tags and ISO/IEC 15693 cards...");
    }
    return true;
}

// Reports `s` (NoResponse is silent). Device errors: field off (best
// effort), reader marked not ready, returns false so the poll stops.
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
        printError("RF off", s);  // field state unknown: always reset
        ready = false;
        return false;
    }
    return true;
}

// ---- Type 2 Tag ----
// Transient RF errors: no answer, or a garbled frame (a collision flag on a
// frame from the one selected card is noise as well).
bool transientRf(pn5180::Status s) {
    return s == pn5180::Status::NoResponse || s == pn5180::Status::Collision ||
           s == pn5180::Status::IntegrityError || s == pn5180::Status::ProtocolError ||
           s == pn5180::Status::CardError;
}

// Runs `step`; after a transient RF error the card is re-activated and the
// step repeated, at most READ_ATTEMPTS times in total. A PICC that received
// an erroneous frame leaves ACTIVE (to IDLE, or HALT if woken from HALT),
// so a repeat needs WUPA + SELECT first.  ISO/IEC 14443-3 6.3
constexpr int READ_ATTEMPTS = 3;
template <typename F>
pn5180::Status withReactivation(const pn5180::CardA& card, F step) {
    pn5180::Status s = step();
    for (int i = 1; i < READ_ATTEMPTS && transientRf(s); ++i) {
        pn5180::CardA active;
        const pn5180::Status a = activateKnown(card, active);
        if (pn5180::requiresRecovery(a)) {
            return a;
        }
        if (pn5180::ok(a)) {
            s = step();
        }
    }
    return s;
}

// Returns false if a device error requires a reset.
bool testT2t(const pn5180::CardA& card) {
    pn5180::Type2Tag tag(nfc);
    uint8_t buf[pn5180::t2t::kReadBytes] = {};
    // pages 0-3: UID, lock bytes, CC
    pn5180::Status s = withReactivation(card, [&] { return tag.read(0, buf); });
    if (!pn5180::ok(s)) {
        Serial.print("T2T MEMTEST FAIL dump: ");
        Serial.println(pn5180::statusToString(s));
        return !pn5180::requiresRecovery(s);
    }
    Serial.print("T2T DUMP pages 0-3:");
    printBytes(buf, sizeof(buf));
    Serial.println();

    pn5180::T2tCapability cc;
    s = withReactivation(card, [&] { return tag.capability(cc); });
    if (s == pn5180::Status::NotFormatted) {
        // No valid CC: the user area is unknown, so nothing is written.
        Serial.print("T2T MEMTEST SKIP no valid capability container, uid");
        printBytes(card.uid, card.uidLen);
        Serial.println();
        return true;
    }
    if (!pn5180::ok(s)) {
        Serial.print("T2T MEMTEST FAIL capability container: ");
        Serial.println(pn5180::statusToString(s));
        return !pn5180::requiresRecovery(s);
    }
    Serial.print("T2T CC version ");
    printHex2(cc.version);
    Serial.print(" data area ");
    Serial.print(cc.dataBytes);
    Serial.print(" bytes, pages ");
    Serial.print(cc.firstPage);
    Serial.print('-');
    Serial.print(cc.lastPage);
    Serial.println(cc.readOnly ? ", read-only" : ", writable");
    if (cc.readOnly) {
        Serial.print("T2T MEMTEST SKIP read-only, page ");
        Serial.print(cc.lastPage);
        Serial.print(" uid");
        printBytes(card.uid, card.uidLen);
        Serial.println();
        return true;
    }
    // With the reader and UID, transient RF errors are retried after a re-activation.
    pn5180::Type2TagPage unit(tag, cc.lastPage, readerA, card.uid, card.uidLen);
    const pn5180::MemTestResult r = pn5180::runMemTest(unit, card.uid, card.uidLen);
    printResult("T2T", "page", cc.lastPage, card.uid, card.uidLen, r, 0);
    return !pn5180::requiresRecovery(r.status) && !pn5180::requiresRecovery(r.restoreStatus);
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
        if (alreadyTested(seenA, MAX_CARDS_A, card.uid, card.uidLen) ||
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
        s = activateKnown(card, active);
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

// ---- ISO 15693 ----
bool testV(const pn5180::CardV& card) {
    pn5180::SystemInfo15693 info;
    pn5180::Status s = readerV.getSystemInfo(card.uid, info);
    if (!pn5180::ok(s)) {
        Serial.print("ISO15693 MEMTEST FAIL system info: ");
        Serial.print(pn5180::statusToString(s));
        Serial.print(" uid");
        printBytes(card.uid, sizeof(card.uid));
        Serial.println();
        return !pn5180::requiresRecovery(s);
    }
    Serial.print("ISO15693 INFO uid");
    printBytes(card.uid, sizeof(card.uid));
    Serial.print(" info flags ");
    printHex2(info.infoFlags);
    Serial.print(" dsfid ");
    printHex2(info.dsfid);
    Serial.print(" afi ");
    printHex2(info.afi);
    Serial.print(" blocks ");
    Serial.print(info.blockCount);
    Serial.print(" x ");
    Serial.print(info.blockSize);
    Serial.print(" bytes, ic ref ");
    printHex2(info.icReference);
    Serial.println();
    if ((info.infoFlags & pn5180::iso15693::INFO_MEMORY_SIZE) == 0 || info.blockCount == 0 ||
        info.blockCount > 256) {
        Serial.print("ISO15693 MEMTEST SKIP memory size unknown, uid");
        printBytes(card.uid, sizeof(card.uid));
        Serial.println();
        return true;
    }
    const uint8_t block = static_cast<uint8_t>(info.blockCount - 1);

    uint8_t data[pn5180::iso15693::kMaxBlockBytes] = {};
    uint8_t security = 0;
    s = readerV.readBlock(card.uid, block, data, info.blockSize, true, &security);
    if (pn5180::requiresRecovery(s)) {
        printError("read block", s);
        return false;
    }
    if (pn5180::ok(s)) {
        Serial.print("ISO15693 block ");
        Serial.print(block);
        Serial.print(" security ");
        printHex2(security);
        Serial.print(" data");
        printBytes(data, info.blockSize);
        Serial.println();
        if ((security & pn5180::iso15693::SECURITY_LOCKED) != 0) {
            Serial.print("ISO15693 MEMTEST SKIP block ");
            Serial.print(block);
            Serial.print(" is locked, uid");
            printBytes(card.uid, sizeof(card.uid));
            Serial.println();
            return true;
        }
    } else {
        Serial.print("ISO15693 block security status unavailable: ");
        Serial.println(pn5180::statusToString(s));
    }

    pn5180::Iso15693Block unit(readerV, card.uid, block, info.blockSize);
    const pn5180::MemTestResult r = pn5180::runMemTest(unit, card.uid, sizeof(card.uid));
    printResult("ISO15693", "block", block, card.uid, sizeof(card.uid), r,
                readerV.lastErrorCode());
    return !pn5180::requiresRecovery(r.status) && !pn5180::requiresRecovery(r.restoreStatus);
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
    // Mark every card present first, so aging below is independent of the
    // order in which cards are tested.
    bool fresh[pn5180::iso15693::kSlots] = {};
    for (uint8_t i = 0; i < result.count; ++i) {
        const pn5180::CardV& c = result.cards[i];
        fresh[i] = !alreadyTested(seenV, pn5180::iso15693::kSlots, c.uid, sizeof(c.uid));
    }
    for (uint8_t i = 0; i < result.count; ++i) {
        if (!fresh[i]) {
            continue;
        }
        const pn5180::CardV& c = result.cards[i];
        if (!remember(seenV, pn5180::iso15693::kSlots, c.uid, sizeof(c.uid))) {
            continue;  // table full of present cards: never test unrecorded
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
    Serial.println("PN5180-MIT MemoryTest example");
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
