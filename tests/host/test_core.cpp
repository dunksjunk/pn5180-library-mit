// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PC unit test of the portable core against a fake HAL (no hardware).
// Build: g++ -std=c++17 -Wall -Wextra -Werror -Isrc -Itests/host tests/host/test_core.cpp
//        src/pn5180/Transport.cpp src/pn5180/Pn5180.cpp src/pn5180/Status.cpp
//        src/pn5180/Iso14443a.cpp src/pn5180/Iso15693.cpp src/pn5180/Type2Tag.cpp
//        src/pn5180/MemTest.cpp
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include "FakeHal.h"
#include "pn5180/Iso14443a.h"
#include "pn5180/Lpcd.h"
#include "pn5180/MemTest.h"
#include "pn5180/Ndef.h"
#include "pn5180/NdefTlv.h"
#include "pn5180/Type5Tag.h"
#include "pn5180/Pn5180.h"
#include "pn5180/Type2Tag.h"

using namespace pn5180;
using fake::FakeCardA;
using fake::FakeT2T;
using fake::FakeCardV;
using fake::FakeHal;

namespace {

int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            ++failures;                                                  \
        }                                                                \
    } while (0)

void testCore() {
    FakeHal hal;
    // FW 4.1, product 3.A, EEPROM 0.93 (byte order per DS Table 51)
    const uint8_t versions[6] = {0x0A, 0x03, 0x01, 0x04, 0x93, 0x00};
    std::memcpy(&hal.eeprom[spec::eeprom::PRODUCT_VERSION], versions, sizeof(versions));

    Pn5180 nfc(hal);
    uint32_t v = 0;
    EXPECT(nfc.readRegister(spec::reg::SYSTEM_CONFIG, v) == Status::NotInitialized);
    EXPECT(nfc.begin() == Status::Ok);
    EXPECT(nfc.hardReset() == Status::Ok);
    EXPECT((hal.regs[spec::reg::IRQ_STATUS] & spec::irq::IDLE) == 0);  // IDLE cleared

    Versions ver;
    EXPECT(nfc.readVersions(ver) == Status::Ok);
    EXPECT(ver.firmware.major == 4 && ver.firmware.minor == 1);
    EXPECT(ver.product.major == 3 && ver.product.minor == 0x0A);

    // Register round trip, little endian, masks.
    nfc.setCommandErrorCheck(false);  // so lastCmd is the write itself
    EXPECT(nfc.writeRegister(spec::reg::TX_CONFIG, 0x12345678u) == Status::Ok);
    EXPECT(hal.lastCmdLen == 6 && hal.lastCmd[0] == 0x00 && hal.lastCmd[1] == 0x18 &&
           hal.lastCmd[2] == 0x78 && hal.lastCmd[5] == 0x12);
    nfc.setCommandErrorCheck(true);
    EXPECT(nfc.writeRegisterOrMask(spec::reg::TX_CONFIG, 0x80000000u) == Status::Ok);
    EXPECT(nfc.writeRegisterAndMask(spec::reg::TX_CONFIG, 0xFFFFFF00u) == Status::Ok);
    EXPECT(nfc.readRegister(spec::reg::TX_CONFIG, v) == Status::Ok);
    EXPECT(v == 0x92345600u);

    // Validation.
    EXPECT(nfc.writeRegister(spec::reg::IRQ_STATUS, 0) == Status::ReadOnly);
    EXPECT(nfc.writeRegister(0x06, 0) == Status::InvalidArgument);
    uint8_t buf[8] = {};
    EXPECT(nfc.readEeprom(250, buf, 6) == Status::InvalidArgument);  // beyond address 254
    EXPECT(nfc.readEeprom(249, buf, 6) == Status::Ok);
    const uint8_t data[2] = {1, 2};
    EXPECT(nfc.writeEeprom(0x20, data, 2) == Status::WriteProtected);
    nfc.setEepromWriteEnabled(true);
    EXPECT(nfc.writeEeprom(0x15, data, 2) == Status::WriteProtected);
    EXPECT(nfc.writeEeprom(0x20, data, 2) == Status::Ok);
    EXPECT(hal.eeprom[0x21] == 2);
    EXPECT(nfc.loadRfConfig(0x1D, 0x80) == Status::InvalidArgument);
    EXPECT(nfc.loadRfConfig(0x00, 0x80) == Status::Ok);
    EXPECT(nfc.rfOn(0x04) == Status::InvalidArgument);
    EXPECT(nfc.sendData(data, 2, 8) == Status::InvalidArgument);

    // Chip exception surfaces as ChipError and the flag gets cleared.
    hal.raiseErrorOn = true;
    EXPECT(nfc.rfOn() == Status::ChipError);
    EXPECT((hal.regs[spec::reg::IRQ_STATUS] & spec::irq::GENERAL_ERROR) == 0);
    EXPECT(nfc.rfOff() == Status::Ok);

    // BUSY stuck high -> timeout, never hangs; lock always released.
    hal.stuckBusy = true;
    EXPECT(nfc.rfOff() == Status::BusyTimeout);
    hal.stuckBusy = false;
    EXPECT(hal.locks == 0);
    EXPECT(!hal.inFrame);
}

// Activates `card` and checks UID / ATQA / SAK.
void expectActivation(FakeCardA& card, uint8_t expectedSak) {
    FakeHal hal;
    hal.card = &card;
    Pn5180 nfc(hal);
    Iso14443a reader(nfc);
    EXPECT(nfc.begin() == Status::Ok);
    // Values the reference module loads (ARC override, DS 11.8.6).
    hal.regs[spec::reg::RF_CONTROL_RX] = 0x1Fu;
    hal.regs[spec::reg::SIGPRO_RM_CONFIG] = 0x440DCu;
    EXPECT(reader.fieldOn() == Status::Ok);
    // fieldOn() sets RX_GAIN 2, keeps the other bits, and writes the DS
    // Table 172 SIGPRO_RM_CONFIG default with MIN_LEVEL 5 (measured, Spec.h).
    EXPECT(hal.regs[spec::reg::RF_CONTROL_RX] == 0x1Eu);
    EXPECT(hal.regs[spec::reg::SIGPRO_RM_CONFIG] == 0x450DCu);
    CardA out;
    EXPECT(reader.activate(out) == Status::Ok);
    EXPECT(out.uidLen == card.uidLen);
    EXPECT(std::memcmp(out.uid, card.uid, card.uidLen) == 0);
    EXPECT(out.atqa == static_cast<uint16_t>(card.atqa[0] | (card.atqa[1] << 8)));
    EXPECT(out.sak == expectedSak);
    EXPECT(card.selectSawCrc);
    // HLTA: card stays silent -> Ok; REQA then gets no answer, WUPA does.
    EXPECT(reader.halt() == Status::Ok);
    EXPECT(card.halted());
    EXPECT(reader.activate(out) == Status::NoResponse);
    EXPECT(reader.activate(out, true) == Status::Ok);
    EXPECT(reader.fieldOff() == Status::Ok);
    EXPECT(hal.locks == 0);
}

// Iso14443a::activateWithRetry with injected transient / persistent faults.
struct RigA {
    FakeCardA card;
    FakeHal hal;
    Pn5180 nfc{hal};
    Iso14443a reader{nfc};
    RigA() {
        const uint8_t u7[7] = {0x04, 0xBB, 0x6A, 0x04, 0xCC, 0x2A, 0x81};
        std::memcpy(card.uid, u7, 7);
        card.uidLen = 7;
        hal.card = &card;
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(reader.fieldOn() == Status::Ok);
    }
};

void testActivateRetry() {
    {  // garbled ATQA once -> second attempt succeeds with a fresh REQA
        RigA r;
        r.hal.garbleA = 1;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::Ok);
        EXPECT(out.uidLen == 7 && out.uid[1] == 0xBB);
        EXPECT(r.hal.requestsA == 2);
        EXPECT(r.hal.rfOffs == 0);  // no field reset by default
    }
    {  // garbled CL1 anticollision answer (2nd frame) once: restart with REQA
        RigA r;
        r.hal.faultSkipA = 1;
        r.hal.garbleA = 1;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::Ok);
        EXPECT(r.hal.requestsA == 2);
        EXPECT(out.uidLen == 7);
    }
    {  // SELECT CL2 (5th frame) unanswered once
        RigA r;
        r.hal.faultSkipA = 4;
        r.hal.muteA = 1;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::Ok);
        EXPECT(r.hal.requestsA == 2);
        EXPECT(r.hal.framesA == 10);  // 5 failed-attempt frames + 5 good ones
    }
    {  // no answer once -> retried
        RigA r;
        r.hal.muteA = 1;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::Ok);
        EXPECT(r.hal.requestsA == 2);
    }
    {  // persistent garbage -> bounded: exactly `attempts` activations, last status
        RigA r;
        r.hal.garbleA = 1000;
        CardA out;
        ActivateRetry p;
        p.attempts = 4;
        // a collision flag on a 1-byte "ATQA" is noise, not a collision
        EXPECT(r.reader.activateWithRetry(out, false, p) == Status::IntegrityError);
        EXPECT(r.hal.requestsA == 4);
    }
    {  // persistent silence -> NoResponse after the default 3 attempts
        RigA r;
        r.hal.muteA = 1000;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::NoResponse);
        EXPECT(r.hal.requestsA == 3);
    }
    {  // attempts clamped to 1..kMaxAttempts
        RigA r;
        r.hal.muteA = 1000;
        CardA out;
        ActivateRetry p;
        p.attempts = 0;
        EXPECT(r.reader.activateWithRetry(out, false, p) == Status::NoResponse);
        EXPECT(r.hal.requestsA == 1);
        p.attempts = 200;
        r.hal.requestsA = 0;
        EXPECT(r.reader.activateWithRetry(out, false, p) == Status::NoResponse);
        EXPECT(r.hal.requestsA == ActivateRetry::kMaxAttempts);
    }
    {  // resetField: one RF_OFF + RF_ON per retry, field left on
        RigA r;
        r.hal.garbleA = 2;
        CardA out;
        ActivateRetry p;
        p.resetField = true;
        EXPECT(r.reader.activateWithRetry(out, false, p) == Status::Ok);
        EXPECT(r.hal.rfOffs == 2);
        EXPECT((r.hal.regs[spec::reg::RF_STATUS] & spec::rfstatus::TX_RF_STATUS) != 0);
    }
    {  // REQA retry does not wake a HALTed card; resetField does (POWER-OFF -> IDLE)
        RigA r;
        CardA out;
        EXPECT(r.reader.activate(out) == Status::Ok);
        EXPECT(r.reader.halt() == Status::Ok);
        EXPECT(r.reader.activateWithRetry(out) == Status::NoResponse);
        ActivateRetry p;
        p.resetField = true;
        EXPECT(r.reader.activateWithRetry(out, false, p) == Status::Ok);
    }
    {  // device error is not retried
        RigA r;
        r.hal.errorAtSendData = 1;
        CardA out;
        EXPECT(r.reader.activateWithRetry(out) == Status::ChipError);
        EXPECT(r.hal.requestsA == 0);
    }
}

// Several Type A cards: bit-wise collisions resolved by anticollision.
struct MultiA {
    FakeCardA c[4];
    FakeHal hal;
    Pn5180 nfc{hal};
    Iso14443a reader{nfc};
    explicit MultiA(int n) {
        for (int i = 0; i < n; ++i) hal.cardsA[i] = &c[i];
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(reader.fieldOn() == Status::Ok);
    }
    void set(int i, std::initializer_list<uint8_t> uid, uint8_t sak = 0x00) {
        c[i].uidLen = 0;
        for (uint8_t b : uid) c[i].uid[c[i].uidLen++] = b;
        c[i].sak = sak;
    }
    bool alignClear() const {
        return (hal.regs[spec::reg::CRC_RX_CONFIG] & spec::kCrcRxBitAlignMask) == 0;
    }
    int badFrames() const { return c[0].badFrames + c[1].badFrames + c[2].badFrames + c[3].badFrames; }
};

bool sameUid(const CardA& a, const FakeCardA& f) {
    return a.uidLen == f.uidLen && std::memcmp(a.uid, f.uid, f.uidLen) == 0;
}

// enumerate() must list exactly the cards c[0..n), each once.
void expectEnumerated(MultiA& m, int n, bool wakeup = false) {
    CardA list[4];
    size_t count = 99;
    EXPECT(m.reader.enumerate(list, 4, count, wakeup) == Status::Ok);
    EXPECT(count == size_t(n));
    for (int i = 0; i < n; ++i) {
        int hits = 0;
        for (size_t k = 0; k < count; ++k) hits += sameUid(list[k], m.c[i]) ? 1 : 0;
        EXPECT(hits == 1);
        EXPECT(m.c[i].halted());
    }
    EXPECT(m.alignClear());
    EXPECT(m.badFrames() == 0);
}

void testMultiCardA() {
    {  // differ in the first UID bit: policy picks the card with a 1 there
        MultiA m(2);
        m.set(0, {0x10, 0x22, 0x33, 0x44});
        m.set(1, {0x11, 0x22, 0x33, 0x44}, 0x08);
        CardA out;
        EXPECT(m.reader.activate(out) == Status::Ok);
        EXPECT(sameUid(out, m.c[1]) && out.sak == 0x08);
        EXPECT(m.c[1].state == FakeCardA::Active && m.c[0].state == FakeCardA::Idle);
        EXPECT(m.alignClear() && m.badFrames() == 0);
        EXPECT(m.reader.halt() == Status::Ok);
        EXPECT(m.reader.activate(out) == Status::Ok);  // the other one, alone now
        EXPECT(sameUid(out, m.c[0]));
    }
    {  // differ only in the last UID CLn bit (bit 31), and so in BCC too
        MultiA m(2);
        m.set(0, {0x5A, 0xA5, 0x0F, 0x01});
        m.set(1, {0x5A, 0xA5, 0x0F, 0x81});
        CardA out;
        EXPECT(m.reader.activate(out) == Status::Ok);
        EXPECT(sameUid(out, m.c[1]));
        expectEnumerated(m, 2, true);
    }
    {  // same UID CLn, only one BCC differs (corrupt): never Ok with a wrong card
        MultiA m(2);
        m.set(0, {0x01, 0x02, 0x03, 0x04});
        m.set(1, {0x01, 0x02, 0x03, 0x04});
        m.c[1].corruptBcc = true;
        CardA out;
        const Status s = m.reader.activate(out);
        EXPECT(s == Status::IntegrityError || (s == Status::Ok && sameUid(out, m.c[0])));
        EXPECT(m.alignClear());
    }
    {  // 3 cards, 7-byte UIDs sharing CL1 collide only at cascade level 2
        MultiA m(3);
        m.set(0, {0x04, 0xBB, 0x6A, 0x04, 0xCC, 0x2A, 0x81});
        m.set(1, {0x04, 0xBB, 0x6A, 0x05, 0xCC, 0x2A, 0x81});
        m.set(2, {0x04, 0xBB, 0x6A, 0x04, 0xCC, 0x2A, 0x01});
        expectEnumerated(m, 3);
    }
    {  // 4/7/10-byte mix with collisions at level 1 (CT vs UID byte)
        MultiA m(4);
        m.set(0, {0x89, 0x01, 0x02, 0x03});  // differs from CT 88h in bit 0 only
        m.set(1, {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66});
        m.set(2, {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x67, 0x77, 0x88, 0x99}, 0x20);
        m.set(3, {0x08, 0x11, 0x22, 0x33});
        expectEnumerated(m, 4);
        // all HALTed: REQA finds nothing, WUPA all of them again
        expectEnumerated(m, 0);
        expectEnumerated(m, 4, true);
    }
    {  // more cards than fit: BufferTooSmall, the first `max` listed
        MultiA m(3);
        m.set(0, {0x01, 0x00, 0x00, 0x00});
        m.set(1, {0x02, 0x00, 0x00, 0x00});
        m.set(2, {0x04, 0x00, 0x00, 0x00});
        CardA list[2];
        size_t count = 0;
        EXPECT(m.reader.enumerate(list, 2, count) == Status::BufferTooSmall);
        EXPECT(count == 2);
        EXPECT(m.c[0].halted() + m.c[1].halted() + m.c[2].halted() == 2);
        CardA none[1];
        EXPECT(m.reader.enumerate(none, 0, count) == Status::BufferTooSmall && count == 0);
        EXPECT(m.reader.enumerate(nullptr, 1, count) == Status::InvalidArgument);
    }
    {  // noise (garbled frame) during enumerate: retried, all cards found
        MultiA m(2);
        m.set(0, {0x10, 0x20, 0x30, 0x40});
        m.set(1, {0x10, 0x20, 0x31, 0x40});
        m.hal.faultSkipA = 1;  // REQA fine, first ANTICOLLISION answer is noise
        m.hal.garbleA = 1;
        expectEnumerated(m, 2);
    }
    {  // noise alone (collision flag, wrong length) is an error, not a card
        MultiA m(1);
        m.set(0, {0x10, 0x20, 0x30, 0x40});
        m.hal.faultSkipA = 1;
        m.hal.garbleA = 1;
        CardA out;
        EXPECT(m.reader.activate(out) == Status::IntegrityError);
        EXPECT(m.alignClear());
        m.hal.garbleA = 100;
        CardA list[2];
        size_t count = 9;
        EXPECT(m.reader.enumerate(list, 2, count) == Status::IntegrityError && count == 0);
    }
    {  // selectUid / activateUid: select a known card among several
        MultiA m(2);
        m.set(0, {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66});
        m.set(1, {0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x67}, 0x00);
        uint16_t atqa = 0;
        EXPECT(m.reader.request(atqa) == Status::Ok);
        CardA out;
        EXPECT(m.reader.selectUid(m.c[0].uid, 7, out) == Status::Ok);
        EXPECT(sameUid(out, m.c[0]));
        EXPECT(m.c[0].state == FakeCardA::Active && m.c[1].state == FakeCardA::Idle);
        EXPECT(m.reader.halt() == Status::Ok);
        // HALTed card: REQA + selectUid finds nobody with that UID
        EXPECT(m.reader.activateUid(m.c[0].uid, 7, out, false) == Status::NoResponse);
        EXPECT(m.reader.activateUid(m.c[0].uid, 7, out, true) == Status::Ok);  // WUPA
        EXPECT(sameUid(out, m.c[0]) && m.c[0].state == FakeCardA::Active);
        EXPECT(m.reader.selectUid(m.c[0].uid, 5, out) == Status::InvalidArgument);
        EXPECT(m.reader.selectUid(nullptr, 4, out) == Status::InvalidArgument);
        const uint8_t unknown[4] = {9, 9, 9, 9};
        EXPECT(m.reader.request(atqa, true) == Status::Ok);
        EXPECT(m.reader.selectUid(unknown, 4, out) == Status::NoResponse);
        EXPECT(m.badFrames() == 0);
    }
    {  // ATQA collision (different ATQAs) is not an error
        MultiA m(2);
        m.set(0, {0x01, 0x02, 0x03, 0x04});
        m.set(1, {0x81, 0x02, 0x03, 0x04});
        m.c[1].atqa[0] = 0x04;
        uint16_t atqa = 0;
        EXPECT(m.reader.request(atqa) == Status::Ok);
        expectEnumerated(m, 2, true);
    }
}

void testIso14443a() {
    // 4-, 7- and 10-byte UIDs (cascade levels 1..3).  ISO/IEC 14443-3 6.5.4
    FakeCardA single;
    const uint8_t u4[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    std::memcpy(single.uid, u4, 4);
    single.uidLen = 4;
    single.atqa[0] = 0x04;
    single.sak = 0x08;
    expectActivation(single, 0x08);

    FakeCardA dbl;
    const uint8_t u7[7] = {0x04, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6};
    std::memcpy(dbl.uid, u7, 7);
    dbl.uidLen = 7;
    dbl.sak = 0x00;
    expectActivation(dbl, 0x00);
    EXPECT(dbl.levelsSelected == 2);

    FakeCardA triple;
    const uint8_t u10[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::memcpy(triple.uid, u10, 10);
    triple.uidLen = 10;
    triple.sak = 0x20;
    expectActivation(triple, 0x20);
    EXPECT(triple.levelsSelected == 3);

    // No card -> NoResponse, never hangs.
    {
        FakeHal hal;
        Pn5180 nfc(hal);
        Iso14443a reader(nfc);
        EXPECT(nfc.begin() == Status::Ok);
        CardA out;
        EXPECT(reader.activate(out) == Status::NoResponse);
        EXPECT((hal.regs[spec::reg::SYSTEM_CONFIG] & spec::kSystemConfigCommandMask) ==
               spec::kCommandIdle);
    }
    // Wrong BCC -> IntegrityError.
    {
        FakeCardA c;
        c.corruptBcc = true;
        FakeHal hal;
        hal.card = &c;
        Pn5180 nfc(hal);
        Iso14443a reader(nfc);
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(reader.fieldOn() == Status::Ok);
        CardA out;
        EXPECT(reader.activate(out) == Status::IntegrityError);
    }
    testActivateRetry();
    testMultiCardA();
}

// Card from the M2 bring-up: single-slot INVENTORY answered
// 00 00 C7 16 95 46 2A A1 1D E0 (flags, DSFID, UID LSB first).
void setGroundTruthUid(FakeCardV& c) {
    const uint8_t u[8] = {0xE0, 0x1D, 0xA1, 0x2A, 0x46, 0x95, 0x16, 0xC7};
    std::memcpy(c.uidMsbFirst, u, 8);
}

struct Rig15693 {
    FakeHal hal;
    Pn5180 nfc{hal};
    Iso15693 reader{nfc};
    Rig15693() { EXPECT(nfc.begin() == Status::Ok); }
};

void testIso15693() {
    // CRC of ISO/IEC 13239: check value of "123456789" is 906Eh.
    EXPECT(fake::crc13239(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0x906E);

    // fieldOn: LOAD_RF_CONFIG 0x0D/0x8D, field on, CRC on both ways.
    {
        Rig15693 r;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        EXPECT(r.hal.rfTxConfig == 0x0D && r.hal.rfRxConfig == 0x8D);
        EXPECT((r.hal.regs[spec::reg::RF_STATUS] & spec::rfstatus::TX_RF_STATUS) != 0);
        EXPECT((r.hal.regs[spec::reg::CRC_TX_CONFIG] & spec::kCrcTxEnable) != 0);
        EXPECT((r.hal.regs[spec::reg::CRC_RX_CONFIG] & spec::kCrcRxEnable) != 0);
    }
    // Single-slot inventory: UID MSB first, DSFID.
    {
        Rig15693 r;
        FakeCardV c;
        setGroundTruthUid(c);
        c.dsfid = 0x5A;
        r.hal.vcards[0] = &c;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        CardV out;
        EXPECT(r.reader.inventory(out) == Status::Ok);
        EXPECT(std::memcmp(out.uid, c.uidMsbFirst, 8) == 0);
        EXPECT(out.uid[0] == 0xE0 && out.uid[7] == 0xC7);
        EXPECT(out.dsfid == 0x5A);
        // Field off -> no answer.
        EXPECT(r.reader.fieldOff() == Status::Ok);
        EXPECT(r.reader.inventory(out) == Status::NoResponse);
        EXPECT(r.hal.locks == 0);
    }
    // Regressions the fake must catch: TX CRC off -> card ignores the frame;
    // RX CRC off -> CRC left in the buffer -> CardError; 14443 config loaded
    // -> nothing; bad CRC from the card -> IntegrityError.
    {
        Rig15693 r;
        FakeCardV c;
        r.hal.vcards[0] = &c;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        CardV out;
        EXPECT(r.nfc.setCrc(false, true) == Status::Ok);
        EXPECT(r.reader.inventory(out) == Status::NoResponse);
        EXPECT(c.requests == 0);
        EXPECT(r.nfc.setCrc(true, false) == Status::Ok);
        EXPECT(r.reader.inventory(out) == Status::CardError);
        EXPECT(r.nfc.setCrc(true, true) == Status::Ok);
        c.corruptCrc = true;
        EXPECT(r.reader.inventory(out) == Status::IntegrityError);
        c.corruptCrc = false;
        EXPECT(r.reader.inventory(out) == Status::Ok);
        // Frame without SOF (start symbol cleared) is ignored by cards.
        r.hal.regs[spec::reg::TX_CONFIG] &= ~spec::txconfig::START_SYMBOL_MASK;
        EXPECT(r.reader.inventory(out) == Status::NoResponse);
        Iso14443a a(r.nfc);
        EXPECT(a.fieldOn() == Status::Ok);
        EXPECT(r.reader.inventory(out) == Status::NoResponse);
    }
    // Two cards in single-slot mode -> Collision.
    {
        Rig15693 r;
        FakeCardV c1;
        FakeCardV c2;
        setGroundTruthUid(c2);
        r.hal.vcards[0] = &c1;
        r.hal.vcards[1] = &c2;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        CardV out;
        EXPECT(r.reader.inventory(out) == Status::Collision);
    }
    // 16 slots: cards in slots 7 and 8 read; TX_CONFIG restored; 15 EOFs.
    {
        Rig15693 r;
        FakeCardV c1;  // ...78 -> slot 8
        FakeCardV c2;  // ...C7 -> slot 7
        setGroundTruthUid(c2);
        r.hal.vcards[0] = &c1;
        r.hal.vcards[1] = &c2;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        const uint32_t txBefore = r.hal.regs[spec::reg::TX_CONFIG];
        InventoryResult res;
        EXPECT(r.reader.inventory16(res) == Status::Ok);
        EXPECT(res.count == 2);
        EXPECT(res.collisionSlots == 0);
        EXPECT(std::memcmp(res.cards[0].uid, c2.uidMsbFirst, 8) == 0);  // slot 7 first
        EXPECT(std::memcmp(res.cards[1].uid, c1.uidMsbFirst, 8) == 0);
        EXPECT(r.hal.eofFrames == 15);
        EXPECT(r.hal.regs[spec::reg::TX_CONFIG] == txBefore);
        // A plain request works again afterwards (SOF + data restored).
        CardV out;
        r.hal.vcards[1] = nullptr;
        EXPECT(r.reader.inventory(out) == Status::Ok);
    }
    // 16 slots with two cards in slot 7 and one in slot 8.
    {
        Rig15693 r;
        FakeCardV a;
        FakeCardV b;
        FakeCardV c;  // slot 8
        setGroundTruthUid(a);
        setGroundTruthUid(b);
        b.uidMsbFirst[7] = 0x97;  // also slot 7
        r.hal.vcards[0] = &a;
        r.hal.vcards[1] = &b;
        r.hal.vcards[2] = &c;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        InventoryResult res;
        EXPECT(r.reader.inventory16(res) == Status::Ok);
        EXPECT(res.count == 1);
        EXPECT(res.collisionSlots == 0x0080);  // slot 7
        EXPECT(std::memcmp(res.cards[0].uid, c.uidMsbFirst, 8) == 0);
        // Without the slot-8 card: nothing clean -> Collision.
        r.hal.vcards[2] = nullptr;
        EXPECT(r.reader.inventory16(res) == Status::Collision);
        EXPECT(res.count == 0 && res.collisionSlots == 0x0080);
    }
    // 16 slots, no card -> NoResponse; field off -> NoResponse.
    {
        Rig15693 r;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        const uint32_t txBefore = r.hal.regs[spec::reg::TX_CONFIG];
        InventoryResult res;
        EXPECT(r.reader.inventory16(res) == Status::NoResponse);
        EXPECT(r.hal.regs[spec::reg::TX_CONFIG] == txBefore);
        FakeCardV c;
        r.hal.vcards[0] = &c;
        EXPECT(r.reader.fieldOff() == Status::Ok);
        EXPECT(r.reader.inventory16(res) == Status::NoResponse);
    }
    // Device error in the middle of the slots: TX_CONFIG still restored.
    {
        Rig15693 r;
        FakeCardV c;  // slot 8
        r.hal.vcards[0] = &c;
        EXPECT(r.reader.fieldOn() == Status::Ok);
        const uint32_t txBefore = r.hal.regs[spec::reg::TX_CONFIG];
        r.hal.errorAtSendData = 3;  // GENERAL_ERROR on the 3rd SEND_DATA
        InventoryResult res;
        EXPECT(r.reader.inventory16(res) == Status::ChipError);
        EXPECT(r.hal.regs[spec::reg::TX_CONFIG] == txBefore);
        EXPECT(r.hal.locks == 0);
    }
}

struct RigT2T {
    FakeHal hal;
    FakeT2T tag;
    Pn5180 nfc{hal};
    Iso14443a reader{nfc};
    Type2Tag t2{nfc};
    RigT2T() {
        hal.t2t = &tag;
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(reader.fieldOn() == Status::Ok);
    }
    bool crcRxOn() const { return (hal.regs[spec::reg::CRC_RX_CONFIG] & spec::kCrcRxEnable) != 0; }
};

void testType2Tag() {
    // CRC_A check value of "123456789" (ISO/IEC 14443-3 Annex B): BF05h.
    EXPECT(fake::crcA(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0xBF05);

    // Read pages 0-3; CC decode -> data area pages 4..127.
    {
        RigT2T r;
        uint8_t buf[16] = {};
        EXPECT(r.t2.read(0, buf) == Status::Ok);
        EXPECT(buf[0] == 0x04 && buf[12] == 0xE1 && buf[14] == 0x3E);
        T2tCapability cc;
        EXPECT(r.t2.capability(cc) == Status::Ok);
        EXPECT(cc.dataBytes == 496 && cc.firstPage == 4 && cc.lastPage == 127 && !cc.readOnly);
        EXPECT(cc.version == 0x10);
        EXPECT(r.tag.ignored == 0);
    }
    // Write round trip; CRC RX restored; ACK seen. read() sets CRC on in both
    // directions itself, so a CRC left off by another layer cannot break it
    // (the fake would return 18 bytes with RX CRC off, or ignore the frame
    // with TX CRC off).
    {
        RigT2T r;
        const uint8_t d[4] = {0xDE, 0xAD, 0xBE, 0xEF};
        EXPECT(r.t2.write(127, d) == Status::Ok);
        EXPECT(r.t2.lastAckNak() == 0x0A);
        EXPECT(r.crcRxOn());
        EXPECT(std::memcmp(r.tag.mem[127], d, 4) == 0);
        uint8_t buf[16] = {};
        EXPECT(r.t2.read(127, buf) == Status::Ok);
        EXPECT(std::memcmp(buf, d, 4) == 0);
        EXPECT(r.nfc.setCrc(true, false) == Status::Ok);
        EXPECT(r.t2.read(127, buf) == Status::Ok);  // read() turns RX CRC back on
        EXPECT(r.crcRxOn());
        EXPECT(r.nfc.setCrc(false, true) == Status::Ok);  // TX CRC off: tag would ignore
        const int before = r.tag.ignored;
        EXPECT(r.t2.read(127, buf) == Status::Ok);  // read() turns TX CRC back on
        EXPECT(r.tag.ignored == before);
        // Fake check: with RX CRC off the CRC_A stays in the buffer (18 bytes),
        // which Type2Tag::read() would reject as CardError.
        EXPECT(r.nfc.setCrc(true, false) == Status::Ok);
        const uint8_t rd[2] = {t2t::CMD_READ, 127};
        uint8_t raw[24] = {};
        size_t n = 0;
        uint8_t bits = 0;
        EXPECT(r.nfc.transceive(rd, 2, 0, raw, sizeof(raw), n, bits, 5) == Status::Ok);
        EXPECT(n == 18 && bits == 0);
        EXPECT(r.hal.locks == 0);
    }
    // WRITE waits for a slow ACK: the timeout handed to transceive covers the
    // measured worst case (12.8 ms) and is what kWriteTimeoutMs says.
    {
        RigT2T r;
        const uint8_t d[4] = {0x01, 0x02, 0x03, 0x04};
        r.tag.writeAckDelayMs = 13;
        EXPECT(r.t2.write(10, d) == Status::Ok);
        EXPECT(r.crcRxOn());
        r.tag.writeAckDelayMs = t2t::kWriteTimeoutMs - 2;
        EXPECT(r.t2.write(10, d) == Status::Ok);
        // Clearly beyond: the host timer starts after SEND_DATA (a few ticks late).
        r.tag.writeAckDelayMs = 2 * t2t::kWriteTimeoutMs;
        EXPECT(r.t2.write(10, d) == Status::NoResponse);
        EXPECT(r.crcRxOn());
    }
    // NAK -> CardError, CRC RX restored.
    {
        RigT2T r;
        r.tag.nakWrites = true;
        const uint8_t d[4] = {1, 2, 3, 4};
        EXPECT(r.t2.write(10, d) == Status::CardError);
        EXPECT(r.t2.lastAckNak() == 0x00);
        EXPECT(r.crcRxOn());
        EXPECT(r.tag.writes == 0);
    }
    // No answer to WRITE -> NoResponse, CRC RX still restored.
    {
        RigT2T r;
        T2tCapability cc;
        EXPECT(r.t2.capability(cc) == Status::Ok);
        r.hal.t2t = nullptr;  // tag leaves the field
        const uint8_t d[4] = {1, 2, 3, 4};
        EXPECT(r.t2.write(10, d) == Status::NoResponse);
        EXPECT(r.crcRxOn());
    }
    // Pages 0-3 and beyond the data area refused; read-only CC refuses writes.
    {
        RigT2T r;
        const uint8_t d[4] = {1, 2, 3, 4};
        EXPECT(r.t2.write(3, d) == Status::InvalidArgument);
        EXPECT(r.t2.write(0, d) == Status::InvalidArgument);
        EXPECT(r.t2.write(128, d) == Status::InvalidArgument);
        EXPECT(r.tag.writes == 0);
    }
    {
        RigT2T r;
        r.tag.mem[3][3] = 0x0F;  // CC3 write access: none
        const uint8_t d[4] = {1, 2, 3, 4};
        EXPECT(r.t2.write(10, d) == Status::ReadOnly);
        EXPECT(r.tag.writes == 0);
        r.tag.mem[3][0] = 0x00;  // no CC magic
        Type2Tag fresh(r.nfc);
        T2tCapability cc;
        EXPECT(fresh.capability(cc) == Status::NotFormatted);
        EXPECT(fresh.write(10, d) == Status::NotFormatted);
        EXPECT(r.tag.writes == 0);
    }
    // MemTest: pass; pattern differs from the original; original restored.
    {
        RigT2T r;
        const uint8_t orig[4] = {0x11, 0x22, 0x33, 0x44};
        std::memcpy(r.tag.mem[127], orig, 4);
        Type2TagPage unit(r.t2, 127);
        const uint8_t uid[7] = {0x04, 0xBB, 0x6A, 0x04, 0xCC, 0x2A, 0x81};
        const MemTestResult m = runMemTest(unit, uid, sizeof(uid));
        EXPECT(m.status == Status::Ok && m.step == MemTestStep::None && m.restored);
        EXPECT(r.tag.writes == 2);
        EXPECT(std::memcmp(r.tag.mem[127], orig, 4) == 0);
        for (int i = 0; i < 4; ++i) EXPECT(m.pattern[i] != orig[i]);
    }
    // Failed verify (weak cell) -> original still written back and verified.
    {
        RigT2T r;
        const uint8_t orig[4] = {0x11, 0x22, 0x33, 0x44};
        std::memcpy(r.tag.mem[127], orig, 4);
        r.tag.weakWrites = 1;
        Type2TagPage unit(r.t2, 127);
        const uint8_t uid[7] = {0x04, 0xBB, 0x6A, 0x04, 0xCC, 0x2A, 0x81};
        const MemTestResult m = runMemTest(unit, uid, sizeof(uid));
        EXPECT(m.status == Status::CardError && m.step == MemTestStep::VerifyPattern);
        EXPECT(m.restored && m.restoreStatus == Status::Ok);
        EXPECT(std::memcmp(r.tag.mem[127], orig, 4) == 0);
    }
}

void testIso15693Memory() {
    const uint8_t uid[8] = {0xE0, 0x1D, 0xA1, 0xE8, 0x3F, 0x9F, 0x9E, 0xD9};
    auto setup = [&](Rig15693& r, FakeCardV& c, FakeCardV& other) {
        std::memcpy(c.uidMsbFirst, uid, 8);
        r.hal.vcards[0] = &c;
        r.hal.vcards[1] = &other;  // a second card must stay silent when not addressed
        EXPECT(r.reader.fieldOn() == Status::Ok);
    };
    // System info, read/write round trip, security status.
    {
        Rig15693 r;
        FakeCardV c, other;
        setup(r, c, other);
        SystemInfo15693 info;
        EXPECT(r.reader.getSystemInfo(uid, info) == Status::Ok);
        EXPECT(info.infoFlags == 0x0F && info.blockCount == 28 && info.blockSize == 4);
        EXPECT(info.icReference == 0x01 && std::memcmp(info.uid, uid, 8) == 0);
        const uint8_t d[4] = {9, 8, 7, 6};
        EXPECT(r.reader.writeBlock(uid, 27, d, 4) == Status::Ok);
        EXPECT(std::memcmp(c.blocks[27], d, 4) == 0 && other.writes == 0);
        uint8_t out[4] = {};
        uint8_t sec = 0xFF;
        EXPECT(r.reader.readBlock(uid, 27, out, 4, true, &sec) == Status::Ok);
        EXPECT(sec == 0x00 && std::memcmp(out, d, 4) == 0);
        EXPECT(r.reader.readBlock(uid, 27, out, 4) == Status::Ok);
        EXPECT(r.reader.lastErrorCode() == 0);
        // Wrong block size -> CardError (length mismatch), bad args refused.
        EXPECT(r.reader.readBlock(uid, 27, out, 8) == Status::CardError);
        EXPECT(r.reader.readBlock(uid, 27, out, 4, true, nullptr) == Status::InvalidArgument);
        EXPECT(r.reader.writeBlock(uid, 27, d, 0) == Status::InvalidArgument);
        // Timeouts: write includes the 20 ms programming window.
        static_assert(iso15693::kWriteBlockTimeoutMs > 20, "write window missing");
        // T2T WRITE ACK measured at < 12.8 ms on the test bed (Type2Tag.h).
        static_assert(t2t::kWriteTimeoutMs >= 13 + 1, "T2T write timeout below measurement");
        static_assert(iso15693::kReadBlockTimeoutMs < 20, "read timeout too long");
    }
    // Locked block: security status reports it; write -> error 12h.
    {
        Rig15693 r;
        FakeCardV c, other;
        setup(r, c, other);
        c.locked[27] = true;
        uint8_t out[4] = {};
        uint8_t sec = 0;
        EXPECT(r.reader.readBlock(uid, 27, out, 4, true, &sec) == Status::Ok);
        EXPECT((sec & iso15693::SECURITY_LOCKED) != 0);
        const uint8_t d[4] = {1, 2, 3, 4};
        EXPECT(r.reader.writeBlock(uid, 27, d, 4) == Status::CardError);
        EXPECT(r.reader.lastErrorCode() == 0x12);
        EXPECT(c.writes == 0);
        // Block out of range -> error 10h; next success clears the code.
        EXPECT(r.reader.readBlock(uid, 200, out, 4) == Status::CardError);
        EXPECT(r.reader.lastErrorCode() == 0x10);
        EXPECT(r.reader.readBlock(uid, 0, out, 4) == Status::Ok);
        EXPECT(r.reader.lastErrorCode() == 0);
    }
    // Regressions: other UID -> NoResponse; RX CRC off -> CardError; TX CRC
    // off -> card ignores the frame; bad card CRC -> IntegrityError.
    {
        Rig15693 r;
        FakeCardV c, other;
        setup(r, c, other);
        SystemInfo15693 info;
        uint8_t wrong[8];
        std::memcpy(wrong, uid, 8);
        wrong[7] ^= 0x01;
        EXPECT(r.reader.getSystemInfo(wrong, info) == Status::NoResponse);
        EXPECT(r.nfc.setCrc(true, false) == Status::Ok);
        EXPECT(r.reader.getSystemInfo(uid, info) == Status::CardError);
        EXPECT(r.nfc.setCrc(false, true) == Status::Ok);
        EXPECT(r.reader.getSystemInfo(uid, info) == Status::NoResponse);
        EXPECT(r.nfc.setCrc(true, true) == Status::Ok);
        c.corruptCrc = true;
        EXPECT(r.reader.getSystemInfo(uid, info) == Status::IntegrityError);
    }
    // MemTest on a 15693 block: pass and original restored.
    {
        Rig15693 r;
        FakeCardV c, other;
        setup(r, c, other);
        const uint8_t orig[4] = {0xAA, 0xBB, 0xCC, 0xDD};
        std::memcpy(c.blocks[27], orig, 4);
        Iso15693Block unit(r.reader, uid, 27, 4);
        const MemTestResult m = runMemTest(unit, uid, 8);
        EXPECT(m.status == Status::Ok && m.restored);
        EXPECT(c.writes == 2 && std::memcmp(c.blocks[27], orig, 4) == 0);
        // Locked: pattern write fails, restore also fails, nothing changed.
        c.locked[27] = true;
        const MemTestResult l = runMemTest(unit, uid, 8);
        EXPECT(l.status == Status::CardError && l.step == MemTestStep::WritePattern);
        EXPECT(!l.restored && std::memcmp(c.blocks[27], orig, 4) == 0);
    }
}

// Scripted memory unit: fails the next `failReads`/`failWrites` calls with
// `err` (a failed write still stores the data, like a lost ACK).
struct FlakyUnit final : MemoryUnit {
    uint8_t mem[4] = {0x11, 0x22, 0x33, 0x44};
    int failReads = 0, failWrites = 0, recovers = 0, writes = 0;
    Status err = Status::Collision;
    Status recoverResult = Status::Ok;
    size_t size() const override { return 4; }
    Status read(uint8_t* out) override {
        if (failReads > 0) { --failReads; return err; }
        std::memcpy(out, mem, 4);
        return Status::Ok;
    }
    Status write(const uint8_t* in) override {
        std::memcpy(mem, in, 4);
        ++writes;
        if (failWrites > 0) { --failWrites; return err; }
        return Status::Ok;
    }
    Status recover() override { ++recovers; return recoverResult; }
};

void testMemTestRetry() {
    const uint8_t orig[4] = {0x11, 0x22, 0x33, 0x44};
    const uint8_t seed[2] = {0x04, 0xBB};
    // Lost ACK on the restore write: recovered and retried, original back.
    {
        FlakyUnit u;
        const MemTestResult m0 = runMemTest(u, seed, 2);  // baseline: 2 writes
        EXPECT(m0.status == Status::Ok && u.recovers == 0 && u.writes == 2);
    }
    {
        FlakyUnit u;
        // pattern write ok, then the restore write's answer is garbled once
        struct R final : MemoryUnit {
            FlakyUnit& f; int n = 0;
            explicit R(FlakyUnit& x) : f(x) {}
            size_t size() const override { return 4; }
            Status read(uint8_t* o) override { return f.read(o); }
            Status write(const uint8_t* in) override {
                if (++n == 2) f.failWrites = 1;
                return f.write(in);
            }
            Status recover() override { return f.recover(); }
        } r(u);
        const MemTestResult m = runMemTest(r, seed, 2);
        EXPECT(m.status == Status::Ok && m.restored);
        EXPECT(u.recovers == 1 && std::memcmp(u.mem, orig, 4) == 0);
    }
    // Garbled first read of the original: retried, test passes.
    {
        FlakyUnit u;
        u.failReads = 1;
        u.err = Status::NoResponse;
        const MemTestResult m = runMemTest(u, seed, 2);
        EXPECT(m.status == Status::Ok && u.recovers == 1);
        EXPECT(std::memcmp(m.original, orig, 4) == 0);
    }
    // Persistent transient error: gives up after kMemTestAttempts.
    {
        FlakyUnit u;
        u.failWrites = 100;
        const MemTestResult m = runMemTest(u, seed, 2);
        EXPECT(m.status == Status::Collision && m.step == MemTestStep::WritePattern);
        EXPECT(!m.restored);
        EXPECT(u.writes == 2 * kMemTestAttempts);
    }
    // Card errors (NAK, mismatch) are not retried.
    {
        FlakyUnit u;
        u.failWrites = 1;
        u.err = Status::CardError;
        const MemTestResult m = runMemTest(u, seed, 2);
        EXPECT(m.status == Status::CardError && u.recovers == 0);
        EXPECT(m.restored && std::memcmp(u.mem, orig, 4) == 0);
    }
    // recover() finds another card: stop retrying, report it.
    {
        FlakyUnit u;
        u.failWrites = 1;
        u.recoverResult = Status::CardError;
        const MemTestResult m = runMemTest(u, seed, 2);
        EXPECT(m.status == Status::CardError && m.step == MemTestStep::WritePattern);
        EXPECT(u.recovers >= 1);
    }
}

#include "NdefTests.inc"
#include "IrqLpcdTests.inc"

void testRecoveryPolicy() {
    EXPECT(requiresRecovery(Status::BusyTimeout));
    EXPECT(requiresRecovery(Status::IrqTimeout));
    EXPECT(requiresRecovery(Status::HalError));
    EXPECT(requiresRecovery(Status::ChipError));
    EXPECT(requiresRecovery(Status::NotInitialized));
    EXPECT(requiresRecovery(Status::RfFieldTimeout));
    EXPECT(!requiresRecovery(Status::Ok));
    EXPECT(!requiresRecovery(Status::NoResponse));
    EXPECT(!requiresRecovery(Status::Collision));
    EXPECT(!requiresRecovery(Status::IntegrityError));
    EXPECT(!requiresRecovery(Status::ProtocolError));
    EXPECT(!requiresRecovery(Status::CardError));
    EXPECT(!requiresRecovery(Status::RfActiveError));
    EXPECT(!requiresRecovery(Status::InvalidArgument));
    EXPECT(requiresRecovery(Status::IrqPinError));
    EXPECT(!requiresRecovery(Status::LowPowerActive));
}

}  // namespace

int main() {
    testMemTestRetry();
    testRecoveryPolicy();
    testCore();
    testIso14443a();
    testIso15693();
    testType2Tag();
    testIso15693Memory();
    testNdef();
    testIrqPin();
    testIrqReview();
    testLpcd();
    if (failures == 0) {
        std::printf("all tests passed\n");
        return 0;
    }
    std::printf("%d failure(s)\n", failures);
    return 1;
}
