// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PC unit test of the portable core against a fake HAL (no hardware).
// Build: g++ -std=c++17 -Wall -Wextra -Werror -Isrc -Itests/host tests/host/test_core.cpp
//        src/pn5180/Transport.cpp src/pn5180/Pn5180.cpp src/pn5180/Status.cpp
//        src/pn5180/Iso14443a.cpp src/pn5180/Iso15693.cpp
#include <cstdio>
#include <cstring>

#include "FakeHal.h"
#include "pn5180/Iso14443a.h"
#include "pn5180/Pn5180.h"

using namespace pn5180;
using fake::FakeCardA;
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
    // fieldOn() sets RX_GAIN 2, keeps the other bits, and restores the
    // DS Table 172 SIGPRO_RM_CONFIG default.
    EXPECT(hal.regs[spec::reg::RF_CONTROL_RX] == 0x1Eu);
    EXPECT(hal.regs[spec::reg::SIGPRO_RM_CONFIG] == 0x430DCu);
    CardA out;
    EXPECT(reader.activate(out) == Status::Ok);
    EXPECT(out.uidLen == card.uidLen);
    EXPECT(std::memcmp(out.uid, card.uid, card.uidLen) == 0);
    EXPECT(out.atqa == static_cast<uint16_t>(card.atqa[0] | (card.atqa[1] << 8)));
    EXPECT(out.sak == expectedSak);
    EXPECT(card.selectSawCrc);
    // HLTA: card stays silent -> Ok; REQA then gets no answer, WUPA does.
    EXPECT(reader.halt() == Status::Ok);
    EXPECT(card.halted);
    EXPECT(reader.activate(out) == Status::NoResponse);
    EXPECT(reader.activate(out, true) == Status::Ok);
    EXPECT(reader.fieldOff() == Status::Ok);
    EXPECT(hal.locks == 0);
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
    // Collision in the anticollision answer -> Collision (not resolved).
    {
        FakeCardA c;
        c.collide = true;
        FakeHal hal;
        hal.card = &c;
        Pn5180 nfc(hal);
        Iso14443a reader(nfc);
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(reader.fieldOn() == Status::Ok);
        CardA out;
        EXPECT(reader.activate(out) == Status::Collision);
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
}

}  // namespace

int main() {
    testRecoveryPolicy();
    testCore();
    testIso14443a();
    testIso15693();
    if (failures == 0) {
        std::printf("all tests passed\n");
        return 0;
    }
    std::printf("%d failure(s)\n", failures);
    return 1;
}
