// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PC unit test of the portable core against a fake HAL (no hardware).
// Build: g++ -std=c++17 -Wall -Wextra -Werror -Isrc -Itests/host tests/host/test_core.cpp
//        src/pn5180/Transport.cpp src/pn5180/Pn5180.cpp src/pn5180/Status.cpp
//        src/pn5180/Iso14443a.cpp
#include <cstdio>
#include <cstring>

#include "FakeHal.h"
#include "pn5180/Iso14443a.h"
#include "pn5180/Pn5180.h"

using namespace pn5180;
using fake::FakeCardA;
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
        CardA out;
        EXPECT(reader.activate(out) == Status::IntegrityError);
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
    if (failures == 0) {
        std::printf("all tests passed\n");
        return 0;
    }
    std::printf("%d failure(s)\n", failures);
    return 1;
}
