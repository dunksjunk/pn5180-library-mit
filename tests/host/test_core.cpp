// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// PC unit test of the portable core against a fake HAL (no hardware).
// Build: g++ -std=c++17 -Wall -Wextra -Werror -Isrc tests/host/test_core.cpp
//        src/pn5180/Transport.cpp src/pn5180/Pn5180.cpp src/pn5180/Status.cpp
#include <cstdio>
#include <cstring>

#include "pn5180/Pn5180.h"

using namespace pn5180;

namespace {

int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            ++failures;                                                  \
        }                                                                \
    } while (0)

// Minimal PN5180 model: register file, EEPROM, BUSY pulse per frame.
class FakeHal : public Hal {
public:
    uint32_t regs[0x44] = {};
    uint8_t eeprom[256] = {};
    uint8_t lastCmd[300] = {};
    size_t lastCmdLen = 0;
    bool stuckBusy = false;
    bool raiseErrorOn = false;  // set GENERAL_ERROR on the next write command
    int frames = 0;
    int locks = 0;
    bool inFrame = false;

    Status begin() override { return Status::Ok; }
    Status beginFrame() override {
        inFrame = true;
        return Status::Ok;
    }
    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len, uint8_t filler) override {
        if (tx != nullptr) {  // instruction frame
            std::memcpy(lastCmd, tx, len);
            lastCmdLen = len;
            execute();
        } else {  // response frame
            if (filler != 0xFF) return Status::HalError;
            for (size_t i = 0; i < len; ++i) rx[i] = i < rspLen ? rsp[i] : 0;
        }
        busyLevel = true;  // BUSY rises after the frame (DS 11.4.1)
        return Status::Ok;
    }
    void endFrame() override {
        inFrame = false;
        ++frames;
    }
    void setReset(bool asserted) override {
        if (!asserted) regs[spec::reg::IRQ_STATUS] = spec::irq::IDLE;  // boot done
    }
    bool busy() override {
        if (stuckBusy) return true;
        // BUSY drops once the host has seen it high and released NSS.
        if (busyLevel && !inFrame) {
            busyLevel = false;
            return true;
        }
        return busyLevel;
    }
    uint32_t millis() override { return now++; }
    void delayMs(uint32_t ms) override { now += ms; }
    void delayUs(uint32_t) override {}
    void lock() override { ++locks; }
    void unlock() override { --locks; }

private:
    bool busyLevel = false;
    uint32_t now = 0;
    uint8_t rsp[512] = {};
    size_t rspLen = 0;

    void execute() {
        const uint8_t c = lastCmd[0];
        uint32_t v = 0;
        if (lastCmdLen >= 6) {
            v = lastCmd[2] | (lastCmd[3] << 8) | (lastCmd[4] << 16) | (uint32_t(lastCmd[5]) << 24);
        }
        if (raiseErrorOn && c != spec::cmd::READ_REGISTER) {
            raiseErrorOn = false;
            regs[spec::reg::IRQ_STATUS] |= spec::irq::GENERAL_ERROR;
            return;
        }
        switch (c) {
            case spec::cmd::WRITE_REGISTER:
                if (lastCmd[1] == spec::reg::IRQ_CLEAR) regs[spec::reg::IRQ_STATUS] &= ~v;
                else regs[lastCmd[1]] = v;
                break;
            case spec::cmd::WRITE_REGISTER_OR_MASK: regs[lastCmd[1]] |= v; break;
            case spec::cmd::WRITE_REGISTER_AND_MASK: regs[lastCmd[1]] &= v; break;
            case spec::cmd::READ_REGISTER:
                v = regs[lastCmd[1]];
                for (int i = 0; i < 4; ++i) rsp[i] = uint8_t(v >> (8 * i));
                rspLen = 4;
                break;
            case spec::cmd::READ_EEPROM:
                std::memcpy(rsp, &eeprom[lastCmd[1]], lastCmd[2]);
                rspLen = lastCmd[2];
                break;
            case spec::cmd::WRITE_EEPROM:
                std::memcpy(&eeprom[lastCmd[1]], &lastCmd[2], lastCmdLen - 2);
                break;
            default: break;
        }
    }
};

}  // namespace

int main() {
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

    if (failures == 0) {
        std::printf("all tests passed\n");
        return 0;
    }
    std::printf("%d failure(s)\n", failures);
    return 1;
}
