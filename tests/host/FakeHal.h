// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Fake HAL for PC unit tests: a minimal PN5180 model (register file, EEPROM,
// BUSY pulse per frame, transceive with an optional simulated Type A card).
#pragma once

#include <cstdint>
#include <cstring>

#include "pn5180/Hal.h"
#include "pn5180/Iso14443a.h"
#include "pn5180/Spec.h"

namespace fake {

using namespace pn5180;

// Simulated ISO/IEC 14443-3 Type A card (single card, no real anticollision).
struct FakeCardA {
    uint8_t uid[10] = {};
    uint8_t uidLen = 4;
    uint8_t atqa[2] = {0x44, 0x00};
    uint8_t sak = 0x00;
    bool halted = false;
    bool collide = false;      // report a collision in the anticollision answer
    bool corruptBcc = false;
    bool selectSawCrc = true;  // false if a SELECT arrived with TX CRC off
    int levelsSelected = 0;

    // Returns true and fills `out` if the card answers.
    bool respond(const uint8_t* in, size_t len, uint8_t lastBits, bool txCrc, uint8_t* out,
                 size_t& outLen, uint32_t& extraStatus) {
        outLen = 0;
        extraStatus = 0;
        if (len == 1 && lastBits == 7 && (in[0] == iso14443a::REQA || in[0] == iso14443a::WUPA)) {
            if (halted && in[0] == iso14443a::REQA) return false;
            halted = false;
            out[0] = atqa[0];
            out[1] = atqa[1];
            outLen = 2;
            return true;
        }
        if (len == 2 && in[0] == iso14443a::HLTA_0 && in[1] == iso14443a::HLTA_1) {
            halted = true;
            return false;
        }
        if (len >= 2 && (in[0] == iso14443a::SEL_CL1 || in[0] == iso14443a::SEL_CL2 ||
                         in[0] == iso14443a::SEL_CL3)) {
            const int level = (in[0] - iso14443a::SEL_CL1) / 2;
            uint8_t cl[5];
            if (!levelBytes(level, cl)) return false;
            if (in[1] == iso14443a::NVB_ANTICOLLISION && len == 2) {
                std::memcpy(out, cl, 5);
                if (corruptBcc) out[4] ^= 0xFF;
                outLen = 5;
                if (collide) extraStatus = spec::rxstatus::COLLISION_DETECTED;
                return true;
            }
            if (in[1] == iso14443a::NVB_SELECT && len == 7 && std::memcmp(&in[2], cl, 5) == 0) {
                if (!txCrc) selectSawCrc = false;
                const bool last = level == lastLevel();
                out[0] = last ? sak : iso14443a::SAK_CASCADE_BIT;
                outLen = 1;
                levelsSelected = level + 1;
                return true;
            }
        }
        return false;
    }

private:
    int lastLevel() const { return uidLen == 4 ? 0 : (uidLen == 7 ? 1 : 2); }

    bool levelBytes(int level, uint8_t cl[5]) const {
        if (level > lastLevel()) return false;
        const bool last = level == lastLevel();
        const uint8_t* p = &uid[level * 3];
        if (last) {
            std::memcpy(cl, p, 4);
        } else {
            cl[0] = iso14443a::CASCADE_TAG;
            std::memcpy(&cl[1], p, 3);
        }
        cl[4] = static_cast<uint8_t>(cl[0] ^ cl[1] ^ cl[2] ^ cl[3]);
        return true;
    }
};

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
    FakeCardA* card = nullptr;  // card in the field (nullptr = none)

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
    uint8_t rfRx[512] = {};

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
            case spec::cmd::SEND_DATA: sendData(); break;
            case spec::cmd::RF_ON:  // DS Table 37, Table 103
                regs[spec::reg::RF_STATUS] |= spec::rfstatus::TX_RF_STATUS;
                regs[spec::reg::IRQ_STATUS] |= spec::irq::TX_RFON;
                break;
            case spec::cmd::RF_OFF:  // DS Table 38, Table 103
                regs[spec::reg::RF_STATUS] &= ~spec::rfstatus::TX_RF_STATUS;
                break;
            case spec::cmd::READ_DATA:
                std::memcpy(rsp, rfRx, sizeof(rfRx));
                rspLen = sizeof(rfRx);
                break;
            default: break;
        }
    }

    void sendData() {
        // DS Table 15 condition: Transceive command must be set.
        if ((regs[spec::reg::SYSTEM_CONFIG] & spec::kSystemConfigCommandMask) !=
            spec::kCommandTransceive) {
            regs[spec::reg::IRQ_STATUS] |= spec::irq::GENERAL_ERROR;
            return;
        }
        if (card == nullptr) return;
        size_t n = 0;
        uint32_t extra = 0;
        const bool txCrc = (regs[spec::reg::CRC_TX_CONFIG] & spec::kCrcTxEnable) != 0;
        if (card->respond(&lastCmd[2], lastCmdLen - 2, lastCmd[1], txCrc, rfRx, n, extra)) {
            regs[spec::reg::RX_STATUS] = static_cast<uint32_t>(n) | extra;
            regs[spec::reg::IRQ_STATUS] |= spec::irq::RX;
        }
    }
};

}  // namespace fake
