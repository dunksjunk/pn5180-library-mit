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
#include "pn5180/Iso15693.h"
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

// CRC_A of ISO/IEC 14443-3 Annex B: polynomial 1021h processed LSB first
// (8408h reflected), preset 6363h, no final inversion, sent LSB first.
inline uint16_t crcA(const uint8_t* p, size_t n) {
    uint16_t crc = 0x6363;
    for (size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0x8408) : uint16_t(crc >> 1);
    }
    return crc;
}

// Simulated NFC Forum Type 2 Tag (already activated). Frames arrive as on
// air: READ/WRITE must carry a correct CRC_A or the tag ignores them.
// Answers: READ -> 16 bytes + CRC_A; WRITE -> 4-bit ACK (Ah) or NAK (0h).
struct FakeT2T {
    static constexpr int kPages = 135;
    uint8_t mem[kPages][4] = {};
    bool nakWrites = false;      // answer every WRITE with NAK
    int weakWrites = 0;          // n > 0: the next n WRITEs store data ^ 01h (still ACK)
    int writes = 0;              // ACKed writes
    int ignored = 0;             // frames ignored for a bad/missing CRC

    FakeT2T() {
        // CC: E1h, version 1.0, 496 bytes data area (pages 4..127), RW.
        const uint8_t cc[4] = {0xE1, 0x10, 0x3E, 0x00};
        std::memcpy(mem[3], cc, 4);
        const uint8_t p0[4] = {0x04, 0xBB, 0x6A, 0x5D};
        std::memcpy(mem[0], p0, 4);
    }

    // Returns the answer; `lastBits` = valid bits of the last byte (0 = 8).
    bool respond(const uint8_t* in, size_t len, uint8_t* out, size_t& outLen, uint8_t& lastBits) {
        outLen = 0;
        lastBits = 0;
        if (len < 3 || crcA(in, len - 2) != uint16_t(in[len - 2] | (in[len - 1] << 8))) {
            ++ignored;
            return false;
        }
        if (in[0] == 0x30 && len == 4) {  // READ
            if (in[1] >= kPages) return nak(out, outLen, lastBits);
            for (int i = 0; i < 16; ++i) out[i] = mem[(in[1] + i / 4) % kPages][i % 4];
            const uint16_t c = crcA(out, 16);
            out[16] = uint8_t(c);
            out[17] = uint8_t(c >> 8);
            outLen = 18;
            return true;
        }
        if (in[0] == 0xA2 && len == 8) {  // WRITE
            if (nakWrites || in[1] >= kPages) return nak(out, outLen, lastBits);
            for (int i = 0; i < 4; ++i) {
                mem[in[1]][i] = weakWrites > 0 ? uint8_t(in[2 + i] ^ 0x01) : in[2 + i];
            }
            if (weakWrites > 0) --weakWrites;
            ++writes;
            out[0] = 0x0A;
            outLen = 1;
            lastBits = 4;
            return true;
        }
        return false;
    }

private:
    static bool nak(uint8_t* out, size_t& outLen, uint8_t& lastBits) {
        out[0] = 0x00;
        outLen = 1;
        lastBits = 4;
        return true;
    }
};

// CRC-16 of ISO/IEC 13239 as used by ISO/IEC 15693-3: polynomial 1021h
// processed LSB first (8408h reflected), preset FFFFh, ones' complement of
// the result, transmitted LSB first.
inline uint16_t crc13239(const uint8_t* p, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0x8408) : uint16_t(crc >> 1);
    }
    return uint16_t(~crc);
}

// Simulated ISO/IEC 15693 card. Frames arrive as they are on air (with CRC);
// the card checks the CRC and answers INVENTORY only.
struct FakeCardV {
    uint8_t uidMsbFirst[8] = {0xE0, 0x04, 0x01, 0x50, 0x12, 0x34, 0x56, 0x78};
    uint8_t dsfid = 0x00;
    bool corruptCrc = false;  // answer with a bad CRC
    int slot = -1;            // current slot of a 16-slot inventory, -1 = none
    int requests = 0;         // valid (CRC-correct) requests seen

    uint8_t uidLsb(int i) const { return uidMsbFirst[7 - i]; }
    // Mask length 0: the slot number is the UID's least significant nibble.
    int mySlot() const { return uidLsb(0) & 0x0F; }

    // `eofOnly`: an EOF without SOF/data (next slot). Returns true and the
    // on-air answer (with CRC) if the card answers.
    bool respond(bool eofOnly, const uint8_t* in, size_t len, uint8_t* out, size_t& outLen) {
        outLen = 0;
        if (eofOnly) {
            if (slot < 0 || slot >= 15) return false;
            ++slot;
            return slot == mySlot() && answer(out, outLen);
        }
        slot = -1;
        if (len < 2 + 2 || crc13239(in, len - 2) != uint16_t(in[len - 2] | (in[len - 1] << 8))) {
            return false;  // no/invalid CRC: a card ignores the frame
        }
        ++requests;
        const uint8_t flags = in[0];
        if ((flags & 0x04) == 0) return addressed(in, len - 2, out, outLen);
        if (len != 5 || in[1] != 0x01 || (flags & 0x04) == 0 || in[2] != 0x00) return false;
        if ((flags & 0x02) == 0) return false;  // only high data rate modelled
        if ((flags & 0x20) != 0) return answer(out, outLen);
        slot = 0;
        return mySlot() == 0 && answer(out, outLen);
    }

    // Memory: kBlocks blocks of blockSize bytes, per-block lock flag.
    static constexpr int kBlocks = 28;
    uint8_t blockSize = 4;
    uint8_t blocks[kBlocks][32] = {};
    bool locked[kBlocks] = {};
    uint8_t infoFlags = 0x0F;  // DSFID, AFI, memory size, IC reference
    int writes = 0;

private:
    // Addressed commands (non-inventory, flags 22h/62h). `len` without CRC.
    bool addressed(const uint8_t* in, size_t len, uint8_t* out, size_t& outLen) {
        const uint8_t flags = in[0];
        // Only addressed mode at high data rate is modelled; others ignored.
        if ((flags & 0x20) == 0 || (flags & 0x02) == 0 || (flags & 0x10) != 0 || len < 10) {
            return false;
        }
        for (int i = 0; i < 8; ++i) {
            if (in[2 + i] != uidLsb(i)) return false;  // not addressed to us
        }
        const bool option = (flags & 0x40) != 0;
        size_t n = 0;
        switch (in[1]) {
            case 0x2B:  // GET SYSTEM INFORMATION
                if (len != 10) return false;
                out[n++] = 0x00;
                out[n++] = infoFlags;
                for (int i = 0; i < 8; ++i) out[n++] = uidLsb(i);
                if (infoFlags & 0x01) out[n++] = dsfid;
                if (infoFlags & 0x02) out[n++] = 0x00;
                if (infoFlags & 0x04) {
                    out[n++] = uint8_t(kBlocks - 1);
                    out[n++] = uint8_t(blockSize - 1);
                }
                if (infoFlags & 0x08) out[n++] = 0x01;
                break;
            case 0x20:  // READ SINGLE BLOCK
                if (len != 11) return false;
                if (in[10] >= kBlocks) return error(0x10, out, outLen);  // block not available
                out[n++] = 0x00;
                if (option) out[n++] = locked[in[10]] ? 0x01 : 0x00;
                for (int i = 0; i < blockSize; ++i) out[n++] = blocks[in[10]][i];
                break;
            case 0x21:  // WRITE SINGLE BLOCK
                if (len != size_t(11 + blockSize) || option) return false;
                if (in[10] >= kBlocks) return error(0x10, out, outLen);
                if (locked[in[10]]) return error(0x12, out, outLen);  // block locked
                std::memcpy(blocks[in[10]], &in[11], blockSize);
                ++writes;
                out[n++] = 0x00;
                break;
            default: return error(0x01, out, outLen);  // command not supported
        }
        return finish(out, n, outLen);
    }

    bool error(uint8_t code, uint8_t* out, size_t& outLen) {
        out[0] = 0x01;  // error flag
        out[1] = code;
        return finish(out, 2, outLen);
    }

    bool finish(uint8_t* out, size_t n, size_t& outLen) {
        uint16_t crc = crc13239(out, n);
        if (corruptCrc) crc ^= 0x5555;
        out[n] = uint8_t(crc);
        out[n + 1] = uint8_t(crc >> 8);
        outLen = n + 2;
        return true;
    }

    bool answer(uint8_t* out, size_t& outLen) {
        out[0] = 0x00;
        out[1] = dsfid;
        for (int i = 0; i < 8; ++i) out[2 + i] = uidLsb(i);
        uint16_t crc = crc13239(out, 10);
        if (corruptCrc) crc ^= 0x5555;
        out[10] = uint8_t(crc);
        out[11] = uint8_t(crc >> 8);
        outLen = 12;
        return true;
    }
};

class FakeHal : public Hal {
public:
    // TX_CONFIG value the fake loads with LOAD_RF_CONFIG 0x0D: data enabled,
    // start symbol and stop symbol set (fake value, the DS does not list it).
    static constexpr uint32_t kTxConfig15693 =
        spec::txconfig::DATA_ENABLE | (1u << 6) | (1u << 8);
    FakeCardV* vcards[4] = {};  // ISO 15693 cards in the field
    uint8_t rfTxConfig = 0xFF;   // last LOAD_RF_CONFIG
    uint8_t rfRxConfig = 0xFF;
    int eofFrames = 0;           // EOF-only frames sent
    int errorAtSendData = 0;     // n > 0: GENERAL_ERROR on the n-th SEND_DATA

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
    FakeT2T* t2t = nullptr;     // activated Type 2 Tag (takes precedence over `card`)

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
            case spec::cmd::LOAD_RF_CONFIG:  // DS Table 31
                if (lastCmd[1] != spec::kRfConfigUnchanged) rfTxConfig = lastCmd[1];
                if (lastCmd[2] != spec::kRfConfigUnchanged) rfRxConfig = lastCmd[2];
                if (lastCmd[1] == spec::kRfConfigIso15693Ask100Tx) regs[spec::reg::TX_CONFIG] = kTxConfig15693;
                break;
            case spec::cmd::RF_ON:  // DS Table 37, Table 103
                regs[spec::reg::RF_STATUS] |= spec::rfstatus::TX_RF_STATUS;
                regs[spec::reg::IRQ_STATUS] |= spec::irq::TX_RFON;
                break;
            case spec::cmd::RF_OFF:  // DS Table 38, Table 103
                regs[spec::reg::RF_STATUS] &= ~spec::rfstatus::TX_RF_STATUS;
                for (FakeCardV* vc : vcards) {
                    if (vc != nullptr) vc->slot = -1;
                }
                break;
            case spec::cmd::READ_DATA:
                std::memcpy(rsp, rfRx, sizeof(rfRx));
                rspLen = sizeof(rfRx);
                break;
            default: break;
        }
    }

    void sendData() {
        if (errorAtSendData > 0 && --errorAtSendData == 0) {
            regs[spec::reg::IRQ_STATUS] |= spec::irq::GENERAL_ERROR;  // DS 11.4.1
            return;
        }
        // DS Table 15 condition: Transceive command must be set.
        if ((regs[spec::reg::SYSTEM_CONFIG] & spec::kSystemConfigCommandMask) !=
            spec::kCommandTransceive) {
            regs[spec::reg::IRQ_STATUS] |= spec::irq::GENERAL_ERROR;
            return;
        }
        if ((regs[spec::reg::RF_STATUS] & spec::rfstatus::TX_RF_STATUS) == 0) {
            for (FakeCardV* v : vcards) {
                if (v != nullptr) v->slot = -1;  // unpowered cards lose their state
            }
            return;  // no field: nobody answers
        }
        if (rfTxConfig == spec::kRfConfigIso15693Ask100Tx) {
            sendData15693();
            return;
        }
        if (t2t != nullptr) {
            sendDataT2T();
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

    // Type A frame on air: CRC_A appended when TX_CRC_ENABLE (DS Table 99).
    // Receive: with RX_CRC_ENABLE (DS Table 92) the CRC is checked and
    // removed; a frame too short to hold a CRC (4-bit ACK/NAK) then fails
    // the check -> DATA_INTEGRITY_ERROR, as a real CRC check would.
    void sendDataT2T() {
        if (rfTxConfig != spec::kRfConfigIso14443a106Tx) return;
        uint8_t air[300];
        size_t airLen = lastCmdLen - 2;
        std::memcpy(air, &lastCmd[2], airLen);
        if (lastCmd[1] != 0) return;  // only whole bytes modelled
        if ((regs[spec::reg::CRC_TX_CONFIG] & spec::kCrcTxEnable) != 0) {
            const uint16_t c = crcA(air, airLen);
            air[airLen++] = uint8_t(c);
            air[airLen++] = uint8_t(c >> 8);
        }
        uint8_t out[64];
        size_t n = 0;
        uint8_t bits = 0;
        if (!t2t->respond(air, airLen, out, n, bits)) return;
        uint32_t st = 0;
        if ((regs[spec::reg::CRC_RX_CONFIG] & spec::kCrcRxEnable) != 0) {
            if (bits != 0 || n < 2 || crcA(out, n - 2) != uint16_t(out[n - 2] | (out[n - 1] << 8))) {
                st |= spec::rxstatus::DATA_INTEGRITY_ERROR;
            } else {
                n -= 2;
            }
        }
        std::memcpy(rfRx, out, n);
        regs[spec::reg::RX_STATUS] = static_cast<uint32_t>(n) | st |
                                     (uint32_t(bits) << spec::rxstatus::NUM_LAST_BITS_SHIFT);
        regs[spec::reg::IRQ_STATUS] |= spec::irq::RX;
    }

    // ISO 15693 frame on air, built from TX_CONFIG / CRC_TX_CONFIG as the
    // chip would (DS Table 98, Table 99); answers combined like the receiver.
    void sendData15693() {
        if (rfRxConfig != spec::kRfConfigIso15693Ask100Rx) return;  // receiver deaf for 15693
        const uint32_t txc = regs[spec::reg::TX_CONFIG];
        const size_t len = lastCmdLen - 2;
        const bool dataOn = (txc & spec::txconfig::DATA_ENABLE) != 0;
        const bool sof = (txc & spec::txconfig::START_SYMBOL_MASK) != 0;
        const bool eof = (txc & spec::txconfig::STOP_SYMBOL_MASK) != 0;
        bool eofOnly = false;
        uint8_t air[300];
        size_t airLen = 0;
        if (!dataOn) {
            // Symbols only. A valid "next slot" is a bare EOF: no SOF, no data.
            if (len != 0 || sof || !eof) return;
            eofOnly = true;
            ++eofFrames;
        } else {
            if (!sof || !eof || len == 0) return;  // malformed frame, cards ignore it
            std::memcpy(air, &lastCmd[2], len);
            airLen = len;
            if ((regs[spec::reg::CRC_TX_CONFIG] & spec::kCrcTxEnable) != 0) {
                const uint16_t crc = crc13239(air, airLen);
                air[airLen++] = uint8_t(crc);
                air[airLen++] = uint8_t(crc >> 8);
            }
        }
        int answers = 0;
        uint8_t frame[64] = {};
        size_t frameLen = 0;
        for (FakeCardV* v : vcards) {
            if (v == nullptr) continue;
            uint8_t out[64];
            size_t n = 0;
            if (v->respond(eofOnly, air, airLen, out, n)) {
                ++answers;
                for (size_t i = 0; i < n; ++i) frame[i] |= out[i];  // superposition
                frameLen = n;
            }
        }
        if (answers == 0) return;
        uint32_t st = 0;
        size_t n = frameLen;
        if (answers > 1) {
            st |= spec::rxstatus::COLLISION_DETECTED;
        } else if ((regs[spec::reg::CRC_RX_CONFIG] & spec::kCrcRxEnable) != 0) {
            if (crc13239(frame, n - 2) != uint16_t(frame[n - 2] | (frame[n - 1] << 8))) {
                st |= spec::rxstatus::DATA_INTEGRITY_ERROR;
            }
            n -= 2;  // CRC checked and removed
        }
        std::memcpy(rfRx, frame, frameLen);
        regs[spec::reg::RX_STATUS] = static_cast<uint32_t>(n) | st;
        regs[spec::reg::IRQ_STATUS] |= spec::irq::RX;
    }
};

}  // namespace fake
