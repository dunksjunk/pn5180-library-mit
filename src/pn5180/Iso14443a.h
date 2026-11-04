// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 14443-3 Type A activation (REQA/WUPA, anticollision, SELECT,
// HLTA) on top of Pn5180::transceive(). Portable core: no Arduino.h, no heap.
//
// Card-side protocol per the public standard ISO/IEC 14443-3; PN5180 setup
// per the NXP data sheet (DS) and AN12650.
#pragma once

#include <cstddef>
#include <cstdint>

#include "Pn5180.h"
#include "Status.h"

namespace pn5180 {

struct CardA {
    uint8_t uid[10] = {};  // 4, 7 or 10 bytes             ISO/IEC 14443-3 6.5.4
    uint8_t uidLen = 0;
    uint16_t atqa = 0;     // first received byte = LSB    ISO/IEC 14443-3 6.5.2
    uint8_t sak = 0;       // SAK of the last cascade level ISO/IEC 14443-3 6.5.3.4
};

namespace iso14443a {
// Card commands                                   ISO/IEC 14443-3 6.4, 6.5
constexpr uint8_t REQA = 0x26;          // short frame, 7 bits
constexpr uint8_t WUPA = 0x52;          // short frame, 7 bits
constexpr uint8_t SEL_CL1 = 0x93;
constexpr uint8_t SEL_CL2 = 0x95;
constexpr uint8_t SEL_CL3 = 0x97;
constexpr uint8_t NVB_ANTICOLLISION = 0x20;  // 2 bytes sent, no UID bits
constexpr uint8_t NVB_SELECT = 0x70;         // 7 bytes sent (full UID CLn + BCC)
constexpr uint8_t HLTA_0 = 0x50;
constexpr uint8_t HLTA_1 = 0x00;
constexpr uint8_t CASCADE_TAG = 0x88;        // CT: UID continues at next level
constexpr uint8_t SAK_CASCADE_BIT = 0x04;    // SAK b3: UID not complete
constexpr uint8_t kShortFrameBits = 7;       // REQA/WUPA are 7-bit short frames
constexpr size_t kMaxCascadeLevels = 3;
}  // namespace iso14443a

class Iso14443a {
public:
    explicit Iso14443a(Pn5180& nfc) : nfc_(nfc) {}

    // Host-side receive timeout per exchange. Type A answers come within a
    // few hundred microseconds (ISO/IEC 14443-3 6.2.1.1 frame delay time);
    // the default leaves margin for millisecond clock granularity.
    void setResponseTimeoutMs(uint32_t ms) { timeoutMs_ = ms; }

    // LOAD_RF_CONFIG for ISO 14443-A 106 kbit/s (DS Table 32), RF_ON and
    // wait for the field (DS Table 37, Table 103), set the receiver to
    // RX_GAIN 2 / SIGPRO_RM_CONFIG 0x430DC (DS Tables 102, 108, 172; must be
    // after RF_ON because of ARC, DS 11.8.6), then wait `guardMs` with
    // unmodulated field so the card can power up before the first command
    // (ISO/IEC 14443-3 5: >= 5 ms).
    Status fieldOn(uint32_t guardMs = 5);

    // RF_OFF (DS Table 38).
    Status fieldOff();

    // REQA (or WUPA when `wakeup`), returns ATQA.  ISO/IEC 14443-3 6.5.2
    Status request(uint16_t& atqa, bool wakeup = false);

    // Anticollision + SELECT over cascade levels 1..3. Collisions are not
    // resolved: Status::Collision is returned.  ISO/IEC 14443-3 6.5.3
    Status select(CardA& card);

    // request() + select(). NoResponse means no card in the field.
    Status activate(CardA& card, bool wakeup = false);

    // HLTA. A card must not answer (ISO/IEC 14443-3 6.4.3), so no response
    // within the timeout is success; any answer is CardError.
    Status halt();

private:
    // Receiver settings written after RF_ON (see Iso14443a.cpp).
    // DS 11.8.6, Tables 102, 108, 172
    Status setupReceiver();
    Status selectLevel(uint8_t sel, uint8_t clUid[4], uint8_t& sak);

    Pn5180& nfc_;
    uint32_t timeoutMs_ = 5;
};

}  // namespace pn5180
