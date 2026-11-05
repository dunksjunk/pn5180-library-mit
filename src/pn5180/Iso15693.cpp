// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// ISO/IEC 15693 INVENTORY. Portable core: no Arduino.h.
#include "Iso15693.h"

#include "Spec.h"

namespace pn5180 {

using namespace iso15693;

namespace {

// Statuses meaning "a slot held an answer, but not a usable one". Anything
// else that is not Ok/NoResponse is a device error and aborts the inventory.
bool isGarbled(Status s) {
    return s == Status::Collision || s == Status::IntegrityError || s == Status::ProtocolError ||
           s == Status::CardError || s == Status::BufferTooSmall;
}

}  // namespace

Status Iso15693::fieldOn(uint32_t guardMs) {
    // ISO 15693 ASK100 26 kbit/s Tx 0x0D / Rx 0x8D.  DS Table 32; AN12650 4.2.3 line 1
    Status s = nfc_.loadRfConfig(spec::kRfConfigIso15693Ask100Tx, spec::kRfConfigIso15693Ask100Rx);
    if (ok(s)) {
        s = nfc_.rfOnAndWait();  // DS Table 37, Table 103; AN12650 4.2.3 line 2
    }
    if (ok(s)) {
        // Every ISO 15693 frame carries a CRC.  ISO/IEC 15693-3 (CRC); DS Tables 92, 99
        s = nfc_.setCrc(true, true);
    }
    if (ok(s) && guardMs > 0) {
        // Cards need >= kPowerUpMs of unmodulated field.  ISO/IEC 15693-3 (power-up)
        nfc_.delayMs(guardMs < kPowerUpMs ? kPowerUpMs : guardMs);
    }
    return s;
}

Status Iso15693::fieldOff() {
    return nfc_.rfOff();  // DS Table 38; AN12650 4.2.1 line 16
}

Status Iso15693::exchange(const uint8_t* tx, size_t txLen, CardV& card) {
    // Slack beyond the expected length so an overlong (garbled) frame is
    // still read and reported as CardError.
    uint8_t rx[kInventoryResponseBytes + kCrcBytes + 4] = {};
    size_t n = 0;
    uint8_t lastBits = 0;
    const Status s = nfc_.transceive(tx, txLen, 0, rx, sizeof(rx), n, lastBits, timeoutMs_);
    if (!ok(s)) {
        return s;
    }
    // The PN5180 checks and removes the CRC (RX_CRC_ENABLE, DS Table 92):
    // on the test bed exactly 10 bytes arrive. 12 bytes would mean the CRC
    // check is off, which must not pass silently.
    if (n != kInventoryResponseBytes || lastBits != 0 || (rx[0] & RESP_FLAG_ERROR) != 0) {
        return Status::CardError;
    }
    // rx: flags, DSFID, UID LSB first.  ISO/IEC 15693-3 (inventory response, UID)
    if (rx[2 + kUidBytes - 1] != kUidMsb) {
        return Status::CardError;
    }
    card.dsfid = rx[1];
    for (size_t i = 0; i < kUidBytes; ++i) {
        card.uid[i] = rx[2 + kUidBytes - 1 - i];  // store MSB first
    }
    return Status::Ok;
}

Status Iso15693::inventory(CardV& card) {
    // flags 26h, INVENTORY, mask length 0.  ISO/IEC 15693-3 (inventory)
    const uint8_t req[3] = {kRequestFlags1Slot, CMD_INVENTORY, kMaskLengthNone};
    return exchange(req, sizeof(req), card);
}

Status Iso15693::runSlots(InventoryResult& result) {
    // Slot 0 is opened by the request itself.  AN12650 4.2.1 line 6
    const uint8_t req[3] = {kRequestFlags16Slots, CMD_INVENTORY, kMaskLengthNone};
    for (size_t slot = 0; slot < kSlots; ++slot) {
        CardV card;
        // Slots 1..15: zero-length SEND_DATA = EOF only (TX_CONFIG set by
        // inventory16()).  AN12650 4.2.1 line 15; DS Table 15 condition
        const Status s = slot == 0 ? exchange(req, sizeof(req), card) : exchange(nullptr, 0, card);
        if (ok(s)) {
            result.cards[result.count++] = card;
        } else if (isGarbled(s)) {
            result.collisionSlots = static_cast<uint16_t>(result.collisionSlots | (1u << slot));
        } else if (s != Status::NoResponse) {
            return s;  // device error
        }
        if (s != Status::NoResponse) {
            // A card answered: wait t2 before the next EOF.  ISO/IEC 15693-3 (t2)
            nfc_.delayMs(kT2Ms);
        }
        if (slot == 0) {
            // From now on send EOF only: clear TX_START_SYMBOL and
            // TX_DATA_ENABLE, keep the stop symbol.  AN12650 4.2.3 line 11; DS Table 98
            const Status m = nfc_.writeRegisterAndMask(spec::reg::TX_CONFIG,
                                                       spec::txconfig::kEofOnlyAndMask);
            if (!ok(m)) {
                return m;
            }
        }
    }
    return Status::Ok;
}

Status Iso15693::inventory16(InventoryResult& result) {
    result = InventoryResult{};
    uint32_t saved = 0;
    Status s = nfc_.readRegister(spec::reg::TX_CONFIG, saved);  // DS Table 98
    if (!ok(s)) {
        return s;
    }
    s = runSlots(result);
    // Restore TX_CONFIG on every path, so the next request carries SOF and data again.
    const Status r = nfc_.writeRegister(spec::reg::TX_CONFIG, saved);
    if (!ok(s)) {
        return s;
    }
    if (!ok(r)) {
        return r;
    }
    if (result.count > 0) {
        return Status::Ok;
    }
    return result.collisionSlots != 0 ? Status::Collision : Status::NoResponse;
}

}  // namespace pn5180
