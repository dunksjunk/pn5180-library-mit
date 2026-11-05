// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Non-destructive memory write test. Portable core.
#include "MemTest.h"

namespace pn5180 {

namespace {

bool equal(const uint8_t* a, const uint8_t* b, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// Errors after which repeating the exchange can succeed.
bool transient(Status s) {
    return s == Status::NoResponse || s == Status::Collision || s == Status::IntegrityError ||
           s == Status::ProtocolError;
}

// Writes `data` and reads it back, once.
Status writeVerifyOnce(MemoryUnit& unit, const uint8_t* data, bool& writeFailed) {
    uint8_t back[MemoryUnit::kMaxBytes] = {};
    Status s = unit.write(data);
    writeFailed = !ok(s);
    if (!ok(s)) {
        return s;
    }
    s = unit.read(back);
    if (!ok(s)) {
        return s;
    }
    return equal(back, data, unit.size()) ? Status::Ok : Status::CardError;
}

// writeVerifyOnce() with retries on transient errors (recover() first).
Status writeVerify(MemoryUnit& unit, const uint8_t* data, bool& writeFailed) {
    Status s = writeVerifyOnce(unit, data, writeFailed);
    for (int attempt = 1; attempt < kMemTestAttempts && transient(s); ++attempt) {
        const Status rc = unit.recover();
        if (!ok(rc) && !transient(rc)) {
            return rc;  // device error or a different card: stop
        }
        if (ok(rc)) {
            s = writeVerifyOnce(unit, data, writeFailed);
        }
    }
    return s;
}

// unit.read() with the same retry policy.
Status readRetry(MemoryUnit& unit, uint8_t* out) {
    Status s = unit.read(out);
    for (int attempt = 1; attempt < kMemTestAttempts && transient(s); ++attempt) {
        const Status rc = unit.recover();
        if (!ok(rc) && !transient(rc)) {
            return rc;
        }
        if (ok(rc)) {
            s = unit.read(out);
        }
    }
    return s;
}

}  // namespace

Status Type2TagPage::read(uint8_t* out) {
    uint8_t buf[t2t::kReadBytes] = {};
    const Status s = tag_.read(page_, buf);
    if (ok(s)) {
        for (size_t i = 0; i < t2t::kPageBytes; ++i) {
            out[i] = buf[i];
        }
    }
    return s;
}

Status Type2TagPage::recover() {
    if (reader_ == nullptr) {
        return Status::Ok;
    }
    // A field reset puts the PICC into POWER-OFF and then IDLE whatever state
    // the failed exchange left it in.  ISO/IEC 14443-3 6.3
    Status s = reader_->fieldOff();
    if (ok(s)) {
        s = reader_->fieldOn();
    }
    if (!ok(s)) {
        return s;
    }
    CardA card;
    s = reader_->activateWithRetry(card, true);  // WUPA  ISO/IEC 14443-3 6.4.1
    if (!ok(s)) {
        return s;
    }
    if (card.uidLen != uidLen_ || !equal(card.uid, uid_, uidLen_)) {
        return Status::CardError;  // another card answered
    }
    return Status::Ok;
}

MemTestResult runMemTest(MemoryUnit& unit, const uint8_t* seed, size_t seedLen) {
    MemTestResult r;
    const size_t n = unit.size();
    if (n == 0 || n > MemoryUnit::kMaxBytes || (seed == nullptr && seedLen != 0)) {
        r.status = Status::InvalidArgument;
        r.step = MemTestStep::ReadOriginal;
        return r;
    }
    Status s = readRetry(unit, r.original);
    if (!ok(s)) {
        r.status = s;
        r.step = MemTestStep::ReadOriginal;
        return r;
    }
    for (size_t i = 0; i < n; ++i) {
        const uint8_t k = seedLen != 0 ? seed[i % seedLen] : 0;
        uint8_t p = static_cast<uint8_t>((k ^ 0x5A) + 17u * i + n);
        if (p == r.original[i]) {
            p = static_cast<uint8_t>(~p);  // must differ from the original
        }
        r.pattern[i] = p;
    }

    bool writeFailed = false;
    s = writeVerify(unit, r.pattern, writeFailed);
    if (!ok(s)) {
        r.status = s;
        r.step = writeFailed ? MemTestStep::WritePattern : MemTestStep::VerifyPattern;
    }
    // Restore on every path once a write was attempted (a failed write may
    // still have changed the memory).
    const Status rs = writeVerify(unit, r.original, writeFailed);
    r.restored = ok(rs);
    r.restoreStatus = rs;
    if (ok(r.status) && !ok(rs)) {
        r.status = rs;
        r.step = writeFailed ? MemTestStep::Restore : MemTestStep::VerifyRestore;
    }
    return r;
}

const char* memTestStepToString(MemTestStep step) {
    switch (step) {
        case MemTestStep::None:          return "none";
        case MemTestStep::ReadOriginal:  return "read original";
        case MemTestStep::WritePattern:  return "write pattern";
        case MemTestStep::VerifyPattern: return "verify pattern";
        case MemTestStep::Restore:       return "restore";
        case MemTestStep::VerifyRestore: return "verify restore";
    }
    return "?";
}

}  // namespace pn5180
