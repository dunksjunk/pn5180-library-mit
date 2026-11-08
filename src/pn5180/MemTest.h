// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Non-destructive write test of one card memory unit (a T2T page or an
// ISO/IEC 15693 block): read the original, write a pattern, verify, write
// the original back, verify. Portable core: no Arduino.h, no heap.
// Not thread-safe; shares the single-owner rule of Pn5180 (see Pn5180.h).
#pragma once

#include <cstddef>
#include <cstdint>

#include "Iso14443a.h"
#include "Iso15693.h"
#include "Status.h"
#include "Type2Tag.h"

namespace pn5180 {

/// One readable/writable memory unit (page or block) of at most kMaxBytes bytes.
class MemoryUnit {
public:
    /// Largest unit: an ISO/IEC 15693 block of 32 bytes.
    static constexpr size_t kMaxBytes = iso15693::kMaxBlockBytes;
    /// Bytes in this unit.
    virtual size_t size() const = 0;
    /// Reads size() bytes into @p out.
    virtual Status read(uint8_t* out) = 0;
    /// Writes size() bytes from @p in.
    virtual Status write(const uint8_t* in) = 0;
    /// Brings the card back to a state in which read()/write() can be
    /// repeated after a transient RF error. Default: nothing to do.
    virtual Status recover() { return Status::Ok; }

protected:
    ~MemoryUnit() = default;
};

/// One T2T page (Type2Tag enforces the data area and the CC access byte).
/// With `reader` and the card's UID, recover() re-activates the same card:
/// a transmission error sends an active PICC back to IDLE, so a command can
/// only be repeated after a new activation.     ISO/IEC 14443-3 6.3 (state diagram)
class Type2TagPage final : public MemoryUnit {
public:
    /// Page @p page of @p tag, without recovery.
    Type2TagPage(Type2Tag& tag, uint8_t page) : tag_(tag), page_(page) {}
    /// Page @p page of @p tag; recover() re-activates the card with this UID
    /// through @p reader (@p uid must stay valid).
    Type2TagPage(Type2Tag& tag, uint8_t page, Iso14443a& reader, const uint8_t* uid,
                 size_t uidLen)
        : tag_(tag), page_(page), reader_(&reader), uid_(uid), uidLen_(uidLen) {}
    size_t size() const override { return t2t::kPageBytes; }
    /// READ returns 4 pages; keeps the first.
    Status read(uint8_t* out) override;
    Status write(const uint8_t* in) override { return tag_.write(page_, in); }
    /// Field off/on, then WUPA + SELECT of this UID (Iso14443a::activateUid,
    /// up to ActivateRetry{}.attempts tries), so another card in the field
    /// cannot answer instead. Without a reader: Ok, nothing done.
    Status recover() override;

private:
    Type2Tag& tag_;
    uint8_t page_;
    Iso14443a* reader_ = nullptr;
    const uint8_t* uid_ = nullptr;
    size_t uidLen_ = 0;
};

/// One ISO/IEC 15693 block, addressed by UID.
class Iso15693Block final : public MemoryUnit {
public:
    /// Block @p block (@p blockSize bytes) of the card with @p uid (MSB
    /// first, must stay valid), through @p reader.
    Iso15693Block(Iso15693& reader, const uint8_t uid[iso15693::kUidBytes], uint8_t block,
                  uint8_t blockSize)
        : reader_(reader), uid_(uid), block_(block), size_(blockSize) {}
    size_t size() const override { return size_; }
    Status read(uint8_t* out) override { return reader_.readBlock(uid_, block_, out, size_); }
    Status write(const uint8_t* in) override { return reader_.writeBlock(uid_, block_, in, size_); }

private:
    Iso15693& reader_;
    const uint8_t* uid_;
    uint8_t block_;
    uint8_t size_;
};

/// Step of runMemTest() that failed.
enum class MemTestStep : uint8_t {
    None,           ///< passed
    ReadOriginal,   ///< nothing was written
    WritePattern,   ///< writing the test pattern failed
    VerifyPattern,  ///< read-back failed or differed (CardError)
    Restore,        ///< writing the original back failed
    VerifyRestore,  ///< original read back wrong (CardError) or unreadable
};

/// Outcome of runMemTest().
struct MemTestResult {
    Status status = Status::Ok;           ///< first failure
    MemTestStep step = MemTestStep::None; ///< step of the first failure
    bool restored = false;                ///< original written back and verified
    Status restoreStatus = Status::Ok;    ///< outcome of the restore when step < Restore
    uint8_t original[MemoryUnit::kMaxBytes] = {};  ///< content before the test
    uint8_t pattern[MemoryUnit::kMaxBytes] = {};   ///< test pattern written
};

/// Transient RF errors (no answer, garbled or colliding frame) are retried:
/// recover(), then the whole write+read-back step again, up to
/// kMemTestAttempts attempts per step. Repeating a write is safe: it stores
/// the same data.
constexpr int kMemTestAttempts = 3;

/// Non-destructive write test of one unit. The pattern is derived from
/// @p seed (e.g. the UID) and the
/// unit size, and is guaranteed to differ from the original in every byte so a
/// stale value cannot pass. Once anything was written, the original is
/// written back (and verified) on every path, also after a failed pattern
/// write or verify. Never use on a unit whose write could brick the card.
MemTestResult runMemTest(MemoryUnit& unit, const uint8_t* seed, size_t seedLen);

/// Name of @p step for logging. Never returns nullptr.
const char* memTestStepToString(MemTestStep step);

}  // namespace pn5180
