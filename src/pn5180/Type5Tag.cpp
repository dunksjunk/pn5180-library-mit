// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NFC Forum Type 5 Tag NDEF mapping over ISO/IEC 15693. Portable core.
#include "Type5Tag.h"

namespace pn5180 {

using namespace t5t;

size_t Type5Tag::blocks() const {
    const size_t n = info_.blockCount;
    return n < kMaxBlocks1Byte ? n : kMaxBlocks1Byte;
}

Status Type5Tag::capability(T5tCapability& cc) {
    ccValid_ = false;
    if (!infoValid_) {
        const Status s = reader_.getSystemInfo(uid_, info_);
        if (!ok(s)) {
            return s;
        }
        infoValid_ = true;
    }
    if ((info_.infoFlags & iso15693::INFO_MEMORY_SIZE) == 0 || info_.blockSize == 0 ||
        info_.blockSize > tlv::kMaxUnitBytes || info_.blockCount == 0) {
        return Status::Unsupported;
    }
    const size_t bs = info_.blockSize;
    const size_t memBytes = blocks() * bs;
    if (memBytes < kCcShortBytes) {
        return Status::NotFormatted;
    }
    // Read the blocks covering the first 8 bytes (enough for an 8-byte CC).
    T5tCapability c;
    const size_t want = memBytes < kCcLongBytes ? memBytes : kCcLongBytes;
    for (size_t b = 0; b * bs < want; ++b) {
        uint8_t blk[iso15693::kMaxBlockBytes] = {};
        const Status s = reader_.readBlock(uid_, static_cast<uint8_t>(b), blk, bs);
        if (!ok(s)) {
            return s;
        }
        for (size_t i = 0; i < bs && b * bs + i < kCcLongBytes; ++i) {
            c.raw[b * bs + i] = blk[i];
        }
    }
    // T5T (Capability Container)
    if (c.raw[0] != kCcMagic1 && c.raw[0] != kCcMagic2) {
        return Status::NotFormatted;
    }
    const uint8_t v = c.raw[1];
    c.versionMajor = static_cast<uint8_t>(v >> kMajorShift);
    c.versionMinor = static_cast<uint8_t>((v >> kMinorShift) & 0x03);
    c.readAccess = static_cast<uint8_t>((v >> kReadAccessShift) & kAccessMask);
    c.writeAccess = static_cast<uint8_t>(v & kAccessMask);
    c.features = c.raw[3];
    if (c.raw[2] != 0) {
        c.ccBytes = kCcShortBytes;
        c.mlenBytes = static_cast<uint32_t>(c.raw[2] * kMlenUnitBytes);
    } else {
        if (want < kCcLongBytes) {
            return Status::NotFormatted;
        }
        c.ccBytes = kCcLongBytes;
        c.mlenBytes = static_cast<uint32_t>(((static_cast<uint32_t>(c.raw[6]) << 8) | c.raw[7]) *
                                            kMlenUnitBytes);
    }
    if (c.mlenBytes == 0) {
        return Status::NotFormatted;
    }
    // T5T (NDEF detection procedure): do not access a higher major version
    // or a tag whose read access is not granted.
    if (c.versionMajor != kSupportedMajor || c.readAccess != kAccessGranted) {
        return Status::Unsupported;
    }
    c.readOnly = c.writeAccess != kAccessGranted;
    // MLEN is the T5T area size; clamp to the physical memory so a CC that
    // counts the CC itself in MLEN (or overstates it) cannot address blocks
    // that do not exist.
    c.areaStart = c.ccBytes;
    const uint64_t end = static_cast<uint64_t>(c.ccBytes) + c.mlenBytes;
    c.areaEnd = static_cast<uint32_t>(end < memBytes ? end : memBytes);
    if (c.areaEnd <= c.areaStart) {
        return Status::NotFormatted;
    }
    cc_ = c;
    ccValid_ = true;
    cc = c;
    return Status::Ok;
}

Status Type5Tag::ensureCapability() {
    if (ccValid_) {
        return Status::Ok;
    }
    T5tCapability c;
    return capability(c);
}

Status Type5Tag::readNdef(uint8_t* buf, size_t capacity, size_t& len) {
    len = 0;
    const Status s = ensureCapability();
    return ok(s) ? readNdefMessage(mem_, buf, capacity, len) : s;
}

Status Type5Tag::writeNdef(const uint8_t* msg, size_t len) {
    const Status s = ensureCapability();
    return ok(s) ? writeNdefMessage(mem_, msg, len) : s;
}

Status Type5Tag::Memory::readUnit(size_t unit, uint8_t* out) {
    if (!tag_.ccValid_ || out == nullptr || unit >= tag_.blocks()) {
        return Status::InvalidArgument;
    }
    return tag_.reader_.readBlock(tag_.uid_, static_cast<uint8_t>(unit), out,
                                  tag_.info_.blockSize);
}

Status Type5Tag::Memory::writeUnit(size_t unit, const uint8_t* in) {
    if (!tag_.ccValid_ || in == nullptr || unit >= tag_.blocks()) {
        return Status::InvalidArgument;
    }
    // Never write a block that holds CC bytes.
    if (unit * tag_.info_.blockSize < tag_.cc_.ccBytes) {
        return Status::InvalidArgument;
    }
    if (tag_.cc_.readOnly) {
        return Status::ReadOnly;
    }
    return tag_.reader_.writeBlock(tag_.uid_, static_cast<uint8_t>(unit), in,
                                   tag_.info_.blockSize);
}

}  // namespace pn5180
