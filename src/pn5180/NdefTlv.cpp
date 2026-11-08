// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// NDEF Message TLV scan / read / write over an NdefMemory. Portable core.
#include "NdefTlv.h"

namespace pn5180 {

using namespace tlv;

namespace {

bool geometryValid(const NdefMemory& mem) {
    const size_t u = mem.unitSize();
    return u != 0 && u <= kMaxUnitBytes && mem.areaStart() < mem.areaEnd();
}

// Byte-wise read access with a one-unit cache.
class ByteReader {
public:
    explicit ByteReader(NdefMemory& mem) : mem_(mem), unitSize_(mem.unitSize()) {}

    Status get(size_t addr, uint8_t& b) {
        const size_t unit = addr / unitSize_;
        if (!cached_ || unit != unit_) {
            cached_ = false;
            const Status s = mem_.readUnit(unit, buf_);
            if (!ok(s)) {
                return s;
            }
            unit_ = unit;
            cached_ = true;
        }
        b = buf_[addr % unitSize_];
        return Status::Ok;
    }

private:
    NdefMemory& mem_;
    size_t unitSize_;
    size_t unit_ = 0;
    bool cached_ = false;
    uint8_t buf_[kMaxUnitBytes] = {};
};

bool overlaps(const NdefTlvScan& scan, size_t start, size_t end) {
    for (size_t i = 0; i < scan.reservedCount; ++i) {
        if (start < scan.reserved[i].end && scan.reserved[i].start < end) {
            return true;
        }
    }
    return false;
}

// Area declared by a Lock or Memory Control TLV value (3 bytes).
// T2T (Lock Control TLV, Memory Control TLV)
NdefTlvScan::Range controlRange(uint8_t type, const uint8_t v[kControlValueBytes]) {
    const size_t pageAddr = v[0] >> 4;
    const size_t byteOffset = v[0] & 0x0F;
    const size_t pageBytes = static_cast<size_t>(1) << (v[2] & 0x0F);
    const size_t size = v[1] == 0 ? 256u : v[1];  // 0 encodes 256
    // Lock Control: size in bits -> bytes rounded up; Memory Control: bytes.
    const size_t bytes = type == T_LOCK_CONTROL ? (size + 7u) / 8u : size;
    NdefTlvScan::Range r;
    r.start = pageAddr * pageBytes + byteOffset;
    r.end = r.start + bytes;
    return r;
}

// Final (or, with `emptyLength`, L = 00h) content of byte `addr` inside the
// written TLV range of `plan`.
uint8_t imageByte(const NdefWritePlan& plan, const uint8_t* msg, size_t addr, bool emptyLength) {
    const size_t off = addr - plan.tlvStart;
    if (off == 0) {
        return T_NDEF;
    }
    if (off < plan.headerBytes) {
        if (emptyLength) {
            // Only the first L byte matters: 00h = empty NDEF TLV.
            return 0x00;
        }
        if (plan.headerBytes == 2) {
            return static_cast<uint8_t>(plan.msgLength);
        }
        if (off == 1) {
            return kLength3Marker;
        }
        return static_cast<uint8_t>(off == 2 ? plan.msgLength >> 8 : plan.msgLength);
    }
    if (off < plan.headerBytes + plan.msgLength) {
        return msg[off - plan.headerBytes];
    }
    return T_TERMINATOR;  // only reached when plan.terminator
}

// Builds and writes unit `unit` of the image; bytes outside the TLV range
// keep their current value (read first if needed).
Status writeImageUnit(NdefMemory& mem, const NdefWritePlan& plan, const uint8_t* msg, size_t unit,
                      bool emptyLength) {
    const size_t u = mem.unitSize();
    const size_t start = unit * u;
    const size_t end = plan.end();
    uint8_t buf[kMaxUnitBytes] = {};
    if (start < plan.tlvStart || start + u > end) {
        const Status s = mem.readUnit(unit, buf);
        if (!ok(s)) {
            return s;
        }
    }
    for (size_t i = 0; i < u; ++i) {
        const size_t a = start + i;
        if (a >= plan.tlvStart && a < end) {
            buf[i] = imageByte(plan, msg, a, emptyLength);
        }
    }
    return mem.writeUnit(unit, buf);
}

}  // namespace

Status scanNdefTlv(NdefMemory& mem, NdefTlvScan& scan) {
    scan = NdefTlvScan{};
    if (!geometryValid(mem)) {
        return Status::NotFormatted;
    }
    const size_t areaEnd = mem.areaEnd();
    ByteReader r(mem);
    size_t addr = mem.areaStart();
    // Start of the trailing run of NULL TLVs before the Terminator / end:
    // a new NDEF TLV goes there, after the last real TLV.
    size_t freeStart = addr;
    while (addr < areaEnd) {
        uint8_t t = 0;
        Status s = r.get(addr, t);
        if (!ok(s)) {
            return s;
        }
        if (t == T_NULL) {  // padding, 1 byte          T2T (NULL TLV)
            ++addr;
            continue;
        }
        if (t == T_TERMINATOR) {  // last TLV             T2T (Terminator TLV)
            break;
        }
        // L: 1 byte, or FFh + 2 bytes MSB first.        T2T (TLV blocks: length field)
        if (areaEnd - addr < 2) {
            return Status::MalformedData;
        }
        uint8_t l0 = 0;
        s = r.get(addr + 1, l0);
        if (!ok(s)) {
            return s;
        }
        size_t header = 2;
        size_t length = l0;
        if (l0 == kLength3Marker) {
            if (areaEnd - addr < 4) {
                return Status::MalformedData;
            }
            uint8_t hi = 0;
            uint8_t lo = 0;
            s = r.get(addr + 2, hi);
            if (ok(s)) {
                s = r.get(addr + 3, lo);
            }
            if (!ok(s)) {
                return s;
            }
            header = 4;
            length = (static_cast<size_t>(hi) << 8) | lo;
        }
        if (length > areaEnd - addr - header) {
            return Status::MalformedData;
        }
        if (t == T_NDEF) {
            scan.found = true;
            scan.tlvStart = addr;
            scan.valueStart = addr + header;
            scan.length = length;
            return Status::Ok;
        }
        if (t == T_LOCK_CONTROL || t == T_MEMORY_CONTROL) {
            if (length != kControlValueBytes) {
                return Status::MalformedData;
            }
            if (scan.reservedCount >= kMaxReserved) {
                return Status::Unsupported;
            }
            uint8_t v[kControlValueBytes] = {};
            for (size_t i = 0; i < kControlValueBytes; ++i) {
                s = r.get(addr + header + i, v[i]);
                if (!ok(s)) {
                    return s;
                }
            }
            scan.reserved[scan.reservedCount++] = controlRange(t, v);
        }
        // Lock/Memory Control, proprietary and unknown TLVs: skip V.
        addr += header + length;
        freeStart = addr;
    }
    // No NDEF TLV: a new one goes after the last TLV (over NULL padding or
    // the Terminator).
    scan.tlvStart = freeStart;
    return Status::Ok;
}

Status readNdefMessage(NdefMemory& mem, uint8_t* buf, size_t capacity, size_t& len) {
    len = 0;
    NdefTlvScan scan;
    Status s = scanNdefTlv(mem, scan);
    if (!ok(s) || !scan.found || scan.length == 0) {
        return s;
    }
    if (overlaps(scan, scan.valueStart, scan.valueStart + scan.length)) {
        return Status::Unsupported;  // would have to skip reserved bytes
    }
    if (buf == nullptr || scan.length > capacity) {
        len = scan.length;
        return buf == nullptr && capacity != 0 ? Status::InvalidArgument : Status::BufferTooSmall;
    }
    ByteReader r(mem);
    for (size_t i = 0; i < scan.length; ++i) {
        s = r.get(scan.valueStart + i, buf[i]);
        if (!ok(s)) {
            return s;
        }
    }
    len = scan.length;
    return Status::Ok;
}

Status planNdefWrite(NdefMemory& mem, size_t msgLength, NdefWritePlan& plan) {
    plan = NdefWritePlan{};
    NdefTlvScan scan;
    const Status s = scanNdefTlv(mem, scan);
    if (!ok(s)) {
        return s;
    }
    if (mem.readOnly()) {
        return Status::ReadOnly;
    }
    if (msgLength > kMaxLength) {
        return Status::BufferTooSmall;
    }
    plan.tlvStart = scan.tlvStart;
    plan.headerBytes = msgLength <= kMaxShortLength ? 2u : 4u;
    plan.msgLength = msgLength;
    const size_t areaEnd = mem.areaEnd();
    const size_t tlvEnd = plan.tlvStart + plan.headerBytes + msgLength;
    if (plan.tlvStart > areaEnd || tlvEnd > areaEnd) {
        return Status::BufferTooSmall;
    }
    // Terminator only if a byte is left.            T2T (Terminator TLV)
    plan.terminator = tlvEnd < areaEnd;
    if (overlaps(scan, plan.tlvStart, plan.end())) {
        return Status::Unsupported;
    }
    const size_t u = mem.unitSize();
    plan.firstUnit = plan.tlvStart / u;
    plan.lastUnit = (plan.end() - 1) / u;
    // Never rewrite a unit that also holds CC bytes.
    if (plan.firstUnit * u < mem.areaStart()) {
        return Status::Unsupported;
    }
    return Status::Ok;
}

Status writeNdefMessage(NdefMemory& mem, const uint8_t* msg, size_t len) {
    if (msg == nullptr && len != 0) {
        return Status::InvalidArgument;
    }
    NdefWritePlan plan;
    Status s = planNdefWrite(mem, len, plan);
    if (!ok(s)) {
        return s;
    }
    const size_t u = mem.unitSize();
    const size_t tUnit = plan.tlvStart / u;        // holds T
    const size_t lUnit = (plan.tlvStart + 1) / u;  // holds the first L byte
    // T2T (NDEF write procedure): 1. empty NDEF TLV (L = 00h) first.
    s = writeImageUnit(mem, plan, msg, tUnit, true);
    if (ok(s) && lUnit != tUnit) {
        s = writeImageUnit(mem, plan, msg, lUnit, true);
    }
    // 2. message body and Terminator.
    for (size_t unit = plan.firstUnit; ok(s) && unit <= plan.lastUnit; ++unit) {
        if (unit != tUnit && unit != lUnit) {
            s = writeImageUnit(mem, plan, msg, unit, false);
        }
    }
    // 3. the real length last.
    if (ok(s)) {
        s = writeImageUnit(mem, plan, msg, tUnit, false);
    }
    if (ok(s) && lUnit != tUnit) {
        s = writeImageUnit(mem, plan, msg, lUnit, false);
    }
    return s;
}

Status backupNdefArea(NdefMemory& mem, const NdefWritePlan& plan, NdefBackup& backup) {
    backup.valid = false;
    const size_t u = mem.unitSize();
    if (u == 0 || u > kMaxUnitBytes || plan.lastUnit < plan.firstUnit) {
        return Status::InvalidArgument;
    }
    const size_t units = plan.lastUnit - plan.firstUnit + 1;
    if (units > NdefBackup::kMaxBytes / u) {
        return Status::BufferTooSmall;
    }
    for (size_t i = 0; i < units; ++i) {
        const Status s = mem.readUnit(plan.firstUnit + i, backup.data + i * u);
        if (!ok(s)) {
            return s;
        }
    }
    backup.firstUnit = plan.firstUnit;
    backup.units = units;
    backup.unitSize = u;
    backup.tlvStart = plan.tlvStart;
    backup.valid = true;
    return Status::Ok;
}

Status restoreNdefArea(NdefMemory& mem, const NdefBackup& backup) {
    const size_t u = backup.unitSize;
    if (!backup.valid || u != mem.unitSize() || backup.units == 0 ||
        backup.units > NdefBackup::kMaxBytes / u) {
        return Status::InvalidArgument;
    }
    const size_t first = backup.firstUnit;
    const size_t last = first + backup.units - 1;
    const size_t tUnit = backup.tlvStart / u;
    const size_t lUnit = (backup.tlvStart + 1) / u;
    const bool tIn = tUnit >= first && tUnit <= last;
    const bool lIn = lUnit >= first && lUnit <= last;
    auto saved = [&](size_t unit) { return backup.data + (unit - first) * u; };

    // 1. T/L unit(s) as an empty NDEF TLV, so the area stays valid while the
    //    other units are restored.  T2T (NDEF write procedure)
    auto writeEmptyMarker = [&](size_t unit) {
        uint8_t buf[kMaxUnitBytes] = {};
        for (size_t i = 0; i < u; ++i) {
            buf[i] = saved(unit)[i];
        }
        for (size_t a = backup.tlvStart; a < backup.tlvStart + 2; ++a) {
            if (a / u == unit) {
                buf[a % u] = a == backup.tlvStart ? T_NDEF : 0x00;
            }
        }
        return mem.writeUnit(unit, buf);
    };
    Status s = Status::Ok;
    if (tIn) {
        s = writeEmptyMarker(tUnit);
    }
    if (ok(s) && lIn && lUnit != tUnit) {
        s = writeEmptyMarker(lUnit);
    }
    // 2. everything else.
    for (size_t unit = first; ok(s) && unit <= last; ++unit) {
        if (!(tIn && unit == tUnit) && !(lIn && unit == lUnit)) {
            s = mem.writeUnit(unit, saved(unit));
        }
    }
    // 3. T/L unit(s) from the backup, last.
    if (ok(s) && tIn) {
        s = mem.writeUnit(tUnit, saved(tUnit));
    }
    if (ok(s) && lIn && lUnit != tUnit) {
        s = mem.writeUnit(lUnit, saved(lUnit));
    }
    if (!ok(s)) {
        return s;
    }
    // Verify.
    for (size_t unit = first; unit <= last; ++unit) {
        uint8_t buf[kMaxUnitBytes] = {};
        s = mem.readUnit(unit, buf);
        if (!ok(s)) {
            return s;
        }
        for (size_t i = 0; i < u; ++i) {
            if (buf[i] != saved(unit)[i]) {
                return Status::CardError;
            }
        }
    }
    return Status::Ok;
}

}  // namespace pn5180
