#include "sim/server/TelemetryFrame.hpp"

#include <algorithm>
#include <bit>

namespace sim::server {
namespace {

constexpr uint8_t kMagic[4] = {'S', 'T', 'L', 'M'};

void putU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

void putU32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
    }
}

void putU64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        p[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
    }
}

uint16_t getU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

uint32_t getU32(const uint8_t* p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
        v |= static_cast<uint32_t>(p[i]) << (8 * i);
    }
    return v;
}

uint64_t getU64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return v;
}

}  // namespace

std::size_t encodeGroundFrame(const TelemetryItem& item, uint8_t* out) noexcept {
    const std::size_t length = std::min<std::size_t>(item.length, vehicle::kMaxTelemetryPacket);
    std::copy(std::begin(kMagic), std::end(kMagic), out);
    putU16(out + 4, kTelemetryProtocolVersion);
    putU16(out + 6, kFrameTelemetryPacket);
    putU32(out + 8, item.runId);
    putU16(out + 12, static_cast<uint16_t>(length));
    putU16(out + 14, 0);
    putU64(out + 16, std::bit_cast<uint64_t>(item.groundTime));
    std::copy(item.bytes, item.bytes + length, out + kGroundHeaderSize);
    return kGroundHeaderSize + length;
}

bool decodeGroundFrame(const uint8_t* data, std::size_t size, TelemetryItem& out) {
    if (size < kGroundHeaderSize || !std::equal(std::begin(kMagic), std::end(kMagic), data)) {
        return false;
    }
    if (getU16(data + 4) != kTelemetryProtocolVersion || getU16(data + 6) != kFrameTelemetryPacket) {
        return false;
    }
    const uint16_t length = getU16(data + 12);
    if (length > vehicle::kMaxTelemetryPacket || size != kGroundHeaderSize + length) {
        return false;
    }
    out.runId = getU32(data + 8);
    out.length = length;
    out.groundTime = std::bit_cast<double>(getU64(data + 16));
    std::copy(data + kGroundHeaderSize, data + size, out.bytes);
    return true;
}

}  // namespace sim::server
