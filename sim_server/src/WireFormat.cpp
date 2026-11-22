#include "sim/server/WireFormat.hpp"

#include <bit>

namespace sim::server {
namespace {

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

void putF64(uint8_t* p, double v) { putU64(p, std::bit_cast<uint64_t>(v)); }

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

double getF64(const uint8_t* p) { return std::bit_cast<double>(getU64(p)); }

}  // namespace

void encodeSnapshot(const vehicle::TruthSnapshot& s, WireSnapshot& out) noexcept {
    uint8_t* b = out.data();
    b[kOffMagic + 0] = 'S';
    b[kOffMagic + 1] = 'S';
    b[kOffMagic + 2] = 'T';
    b[kOffMagic + 3] = 'R';
    putU16(b + kOffVersion, kProtocolVersion);
    putU16(b + kOffType, kMsgTruthSnapshot);
    putU32(b + kOffRunId, s.runId);
    putU32(b + kOffFlags, s.flags);
    putU64(b + kOffStep, s.step);
    putF64(b + kOffSimTime, s.simTime);
    for (std::size_t i = 0; i < 3; ++i) {
        putF64(b + kOffPosition + 8 * i, s.position[i]);
        putF64(b + kOffVelocity + 8 * i, s.velocity[i]);
        putF64(b + kOffAngularRate + 8 * i, s.angularRate[i]);
    }
    for (std::size_t i = 0; i < 4; ++i) {
        putF64(b + kOffAttitude + 8 * i, s.attitude[i]);
    }
    putF64(b + kOffThrottle, s.throttle);
    putF64(b + kOffGimbalPitch, s.gimbalPitch);
    putF64(b + kOffGimbalYaw, s.gimbalYaw);
    putF64(b + kOffPropellantMass, s.propellantMass);
    putF64(b + kOffTotalMass, s.totalMass);
}

bool decodeSnapshot(const uint8_t* b, std::size_t size, vehicle::TruthSnapshot& s) {
    if (b == nullptr || size != kSnapshotWireSize) {
        return false;
    }
    if (b[0] != 'S' || b[1] != 'S' || b[2] != 'T' || b[3] != 'R') {
        return false;
    }
    if (getU16(b + kOffVersion) != kProtocolVersion || getU16(b + kOffType) != kMsgTruthSnapshot) {
        return false;
    }
    s.runId = getU32(b + kOffRunId);
    s.flags = getU32(b + kOffFlags);
    s.step = getU64(b + kOffStep);
    s.simTime = getF64(b + kOffSimTime);
    for (std::size_t i = 0; i < 3; ++i) {
        s.position[i] = getF64(b + kOffPosition + 8 * i);
        s.velocity[i] = getF64(b + kOffVelocity + 8 * i);
        s.angularRate[i] = getF64(b + kOffAngularRate + 8 * i);
    }
    for (std::size_t i = 0; i < 4; ++i) {
        s.attitude[i] = getF64(b + kOffAttitude + 8 * i);
    }
    s.throttle = getF64(b + kOffThrottle);
    s.gimbalPitch = getF64(b + kOffGimbalPitch);
    s.gimbalYaw = getF64(b + kOffGimbalYaw);
    s.propellantMass = getF64(b + kOffPropellantMass);
    s.totalMass = getF64(b + kOffTotalMass);
    return true;
}

}  // namespace sim::server
