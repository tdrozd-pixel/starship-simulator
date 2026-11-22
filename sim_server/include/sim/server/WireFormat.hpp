#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "sim/vehicle/TruthSnapshot.hpp"

// Binary encoding of the truth stream, version 1. Layout is fixed by
// docs/protocol/truth_stream.md. Little-endian, assembled byte by byte so it
// does not depend on host endianness.
namespace sim::server {

constexpr std::size_t kSnapshotWireSize = 176;
constexpr uint16_t kProtocolVersion = 1;
constexpr uint16_t kMsgTruthSnapshot = 1;

// Byte offsets into the wire snapshot.
constexpr std::size_t kOffMagic = 0;
constexpr std::size_t kOffVersion = 4;
constexpr std::size_t kOffType = 6;
constexpr std::size_t kOffRunId = 8;
constexpr std::size_t kOffFlags = 12;
constexpr std::size_t kOffStep = 16;
constexpr std::size_t kOffSimTime = 24;
constexpr std::size_t kOffPosition = 32;
constexpr std::size_t kOffVelocity = 56;
constexpr std::size_t kOffAttitude = 80;
constexpr std::size_t kOffAngularRate = 112;
constexpr std::size_t kOffThrottle = 136;
constexpr std::size_t kOffGimbalPitch = 144;
constexpr std::size_t kOffGimbalYaw = 152;
constexpr std::size_t kOffPropellantMass = 160;
constexpr std::size_t kOffTotalMass = 168;

using WireSnapshot = std::array<uint8_t, kSnapshotWireSize>;

void encodeSnapshot(const vehicle::TruthSnapshot& snapshot, WireSnapshot& out) noexcept;

// Returns false on bad magic, version, type or length.
bool decodeSnapshot(const uint8_t* data, std::size_t size, vehicle::TruthSnapshot& out);

}  // namespace sim::server
