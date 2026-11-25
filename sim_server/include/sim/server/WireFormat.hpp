#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "sim/vehicle/TruthSnapshot.hpp"

// Binary encoding of the truth stream, version 3. Layout is fixed by
// docs/protocol/truth_stream.md. Little-endian, assembled byte by byte so it
// does not depend on host endianness.
namespace sim::server {

constexpr std::size_t kSnapshotWireSize = 296;
constexpr uint16_t kProtocolVersion = 3;
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
// Version 2 additions: per engine actual values (engines 0..2 sea level, 3..5 Raptor Vacuum).
constexpr std::size_t kOffEngineThrottle = 176;     // f64[6]
constexpr std::size_t kOffEngineGimbalPitch = 224;  // f64[3], sea-level engines only
constexpr std::size_t kOffEngineGimbalYaw = 248;    // f64[3]
// Per engine running flags live in `flags`: engine i is bit (kFlagEngine0Shift + i), bits 3..8.
constexpr uint32_t kFlagEngine0Shift = 3;
constexpr uint32_t kFlagEngineMask = 0x3Fu << kFlagEngine0Shift;
// Version 3 additions: catch tower. Flags bit 9 = caught, bit 10 = crashed (kSnapCaught, kSnapCrashed).
constexpr std::size_t kOffArmAngle = 272;   // f64[2], actual arm angle psi from open [rad], arm 0 then arm 1
constexpr std::size_t kOffTowerState = 288; // u32, 0 Open, 1 Closing, 2 Closed, 3 Holding, 4 Opening
constexpr std::size_t kOffCatchResult = 292;  // u32, 0 None, 1 Caught, 2 ArmStrike, 3 TowerStrike

using WireSnapshot = std::array<uint8_t, kSnapshotWireSize>;

void encodeSnapshot(const vehicle::TruthSnapshot& snapshot, WireSnapshot& out) noexcept;

// Returns false on bad magic, version (only 3 is accepted), type or length.
bool decodeSnapshot(const uint8_t* data, std::size_t size, vehicle::TruthSnapshot& out);

}  // namespace sim::server
