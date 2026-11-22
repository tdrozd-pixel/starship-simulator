#pragma once

#include <cstddef>
#include <cstdint>

#include "sim/server/TelemetryRing.hpp"

// Ground frame encoding of the telemetry stream, version 1. Layout is fixed by
// docs/protocol/telemetry_stream.md. The 24-byte header is little-endian; the
// CCSDS packet that follows is copied unchanged.
namespace sim::server {

constexpr std::size_t kGroundHeaderSize = 24;
constexpr std::size_t kMaxGroundFrame = kGroundHeaderSize + vehicle::kMaxTelemetryPacket;
constexpr uint16_t kTelemetryProtocolVersion = 1;
constexpr uint16_t kFrameTelemetryPacket = 1;
constexpr std::size_t kMinCcsdsPacket = 6;  // primary header

// APID of a CCSDS packet (11 bits of the big-endian primary header). `length` must be >= 2.
inline uint16_t packetApid(const uint8_t* packet) noexcept {
    return static_cast<uint16_t>(((packet[0] & 0x07u) << 8) | packet[1]);
}

// Writes the frame into `out` (at least kMaxGroundFrame bytes); returns its size.
std::size_t encodeGroundFrame(const TelemetryItem& item, uint8_t* out) noexcept;

// Returns false on bad magic, version, frame type or length.
bool decodeGroundFrame(const uint8_t* data, std::size_t size, TelemetryItem& out);

}  // namespace sim::server
