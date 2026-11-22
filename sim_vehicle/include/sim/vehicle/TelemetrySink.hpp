#pragma once

#include <cstddef>
#include <cstdint>

namespace sim::vehicle {

// Largest CCSDS packet the flight computer sends (docs/telemetry/gnc_packets.md).
constexpr std::size_t kMaxTelemetryPacket = 256;

// Receives downlinked telemetry on the ground side of the comms model (DownlinkModel),
// on the sim thread. `packet` is a complete CCSDS Space Packet; `groundTime` is the sim
// time at which the ground received it [s]. Implementations must not block or allocate.
class TelemetrySink {
public:
    virtual ~TelemetrySink() = default;
    virtual void deliver(const uint8_t* packet, std::size_t length, double groundTime) noexcept = 0;
};

}  // namespace sim::vehicle
