#pragma once

#include <cstdint>

namespace sim::vehicle {

// Host representation of one truth snapshot for visual clients. The wire
// encoding lives in sim_server; this layer knows nothing about networking.
// Frames: world is local ENU (z up), body +z toward the nose.
struct TruthSnapshot {
    uint32_t runId = 0;  // stamped by the sink, publisher leaves 0
    uint32_t flags = 0;  // kSnap* bits
    uint64_t step = 0;   // sim step count since run start
    double simTime = 0.0;  // [s]
    double position[3] = {};     // world [m]
    double velocity[3] = {};     // world [m/s]
    double attitude[4] = {1.0, 0.0, 0.0, 0.0};  // w, x, y, z, body to world
    double angularRate[3] = {};  // body [rad/s]
    double throttle = 0.0;       // ACTUAL, fraction of max thrust
    double gimbalPitch = 0.0;    // ACTUAL [rad]
    double gimbalYaw = 0.0;      // ACTUAL [rad]
    double propellantMass = 0.0;  // [kg]
    double totalMass = 0.0;       // [kg]
};

constexpr uint32_t kSnapEngineRunning = 1u << 0;
constexpr uint32_t kSnapTouchdown = 1u << 1;
constexpr uint32_t kSnapHalted = 1u << 2;

// Receives snapshots on the sim thread. Implementations must not block or allocate.
class TruthSnapshotSink {
public:
    virtual ~TruthSnapshotSink() = default;
    virtual void push(const TruthSnapshot& snapshot) noexcept = 0;
};

}  // namespace sim::vehicle
