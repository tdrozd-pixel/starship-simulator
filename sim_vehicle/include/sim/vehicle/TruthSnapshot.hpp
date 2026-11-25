#pragma once

#include <cstdint>

#include "sim/vehicle/HopperParams.hpp"

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
    // Per engine, ACTUAL values. Index 0..2 sea-level Raptors, 3..5 Raptor Vacuums.
    // throttle/gimbalPitch/gimbalYaw above stay the mean over the three sea-level engines.
    double engineThrottle[kEngineCount] = {};              // fraction of rated flow
    double engineGimbalPitch[kSeaLevelEngineCount] = {};   // [rad], sea-level engines only (RVacs are fixed)
    double engineGimbalYaw[kSeaLevelEngineCount] = {};     // [rad]
    bool engineRunning[kEngineCount] = {};                 // producing thrust
    // Catch tower (zero and Open/None when there is no tower).
    double armAngle[2] = {};     // ACTUAL arm closure angle psi, from open [rad], arm 0 then arm 1
    uint32_t towerState = 0;     // 0 Open, 1 Closing, 2 Closed, 3 Holding, 4 Opening
    uint32_t catchResult = 0;    // 0 None, 1 Caught, 2 ArmStrike, 3 TowerStrike
};

constexpr uint32_t kSnapEngineRunning = 1u << 0;
constexpr uint32_t kSnapTouchdown = 1u << 1;
constexpr uint32_t kSnapHalted = 1u << 2;
constexpr uint32_t kSnapCaught = 1u << 9;   // caught by the tower arms (bits 3 to 8 are not used yet)
constexpr uint32_t kSnapCrashed = 1u << 10; // crashed into the tower or arms (placeholder)

// Receives snapshots on the sim thread. Implementations must not block or allocate.
class TruthSnapshotSink {
public:
    virtual ~TruthSnapshotSink() = default;
    virtual void push(const TruthSnapshot& snapshot) noexcept = 0;
};

}  // namespace sim::vehicle
