#pragma once

#include <numbers>

namespace sim::vehicle {

constexpr double kDegToRad = std::numbers::pi / 180.0;

// Single-engine test hopper used to bring up the framework (phase 1 smoke test).
// These are PLACEHOLDER numbers for a generic vehicle, not Starship data.
// Real vehicle data and sources go in data/ with every assumption documented.
struct HopperParams {
    // Mass and geometry
    double dryMass = 20000.0;           // [kg]
    double initialPropellant = 10000.0;  // [kg]
    double radius = 4.5;                // [m], for inertia and drag area
    double length = 20.0;               // [m]
    double engineLeverArm = 8.0;        // [m], CG to gimbal point (placeholder: fixed)

    // Engine
    double maxThrust = 600e3;       // [N]
    double minThrottle = 0.40;      // fraction of max thrust
    double ispSeaLevel = 330.0;     // [s]
    double ignitionDelay = 0.5;     // [s] from command to thrust
    double throttleTimeConstant = 0.15;  // [s]
    double throttleRateLimit = 2.0;      // [1/s]

    // Thrust vector control
    double gimbalLimit = 15.0 * kDegToRad;          // [rad]
    double gimbalRateLimit = 20.0 * kDegToRad;      // [rad/s]
    double gimbalTimeConstant = 0.05;               // [s]

    // Aerodynamics (placeholder: constant axial drag)
    double dragCoefficient = 0.8;
};

}  // namespace sim::vehicle
