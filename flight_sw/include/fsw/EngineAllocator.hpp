#pragma once

// Control allocation for the three sea-level Raptors: turns a requested body torque and a
// common throttle into per-engine throttle and gimbal commands. A pure function (no state,
// no allocation) so it can be tested on its own.
//
// Axes: body x/y are pitch/yaw (lateral), body z is the vehicle axis (roll).
//  - Roll (Mz): thrust is axial, so differential throttle cannot make roll torque. Roll comes
//    only from tangential gimbal on all three engines, torque = N * R * T * angle. The angle
//    is limited to `rollGimbalBudget`, and that part of the gimbal range is RESERVED: pitch
//    and yaw common gimbal may only use `gimbalLimit - rollGimbalBudget`, so roll always has
//    headroom (roll has priority).
//  - Pitch/yaw (Mx, My): common gimbal first, torque = L * T_total * lateral fraction.
//    When that saturates, the remainder is made by differential throttle across the ring:
//    thrust offsets dT_i with sum dT_i = 0 (total thrust preserved) and the per-engine
//    thrust kept between the minimum-throttle floor and full throttle (the whole offset set
//    is scaled down together, so the direction is kept). At the throttle floor there is no
//    downward headroom, so no differential throttle is available there.
//  - Whatever cannot be achieved is reported in `residualTorque`.
// Assumes the engines form an equally spaced ring (true for the Ship's three sea-level engines).

#include "fsw/Hal.hpp"

namespace fsw {

struct AllocatorConfig {
    double leverArm = 0.0;           // [m] CG to the gimbal plane
    double ringRadius = 0.0;         // [m] engine distance from the axis
    double azimuth0 = 0.0;           // [rad] azimuth of engine 0 from body +x toward +y
    double gimbalLimit = 0.0;        // [rad] per engine, total
    double rollGimbalBudget = 0.0;   // [rad] tangential gimbal reserved for roll
    double vacuumThrust = 0.0;       // [N] per engine at full rated flow
    double nozzleExitArea = 0.0;     // [m^2] per engine
    double ambientPressure = 101325.0;  // [Pa] assumed
    double minThrottle = 0.0;        // fraction of rated flow
};

struct AllocatorRequest {
    double throttle = 0.0;     // common throttle from the vertical loop [fraction]
    double torque[3] = {};     // requested body torque about the CG [N m]
};

struct AllocatorResult {
    double throttle[kSeaLevelEngineCount] = {};  // [fraction]
    double pitch[kSeaLevelEngineCount] = {};     // gimbal [rad], about body x
    double yaw[kSeaLevelEngineCount] = {};       // gimbal [rad], about body y
    double residualTorque[3] = {};               // requested minus achievable [N m]
    bool gimbalSaturated = false;           // pitch/yaw common gimbal hit its cap
    bool rollSaturated = false;             // roll hit its reserved budget
    bool differentialThrottleUsed = false;  // pitch/yaw remainder given to differential throttle
};

AllocatorResult allocateSeaLevelEngines(const AllocatorConfig& config, const AllocatorRequest& request);

}  // namespace fsw
