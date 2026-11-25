#include "fsw/EngineAllocator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace fsw {

AllocatorResult allocateSeaLevelEngines(const AllocatorConfig& c, const AllocatorRequest& r) {
    constexpr int kN = kSeaLevelEngineCount;
    AllocatorResult out;
    const double commonThrottle = std::clamp(std::max(c.minThrottle, r.throttle), 0.0, 1.0);
    for (int i = 0; i < kN; ++i) {
        out.throttle[i] = commonThrottle;
    }
    for (int i = 0; i < 3; ++i) {
        out.residualTorque[i] = r.torque[i];
    }
    if (c.leverArm <= 0.0 || c.vacuumThrust <= 0.0 || c.gimbalLimit <= 0.0) {
        return out;
    }

    const double lossPerEngine = c.ambientPressure * c.nozzleExitArea;  // [N]
    const double thrust0 = std::max(1.0, commonThrottle * c.vacuumThrust - lossPerEngine);  // [N] per engine
    const double totalThrust = kN * thrust0;

    // Roll: tangential gimbal, same angle on every engine, limited to the reserved budget.
    double roll = 0.0;  // [rad]
    if (c.ringRadius > 0.0) {
        const double wanted = r.torque[2] / (kN * c.ringRadius * thrust0);
        roll = std::clamp(wanted, -c.rollGimbalBudget, c.rollGimbalBudget);
        out.rollSaturated = roll != wanted;
        out.residualTorque[2] = r.torque[2] - roll * kN * c.ringRadius * thrust0;
    }

    // Pitch/yaw by common gimbal. Lateral thrust fraction u = (sin yaw, -sin pitch):
    // torque_x = L * T * u_y, torque_y = -L * T * u_x.
    const double cap = std::max(0.0, c.gimbalLimit - c.rollGimbalBudget);
    double ux = -r.torque[1] / (c.leverArm * totalThrust);
    double uy = r.torque[0] / (c.leverArm * totalThrust);
    const double uMax = std::sin(cap);
    const double uNorm = std::hypot(ux, uy);
    if (uNorm > uMax) {
        out.gimbalSaturated = true;
        ux *= uMax / uNorm;
        uy *= uMax / uNorm;
    }
    out.residualTorque[0] = r.torque[0] - c.leverArm * totalThrust * uy;
    out.residualTorque[1] = r.torque[1] + c.leverArm * totalThrust * ux;

    // Remainder by differential throttle: dT_i = 2 / (N R^2) * (y_i Mx - x_i My) gives
    // Mx = sum y_i dT_i, My = -sum x_i dT_i and sum dT_i = 0 on a symmetric ring.
    if (c.ringRadius > 0.0 && out.gimbalSaturated) {
        const double thrustMin = std::max(0.0, c.minThrottle * c.vacuumThrust - lossPerEngine);
        const double thrustMax = std::max(thrustMin, c.vacuumThrust - lossPerEngine);
        const double down = std::max(0.0, thrust0 - thrustMin);
        const double up = std::max(0.0, thrustMax - thrust0);
        double delta[kN];
        double scale = 1.0;
        for (int i = 0; i < kN; ++i) {
            const double az = c.azimuth0 + 2.0 * std::numbers::pi * i / kN;
            const double x = c.ringRadius * std::cos(az);
            const double y = c.ringRadius * std::sin(az);
            delta[i] = 2.0 / (kN * c.ringRadius * c.ringRadius) *
                       (y * out.residualTorque[0] - x * out.residualTorque[1]);
            const double room = delta[i] > 0.0 ? up : down;
            if (std::abs(delta[i]) > room) {
                scale = std::min(scale, room / std::abs(delta[i]));
            }
        }
        double mx = 0.0;
        double my = 0.0;
        for (int i = 0; i < kN; ++i) {
            const double az = c.azimuth0 + 2.0 * std::numbers::pi * i / kN;
            const double d = delta[i] * scale;
            out.throttle[i] = std::clamp(commonThrottle + d / c.vacuumThrust, 0.0, 1.0);
            mx += c.ringRadius * std::sin(az) * d;
            my -= c.ringRadius * std::cos(az) * d;
        }
        out.differentialThrottleUsed = scale > 0.0;
        out.residualTorque[0] -= mx;
        out.residualTorque[1] -= my;
    }

    // Per-engine gimbal: common lateral direction plus the tangential roll tilt.
    for (int i = 0; i < kN; ++i) {
        const double az = c.azimuth0 + 2.0 * std::numbers::pi * i / kN;
        const double lx = ux - roll * std::sin(az);
        const double ly = uy + roll * std::cos(az);
        const double pitch = -std::asin(std::clamp(ly, -1.0, 1.0));
        const double yaw = std::asin(std::clamp(lx / std::cos(pitch), -1.0, 1.0));
        out.pitch[i] = std::clamp(pitch, -c.gimbalLimit, c.gimbalLimit);
        out.yaw[i] = std::clamp(yaw, -c.gimbalLimit, c.gimbalLimit);
    }
    return out;
}

}  // namespace fsw
