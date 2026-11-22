#pragma once

#include <Eigen/Core>

#include "fsw/Hal.hpp"

// Bus message types and channel names shared by the vehicle models.
namespace sim::vehicle {

struct ImuMsg {
    Eigen::Vector3d specificForce = Eigen::Vector3d::Zero();  // body [m/s^2]
    Eigen::Vector3d angularRate = Eigen::Vector3d::Zero();    // body [rad/s]
};

struct AltimeterMsg {
    double range = 0.0;  // [m]
    bool valid = false;
};

struct ActuatorCommandMsg {
    bool engineOn = false;
    double throttle = 0.0;
    double gimbalPitch = 0.0;
    double gimbalYaw = 0.0;
};

using FswTelemetryMsg = fsw::Telemetry;

namespace channels {
inline constexpr const char* kImu = "sensors/imu";
inline constexpr const char* kAltimeter = "sensors/radar_altimeter";
inline constexpr const char* kActuatorCmd = "fsw/actuator_cmd";
inline constexpr const char* kFswTelemetry = "fsw/telemetry";
}  // namespace channels

}  // namespace sim::vehicle
