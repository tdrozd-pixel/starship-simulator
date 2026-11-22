#pragma once

#include <Eigen/Core>

#include "fsw/Hal.hpp"
#include "sim/vehicle/HopperParams.hpp"

// Bus message types and channel names shared by the vehicle models.
namespace sim::vehicle {

static_assert(kEngineCount == fsw::kEngineCount, "flight software and vehicle disagree on engine count");

struct ImuMsg {
    Eigen::Vector3d specificForce = Eigen::Vector3d::Zero();  // body [m/s^2]
    Eigen::Vector3d angularRate = Eigen::Vector3d::Zero();    // body [rad/s]
};

struct AltimeterMsg {
    double range = 0.0;  // slant range along body -z [m]
    bool valid = false;
};

struct GnssMsg {
    bool valid = false;
    int fixType = 0;         // fsw::GnssFixType values
    double fixTime = 0.0;    // time of validity [s]
    double latitude = 0.0;   // WGS-84 [rad]
    double longitude = 0.0;  // [rad]
    double height = 0.0;     // ellipsoidal [m]
    Eigen::Vector3d velocityEnu = Eigen::Vector3d::Zero();  // [m/s]
    double horizontalSigma = 0.0;  // receiver-reported 1 sigma [m]
    double verticalSigma = 0.0;    // [m]
};

struct EngineCommandMsg {
    bool on = false;
    double throttle = 0.0;     // fraction of rated flow
    double gimbalPitch = 0.0;  // [rad]
    double gimbalYaw = 0.0;    // [rad]
};

struct ActuatorCommandMsg {
    EngineCommandMsg engines[kEngineCount];
};

using FswTelemetryMsg = fsw::Telemetry;

namespace channels {
inline constexpr const char* kImu = "sensors/imu";
inline constexpr const char* kAltimeter = "sensors/radar_altimeter";
inline constexpr const char* kGnss = "sensors/gnss";
inline constexpr const char* kActuatorCmd = "fsw/actuator_cmd";
inline constexpr const char* kFswTelemetry = "fsw/telemetry";
}  // namespace channels

}  // namespace sim::vehicle
