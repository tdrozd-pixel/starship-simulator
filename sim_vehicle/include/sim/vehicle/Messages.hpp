#pragma once

#include <Eigen/Core>

#include "fsw/Hal.hpp"
#include "sim/vehicle/HopperParams.hpp"

// Bus message types and channel names shared by the vehicle models.
namespace sim::vehicle {

static_assert(kEngineCount == fsw::kEngineCount && kSeaLevelEngineCount == fsw::kSeaLevelEngineCount,
              "flight software and vehicle disagree on engine count");

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
    int satellitesVisible = 0;
    int satellitesUsed = 0;
};

struct GimbalFeedbackMsg {
    bool valid[kSeaLevelEngineCount] = {};
    double pitch[kSeaLevelEngineCount] = {};  // measured, about body x [rad]
    double yaw[kSeaLevelEngineCount] = {};    // measured, about body y [rad]
};

// Measured engine throttle and controller state, all six engines.
struct EngineFeedbackMsg {
    bool valid[kEngineCount] = {};
    double throttle[kEngineCount] = {};  // measured actual [fraction of rated flow]
    fsw::EngineState state[kEngineCount] = {};
};

struct EngineCommandMsg {
    bool on = false;
    double throttle = 0.0;  // fraction of rated flow
};

struct GimbalCommandMsg {
    double pitch = 0.0;  // [rad]
    double yaw = 0.0;    // [rad]
};

// Six engines (0..2 sea level, 3..5 RVac); gimbals only for the three sea-level engines.
struct ActuatorCommandMsg {
    EngineCommandMsg engines[kEngineCount];
    GimbalCommandMsg gimbals[kSeaLevelEngineCount];
};

using FswTelemetryMsg = fsw::Telemetry;

namespace channels {
inline constexpr const char* kImu = "sensors/imu";
inline constexpr const char* kAltimeter = "sensors/radar_altimeter";
inline constexpr const char* kGnss = "sensors/gnss";
inline constexpr const char* kGimbalFeedback = "sensors/gimbal_feedback";
inline constexpr const char* kEngineFeedback = "sensors/engine_feedback";
inline constexpr const char* kActuatorCmd = "fsw/actuator_cmd";
inline constexpr const char* kFswTelemetry = "fsw/telemetry";
}  // namespace channels

}  // namespace sim::vehicle
