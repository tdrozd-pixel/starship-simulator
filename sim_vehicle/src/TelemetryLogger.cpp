#include "sim/vehicle/TelemetryLogger.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim::vehicle {
namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
}  // namespace

TelemetryLogger::TelemetryLogger(std::string name, double rateHz, const HopperDynamics& truth, std::string path)
    : Model(std::move(name), rateHz), truth_(truth), path_(std::move(path)) {}

void TelemetryLogger::initialize(Bus& bus) {
    fsw_ = &bus.channel<FswTelemetryMsg>(channels::kFswTelemetry);
    commands_ = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
    const bool ok = csv_.open(path_, {"t", "altitude", "vertical_velocity", "east", "north", "east_velocity",
                                      "north_velocity", "tilt_deg", "mass", "propellant", "throttle_actual",
                                      "engine_running", "throttle_cmd", "fsw_mode", "alt_est", "vel_est",
                                      "alt_setpoint", "vel_setpoint", "mass_est", "east_est", "north_est",
                                      "east_setpoint", "north_setpoint", "east_accel_cmd", "north_accel_cmd",
                                      "gimbal_pitch_deg", "gimbal_yaw_deg", "gnss_fix"});
    if (!ok) {
        throw std::runtime_error("TelemetryLogger: cannot open " + path_);
    }
}

void TelemetryLogger::update(const StepContext& ctx) {
    const auto& s = truth_.state();
    const auto& tm = fsw_->latest().data;
    const auto& cmd = commands_->latest().data;
    // Truth columns are the vehicle base (what the flight software navigates).
    const Eigen::Vector3d base = truth_.basePosition();
    const Eigen::Vector3d baseVelocity = truth_.pointVelocityWorld(Eigen::Vector3d::Zero());
    const Eigen::Vector2d gimbal = truth_.meanGimbal();  // actual [rad]
    const double cosTilt = std::clamp((s.attitude * Eigen::Vector3d::UnitZ()).z(), -1.0, 1.0);
    csv_.row({ctx.t, base.z(), baseVelocity.z(), base.x(), base.y(), baseVelocity.x(), baseVelocity.y(),
              std::acos(cosTilt) * kRadToDeg, truth_.mass(), s.propellantMass,
              truth_.meanThrottle(), truth_.engineRunning() ? 1.0 : 0.0, cmd.engines[0].throttle,
              static_cast<double>(tm.mode), tm.altitudeEstimate, tm.verticalVelocityEstimate, tm.altitudeSetpoint,
              tm.velocitySetpoint, tm.massEstimate, tm.eastEstimate, tm.northEstimate, tm.eastSetpoint,
              tm.northSetpoint, tm.eastAccelCommand, tm.northAccelCommand, gimbal.x() * kRadToDeg,
              gimbal.y() * kRadToDeg, static_cast<double>(tm.gnssFixType)});
}

}  // namespace sim::vehicle
