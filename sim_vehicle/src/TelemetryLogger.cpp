#include "sim/vehicle/TelemetryLogger.hpp"

#include <stdexcept>
#include <utility>

namespace sim::vehicle {

TelemetryLogger::TelemetryLogger(std::string name, double rateHz, const HopperDynamics& truth, std::string path)
    : Model(std::move(name), rateHz), truth_(truth), path_(std::move(path)) {}

void TelemetryLogger::initialize(Bus& bus) {
    fsw_ = &bus.channel<FswTelemetryMsg>(channels::kFswTelemetry);
    commands_ = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
    const bool ok = csv_.open(path_, {"t", "altitude", "vertical_velocity", "mass", "propellant", "throttle_actual",
                                      "engine_running", "throttle_cmd", "fsw_mode", "alt_est", "vel_est",
                                      "alt_setpoint", "vel_setpoint", "mass_est"});
    if (!ok) {
        throw std::runtime_error("TelemetryLogger: cannot open " + path_);
    }
}

void TelemetryLogger::update(const StepContext& ctx) {
    const auto& s = truth_.state();
    const auto& tm = fsw_->latest().data;
    const auto& cmd = commands_->latest().data;
    csv_.row({ctx.t, s.position.z(), s.velocity.z(), truth_.mass(), s.propellantMass, s.throttle,
              truth_.engineRunning() ? 1.0 : 0.0, cmd.throttle, static_cast<double>(tm.mode), tm.altitudeEstimate,
              tm.verticalVelocityEstimate, tm.altitudeSetpoint, tm.velocitySetpoint, tm.massEstimate});
}

}  // namespace sim::vehicle
