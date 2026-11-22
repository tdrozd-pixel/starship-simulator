#include "sim/vehicle/FlightSoftwareModel.hpp"

#include <utility>

namespace sim::vehicle {

void SimHal::bind(Bus& bus) {
    imu_ = &bus.channel<ImuMsg>(channels::kImu);
    altimeter_ = &bus.channel<AltimeterMsg>(channels::kAltimeter);
    commands_ = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
    telemetry_ = &bus.channel<FswTelemetryMsg>(channels::kFswTelemetry);
}

fsw::ImuSample SimHal::readImu() {
    fsw::ImuSample sample;
    const auto& msg = imu_->latest();
    sample.valid = msg.valid();
    sample.stamp = msg.stamp;
    for (int i = 0; i < 3; ++i) {
        sample.specificForce[i] = msg.data.specificForce[i];
        sample.angularRate[i] = msg.data.angularRate[i];
    }
    return sample;
}

fsw::AltimeterSample SimHal::readAltimeter() {
    fsw::AltimeterSample sample;
    const auto& msg = altimeter_->latest();
    sample.valid = msg.valid() && msg.data.valid;
    sample.stamp = msg.stamp;
    sample.range = msg.data.range;
    return sample;
}

void SimHal::commandActuators(const fsw::ActuatorCommand& command) {
    ActuatorCommandMsg msg;
    msg.engineOn = command.engineOn;
    msg.throttle = command.throttle;
    msg.gimbalPitch = command.gimbalPitch;
    msg.gimbalYaw = command.gimbalYaw;
    commands_->publish(msg, time_);
}

void SimHal::publishTelemetry(const fsw::Telemetry& telemetry) { telemetry_->publish(telemetry, time_); }

FlightSoftwareModel::FlightSoftwareModel(std::string name, const fsw::HopConfig& config)
    : Model(std::move(name), config.rateHz), config_(config) {}

void FlightSoftwareModel::initialize(Bus& bus) {
    hal_.bind(bus);
    computer_ = std::make_unique<fsw::HopFlightComputer>(hal_, config_);
}

void FlightSoftwareModel::update(const StepContext& ctx) {
    hal_.setTime(ctx.t);
    computer_->tick();
}

}  // namespace sim::vehicle
