#include "sim/vehicle/FlightSoftwareModel.hpp"

#include <utility>

namespace sim::vehicle {

void SimHal::bind(Bus& bus) {
    imu_ = &bus.channel<ImuMsg>(channels::kImu);
    altimeter_ = &bus.channel<AltimeterMsg>(channels::kAltimeter);
    gnss_ = &bus.channel<GnssMsg>(channels::kGnss);
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

fsw::GnssSample SimHal::readGnss() {
    fsw::GnssSample sample;
    const auto& msg = gnss_->latest();
    sample.valid = msg.valid() && msg.data.valid;
    sample.stamp = msg.stamp;
    sample.fixTime = msg.data.fixTime;
    sample.fixType = static_cast<fsw::GnssFixType>(msg.data.fixType);
    sample.latitude = msg.data.latitude;
    sample.longitude = msg.data.longitude;
    sample.height = msg.data.height;
    for (int i = 0; i < 3; ++i) {
        sample.velocityEnu[i] = msg.data.velocityEnu[i];
    }
    sample.horizontalSigma = msg.data.horizontalSigma;
    sample.verticalSigma = msg.data.verticalSigma;
    return sample;
}

void SimHal::commandActuators(const fsw::ActuatorCommand& command) {
    ActuatorCommandMsg msg;
    for (int i = 0; i < kEngineCount; ++i) {
        const fsw::EngineCommand& c = command.engines[i];
        msg.engines[i] = EngineCommandMsg{c.on, c.throttle, c.gimbalPitch, c.gimbalYaw};
    }
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
