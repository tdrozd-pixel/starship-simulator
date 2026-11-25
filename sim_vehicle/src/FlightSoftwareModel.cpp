#include "sim/vehicle/FlightSoftwareModel.hpp"

#include <utility>

namespace sim::vehicle {

void SimHal::bind(Bus& bus) {
    imu_ = &bus.channel<ImuMsg>(channels::kImu);
    altimeter_ = &bus.channel<AltimeterMsg>(channels::kAltimeter);
    gnss_ = &bus.channel<GnssMsg>(channels::kGnss);
    gimbalFeedback_ = &bus.channel<GimbalFeedbackMsg>(channels::kGimbalFeedback);
    engineFeedback_ = &bus.channel<EngineFeedbackMsg>(channels::kEngineFeedback);
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
    sample.satellitesVisible = msg.data.satellitesVisible;
    sample.satellitesUsed = msg.data.satellitesUsed;
    return sample;
}

fsw::GimbalFeedbackSample SimHal::readGimbalFeedback() {
    fsw::GimbalFeedbackSample sample;
    const auto& msg = gimbalFeedback_->latest();
    sample.stamp = msg.stamp;
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        sample.valid[i] = msg.valid() && msg.data.valid[i];
        sample.pitch[i] = msg.data.pitch[i];
        sample.yaw[i] = msg.data.yaw[i];
    }
    return sample;
}

fsw::EngineFeedbackSample SimHal::readEngineFeedback() {
    fsw::EngineFeedbackSample sample;
    const auto& msg = engineFeedback_->latest();
    sample.stamp = msg.stamp;
    for (int i = 0; i < kEngineCount; ++i) {
        sample.valid[i] = msg.valid() && msg.data.valid[i];
        sample.throttle[i] = msg.data.throttle[i];
        sample.state[i] = msg.data.state[i];
    }
    return sample;
}

void SimHal::commandActuators(const fsw::ActuatorCommand& command) {
    ActuatorCommandMsg msg;
    for (int i = 0; i < kEngineCount; ++i) {
        const fsw::EngineCommand& c = command.engines[i];
        msg.engines[i] = EngineCommandMsg{c.on, c.throttle};
    }
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        msg.gimbals[i] = GimbalCommandMsg{command.gimbals[i].pitch, command.gimbals[i].yaw};
    }
    commands_->publish(msg, time_);
}

void SimHal::publishTelemetry(const fsw::Telemetry& telemetry) { telemetry_->publish(telemetry, time_); }

void SimHal::downlink(const uint8_t* packet, std::size_t length) {
    if (downlinkQueue_ != nullptr) {
        downlinkQueue_->push(packet, length, time_);
    }
}

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
