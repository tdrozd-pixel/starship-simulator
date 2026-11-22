#pragma once

#include <memory>
#include <string>

#include "fsw/Hal.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Bus.hpp"
#include "sim/core/Model.hpp"
#include "sim/vehicle/Downlink.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

// Simulated hardware for the flight software: reads sensor channels, writes
// actuator commands and telemetry back onto the bus. This is the seam where a
// real flight computer would plug in for hardware-in-the-loop testing.
class SimHal final : public fsw::Hal {
public:
    void bind(Bus& bus);
    void setTime(double t) { time_ = t; }
    // Attach the onboard downlink queue (non-owning). Without one, packets are dropped.
    void setDownlinkQueue(DownlinkQueue* queue) { downlinkQueue_ = queue; }

    double timeSec() const override { return time_; }
    fsw::ImuSample readImu() override;
    fsw::AltimeterSample readAltimeter() override;
    fsw::GnssSample readGnss() override;
    void commandActuators(const fsw::ActuatorCommand& command) override;
    fsw::GimbalFeedbackSample readGimbalFeedback() override;
    void publishTelemetry(const fsw::Telemetry& telemetry) override;
    void downlink(const uint8_t* packet, std::size_t length) override;

private:
    double time_ = 0.0;
    const Channel<ImuMsg>* imu_ = nullptr;
    const Channel<AltimeterMsg>* altimeter_ = nullptr;
    const Channel<GnssMsg>* gnss_ = nullptr;
    const Channel<GimbalFeedbackMsg>* gimbalFeedback_ = nullptr;
    DownlinkQueue* downlinkQueue_ = nullptr;
    Channel<ActuatorCommandMsg>* commands_ = nullptr;
    Channel<FswTelemetryMsg>* telemetry_ = nullptr;
};

// Runs the flight software at its own rate inside the simulation, in lockstep.
class FlightSoftwareModel final : public Model {
public:
    FlightSoftwareModel(std::string name, const fsw::HopConfig& config);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

    const fsw::HopFlightComputer& computer() const { return *computer_; }
    void setDownlinkQueue(DownlinkQueue* queue) { hal_.setDownlinkQueue(queue); }

private:
    fsw::HopConfig config_;
    SimHal hal_;
    std::unique_ptr<fsw::HopFlightComputer> computer_;
};

}  // namespace sim::vehicle
