#pragma once

#include <memory>
#include <string>

#include "fsw/Hal.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Bus.hpp"
#include "sim/core/Model.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

// Simulated hardware for the flight software: reads sensor channels, writes
// actuator commands and telemetry back onto the bus. This is the seam where a
// real flight computer would plug in for hardware-in-the-loop testing.
class SimHal final : public fsw::Hal {
public:
    void bind(Bus& bus);
    void setTime(double t) { time_ = t; }

    double timeSec() const override { return time_; }
    fsw::ImuSample readImu() override;
    fsw::AltimeterSample readAltimeter() override;
    void commandActuators(const fsw::ActuatorCommand& command) override;
    void publishTelemetry(const fsw::Telemetry& telemetry) override;

private:
    double time_ = 0.0;
    const Channel<ImuMsg>* imu_ = nullptr;
    const Channel<AltimeterMsg>* altimeter_ = nullptr;
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

private:
    fsw::HopConfig config_;
    SimHal hal_;
    std::unique_ptr<fsw::HopFlightComputer> computer_;
};

}  // namespace sim::vehicle
