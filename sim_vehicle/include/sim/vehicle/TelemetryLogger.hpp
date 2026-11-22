#pragma once

#include <string>

#include "sim/core/Bus.hpp"
#include "sim/core/CsvWriter.hpp"
#include "sim/core/Model.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

// Logs truth next to what the flight software believed, for plotting in Python.
class TelemetryLogger final : public Model {
public:
    TelemetryLogger(std::string name, double rateHz, const HopperDynamics& truth, std::string path);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    const HopperDynamics& truth_;
    std::string path_;
    CsvWriter csv_;
    const Channel<FswTelemetryMsg>* fsw_ = nullptr;
    const Channel<ActuatorCommandMsg>* commands_ = nullptr;
};

}  // namespace sim::vehicle
