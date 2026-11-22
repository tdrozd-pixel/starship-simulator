#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Simulation.hpp"
#include "sim/vehicle/FlightSoftwareModel.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/HopperParams.hpp"
#include "sim/vehicle/Sensors.hpp"

namespace sim::vehicle {

struct HopScenarioOptions {
    uint64_t seed = 1;
    bool idealSensors = false;  // "truth mode": perfect sensors for tuning control
    std::string logPath;        // empty: no CSV log
    double logRateHz = 100.0;
    double imuRateHz = 500.0;
    double altimeterRateHz = 50.0;
    HopperParams vehicle;
    fsw::HopConfig hop;         // vehicle-knowledge fields are filled from `vehicle`
    ImuSpec imu;
    AltimeterSpec altimeter;
};

struct HopScenario {
    std::unique_ptr<Simulation> sim;
    HopperDynamics* vehicle = nullptr;
    FlightSoftwareModel* fsw = nullptr;
};

// Builds and initializes the vertical hop: vehicle on the pad, IMU, radar
// altimeter, flight software, optional logger.
HopScenario buildHopScenario(const HopScenarioOptions& options);

}  // namespace sim::vehicle
