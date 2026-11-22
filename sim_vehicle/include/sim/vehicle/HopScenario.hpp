#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Simulation.hpp"
#include "sim/env/Weather.hpp"
#include "sim/vehicle/FlightSoftwareModel.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/HopperParams.hpp"
#include "sim/vehicle/Sensors.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"
#include "sim/vehicle/TruthSnapshotPublisher.hpp"

namespace sim::vehicle {

struct HopScenarioOptions {
    HopScenarioOptions() {
        weather.gusts.push_back({Eigen::Vector3d(100.0, 0.0, 0.0), 8.0, 1.0});
    }

    uint64_t seed = 1;
    bool idealSensors = false;  // "truth mode": perfect sensors for tuning control
    std::string logPath;        // empty: no CSV log
    double logRateHz = 100.0;
    double imuRateHz = 500.0;
    double altimeterRateHz = 50.0;
    double gnssModelRateHz = 100.0;  // times the GNSS latency; solutions come at gnss.outputRateHz
    HopperParams vehicle;
    LaunchSite site;
    fsw::HopConfig hop;         // vehicle-knowledge fields are filled from `vehicle`
    TruthSnapshotSink* snapshotSink = nullptr;  // non-owning; null: no publisher
    double snapshotRateHz = 100.0;
    ImuSpec imu;
    AltimeterSpec altimeter;
    GnssSpec gnss;
    env::WeatherParameters weather;
    Eigen::Quaterniond initialAttitude = Eigen::Quaterniond::Identity();
    Eigen::Vector3d initialAngularRate = Eigen::Vector3d::Zero();
};

struct HopScenario {
    std::unique_ptr<env::LocalTangentFrame> site;  // sim ENU origin = landing target
    std::unique_ptr<Simulation> sim;
    HopperDynamics* vehicle = nullptr;
    FlightSoftwareModel* fsw = nullptr;
    env::WeatherModel* weather = nullptr;
    TruthSnapshotPublisher* snapshotPublisher = nullptr;  // null when no sink was given
};

// Builds and initializes the hop: vehicle on the ground at the liftoff point, IMU,
// radar altimeter, GNSS, flight software, optional logger. The sim ENU origin is
// the landing target (site.target), so the target is at east = north = 0.
HopScenario buildHopScenario(const HopScenarioOptions& options);

}  // namespace sim::vehicle
