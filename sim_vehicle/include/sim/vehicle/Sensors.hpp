#pragma once

#include <string>

#include <Eigen/Core>

#include "sim/core/Bus.hpp"
#include "sim/core/Model.hpp"
#include "sim/core/Random.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

// Error model parameters for an IMU. Defaults are generic tactical-grade
// placeholders, not any specific flight unit.
struct ImuSpec {
    double accelNoiseDensity = 1.0e-3;  // [m/s^2/sqrt(Hz)]
    double gyroNoiseDensity = 1.0e-4;   // [rad/s/sqrt(Hz)]
    double accelBiasSigma = 0.02;       // turn-on bias, 1 sigma [m/s^2]
    double gyroBiasSigma = 1.0e-4;      // turn-on bias, 1 sigma [rad/s]
    double accelBiasRandomWalk = 1.0e-4;  // [m/s^2/sqrt(s)]
    double gyroBiasRandomWalk = 1.0e-6;   // [rad/s/sqrt(s)]
    double accelQuantum = 1.0e-3;       // [m/s^2]
    double gyroQuantum = 1.0e-6;        // [rad/s]
    bool ideal = false;                 // true: perfect truth, for controller tuning
};

// IMU at the CG. TODO(phase 6, item 30): lever arm, misalignment, scale factor,
// saturation, latency, flexible-body pickup, failure modes, redundant units.
class ImuModel final : public Model {
public:
    ImuModel(std::string name, double rateHz, const HopperDynamics& truth, ImuSpec spec, uint64_t seed);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    const HopperDynamics& truth_;
    ImuSpec spec_;
    Rng rng_;
    Eigen::Vector3d accelBias_;
    Eigen::Vector3d gyroBias_;
    Channel<ImuMsg>* out_ = nullptr;
};

struct AltimeterSpec {
    double noiseSigma = 0.05;   // [m]
    double maxRange = 500.0;    // [m]
    double dropoutProbability = 0.0;  // per sample
    bool ideal = false;
};

// Radar altimeter measuring straight down. TODO(phase 6, item 30): beam
// geometry vs attitude, terrain, latency, multipath near the tower.
class RadarAltimeterModel final : public Model {
public:
    RadarAltimeterModel(std::string name, double rateHz, const HopperDynamics& truth, AltimeterSpec spec,
                        uint64_t seed);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    const HopperDynamics& truth_;
    AltimeterSpec spec_;
    Rng rng_;
    Channel<AltimeterMsg>* out_ = nullptr;
};

}  // namespace sim::vehicle
