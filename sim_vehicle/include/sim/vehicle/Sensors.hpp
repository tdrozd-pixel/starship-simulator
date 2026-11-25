#pragma once

#include <string>

#include <Eigen/Core>

#include "sim/core/Bus.hpp"
#include "sim/core/Model.hpp"
#include "sim/core/Random.hpp"
#include "sim/env/Geodesy.hpp"
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

// Radar altimeter at the base, measuring slant range along body -z to flat ground.
// TODO(phase 6, item 30): beam width, terrain, latency, multipath near the tower.
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

// GNSS receiver error model. Public sources describe Starship navigation as GNSS
// plus inertial; one unverified report says RTK-grade (centimeter) differential
// GPS is used near the tower. Values below are generic receiver figures
// (estimates), not a SpaceX unit.
struct GnssSpec {
    int fixType = 2;                  // fsw::GnssFixType: 1 standalone, 2 RTK fixed
    double outputRateHz = 10.0;       // solutions per second (must divide the model rate)
    double latency = 0.05;            // [s] fix time of validity to output
    // Errors: slowly varying (first-order Gauss-Markov) plus white noise, per axis.
    double horizontalBiasSigma = 0.01;  // [m]
    double verticalBiasSigma = 0.02;    // [m]
    double biasTimeConstant = 30.0;     // [s]
    double horizontalNoiseSigma = 0.008;  // [m]
    double verticalNoiseSigma = 0.015;    // [m]
    double velocityNoiseSigma = 0.02;     // [m/s] Doppler velocity, per axis
    double reportedHorizontalSigma = 0.015;  // [m] what the receiver claims
    double reportedVerticalSigma = 0.03;     // [m]
    double angleQuantum = 1.0e-10;      // [rad] latitude/longitude output resolution (~0.6 mm)
    double dropoutProbability = 0.0;    // per solution
    bool ideal = false;                 // true: exact antenna truth, no latency

    // Single-point GPS, no corrections: meter-level, slowly wandering.
    static GnssSpec standalone() {
        GnssSpec s;
        s.fixType = 1;
        s.horizontalBiasSigma = 1.2;
        s.verticalBiasSigma = 2.5;
        s.biasTimeConstant = 300.0;
        s.horizontalNoiseSigma = 0.3;
        s.verticalNoiseSigma = 0.6;
        s.velocityNoiseSigma = 0.05;
        s.reportedHorizontalSigma = 1.5;
        s.reportedVerticalSigma = 3.0;
        return s;
    }
};

// GNSS receiver with its antenna at `antennaFromBase` (body frame). Truth is the
// sim ENU position of the antenna, tied to WGS-84 through the site frame.
// Runs at its model rate to time the latency, and outputs at outputRateHz.
// TODO(phase 6, item 30): multipath near the tower, satellite geometry, plume and
// plasma attenuation, cycle slips, loss of RTK fix.
class GnssModel final : public Model {
public:
    GnssModel(std::string name, double rateHz, const HopperDynamics& truth, const env::LocalTangentFrame& site,
              const Eigen::Vector3d& antennaFromBase, GnssSpec spec, uint64_t seed,
              uint64_t constellationSeed = 0);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    double gaussMarkovStep(double value, double sigma, double period);
    void stepConstellation();

    const HopperDynamics& truth_;
    const env::LocalTangentFrame& site_;
    Eigen::Vector3d antenna_;
    GnssSpec spec_;
    Rng rng_;
    // Satellite counts: placeholder random walk on its own stream so the error
    // sequence above (and therefore every flight) is unchanged.
    // TODO(phase 6, item 30): real almanac geometry.
    Rng constellationRng_;
    int satellitesVisible_ = 24;  // [count]
    int stepsPerOutput_ = 1;
    int stepCounter_ = 0;
    Eigen::Vector3d bias_ = Eigen::Vector3d::Zero();  // ENU [m]
    bool pending_ = false;
    GnssMsg pendingFix_;
    Channel<GnssMsg>* out_ = nullptr;
};

}  // namespace sim::vehicle

namespace sim::vehicle {

// Gimbal position sensor (resolver or encoder on each actuator). Values are
// estimates, not a SpaceX part.
struct GimbalFeedbackSpec {
    double noiseSigma = 1.7e-4;  // [rad] 1 sigma, estimate
    double biasSigma = 3.5e-4;   // [rad] per-axis turn-on bias, 1 sigma, estimate
    double quantum = 2.0 * 3.14159265358979323846 / 65536.0;  // [rad] 16-bit encoder, estimate
    bool ideal = false;          // true: exact truth, no latency
};

// Publishes the actual gimbal positions with one sample of latency (build order item 33).
// TODO(phase 6, item 33): failures, saturation, per-sensor mounting.
class GimbalFeedbackModel final : public Model {
public:
    GimbalFeedbackModel(std::string name, double rateHz, const HopperDynamics& truth, GimbalFeedbackSpec spec,
                        uint64_t seed);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    const HopperDynamics& truth_;
    GimbalFeedbackSpec spec_;
    Rng rng_;
    EngineGimbals bias_;
    GimbalFeedbackMsg previous_;
    bool havePrevious_ = false;
    Channel<GimbalFeedbackMsg>* out_ = nullptr;
};

// Engine controller feedback: measured throttle (flow) and engine state for all six engines.
// Values are estimates, not a SpaceX part. State is what the engine controller reports.
struct EngineFeedbackSpec {
    double noiseSigma = 0.002;  // [fraction of rated flow] 1 sigma, estimate
    double quantum = 1.0 / 4096.0;  // [fraction] 12-bit, estimate
    bool ideal = false;         // true: exact truth
};

// TODO(phase 6, item 33): failures, latency, per-engine mounting.
class EngineFeedbackModel final : public Model {
public:
    EngineFeedbackModel(std::string name, double rateHz, const HopperDynamics& truth, EngineFeedbackSpec spec,
                        uint64_t seed);

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

private:
    const HopperDynamics& truth_;
    EngineFeedbackSpec spec_;
    Rng rng_;
    Channel<EngineFeedbackMsg>* out_ = nullptr;
};

}  // namespace sim::vehicle
