#include "sim/vehicle/Sensors.hpp"

#include <cmath>
#include <utility>

namespace sim::vehicle {
namespace {

double quantize(double value, double quantum) {
    return quantum > 0.0 ? std::round(value / quantum) * quantum : value;
}

Eigen::Vector3d gaussianVector(Rng& rng, double sigma) {
    return {rng.normal(0.0, sigma), rng.normal(0.0, sigma), rng.normal(0.0, sigma)};
}

}  // namespace

ImuModel::ImuModel(std::string name, double rateHz, const HopperDynamics& truth, ImuSpec spec, uint64_t seed)
    : Model(std::move(name), rateHz), truth_(truth), spec_(spec), rng_(seed) {
    accelBias_ = gaussianVector(rng_, spec_.accelBiasSigma);
    gyroBias_ = gaussianVector(rng_, spec_.gyroBiasSigma);
}

void ImuModel::initialize(Bus& bus) { out_ = &bus.channel<ImuMsg>(channels::kImu); }

void ImuModel::update(const StepContext& ctx) {
    ImuMsg msg;
    msg.specificForce = truth_.specificForceBody();
    msg.angularRate = truth_.state().angularRate;

    if (!spec_.ideal) {
        const double rate = rateHz();
        const double samplePeriod = 1.0 / rate;
        const double sqrtRate = std::sqrt(rate);

        msg.specificForce += accelBias_ + gaussianVector(rng_, spec_.accelNoiseDensity * sqrtRate);
        msg.angularRate += gyroBias_ + gaussianVector(rng_, spec_.gyroNoiseDensity * sqrtRate);

        accelBias_ += gaussianVector(rng_, spec_.accelBiasRandomWalk * std::sqrt(samplePeriod));
        gyroBias_ += gaussianVector(rng_, spec_.gyroBiasRandomWalk * std::sqrt(samplePeriod));

        for (int i = 0; i < 3; ++i) {
            msg.specificForce[i] = quantize(msg.specificForce[i], spec_.accelQuantum);
            msg.angularRate[i] = quantize(msg.angularRate[i], spec_.gyroQuantum);
        }
    }

    out_->publish(msg, ctx.t);
}

RadarAltimeterModel::RadarAltimeterModel(std::string name, double rateHz, const HopperDynamics& truth,
                                         AltimeterSpec spec, uint64_t seed)
    : Model(std::move(name), rateHz), truth_(truth), spec_(spec), rng_(seed) {}

void RadarAltimeterModel::initialize(Bus& bus) { out_ = &bus.channel<AltimeterMsg>(channels::kAltimeter); }

void RadarAltimeterModel::update(const StepContext& ctx) {
    AltimeterMsg msg;
    const double trueRange = truth_.state().position.z();
    msg.valid = trueRange <= spec_.maxRange;
    msg.range = trueRange;

    if (!spec_.ideal) {
        msg.range += rng_.normal(0.0, spec_.noiseSigma);
        if (spec_.dropoutProbability > 0.0 && rng_.uniform() < spec_.dropoutProbability) {
            msg.valid = false;
        }
    }

    out_->publish(msg, ctx.t);
}

}  // namespace sim::vehicle
