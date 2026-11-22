#include "sim/vehicle/Sensors.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
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
    // Slant range from the base along body -z to flat ground at z = 0.
    const double cosTilt = (truth_.state().attitude * Eigen::Vector3d::UnitZ()).z();
    const double baseHeight = std::max(0.0, truth_.basePosition().z());
    const double trueRange = cosTilt > 0.1 ? baseHeight / cosTilt : spec_.maxRange + 1.0;
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

GnssModel::GnssModel(std::string name, double rateHz, const HopperDynamics& truth,
                     const env::LocalTangentFrame& site, const Eigen::Vector3d& antennaFromBase, GnssSpec spec,
                     uint64_t seed)
    : Model(std::move(name), rateHz), truth_(truth), site_(site), antenna_(antennaFromBase), spec_(spec), rng_(seed) {
    const double ratio = rateHz / spec_.outputRateHz;
    stepsPerOutput_ = static_cast<int>(std::lround(ratio));
    if (stepsPerOutput_ < 1 || std::abs(ratio - stepsPerOutput_) > 1e-9) {
        throw std::invalid_argument("GnssModel: output rate must divide the model rate");
    }
    if (!spec_.ideal && spec_.latency >= 1.0 / spec_.outputRateHz) {
        throw std::invalid_argument("GnssModel: latency must be shorter than the output period");
    }
    // Errors start somewhere in their steady-state distribution.
    bias_ = Eigen::Vector3d(rng_.normal(0.0, spec_.horizontalBiasSigma), rng_.normal(0.0, spec_.horizontalBiasSigma),
                            rng_.normal(0.0, spec_.verticalBiasSigma));
}

void GnssModel::initialize(Bus& bus) { out_ = &bus.channel<GnssMsg>(channels::kGnss); }

double GnssModel::gaussMarkovStep(double value, double sigma, double period) {
    const double phi = std::exp(-period / spec_.biasTimeConstant);
    return phi * value + sigma * std::sqrt(1.0 - phi * phi) * rng_.normal();
}

void GnssModel::update(const StepContext& ctx) {
    // Output a solution once its latency has passed.
    if (pending_ && ctx.t >= pendingFix_.fixTime + spec_.latency - 1e-9) {
        out_->publish(pendingFix_, ctx.t);
        pending_ = false;
    }

    const bool fixDue = (stepCounter_ % stepsPerOutput_) == 0;
    ++stepCounter_;
    if (!fixDue) {
        return;
    }

    Eigen::Vector3d position = truth_.pointWorld(antenna_);
    Eigen::Vector3d velocity = truth_.pointVelocityWorld(antenna_);
    GnssMsg fix;
    fix.valid = true;
    fix.fixType = spec_.fixType;
    fix.fixTime = ctx.t;
    fix.horizontalSigma = spec_.reportedHorizontalSigma;
    fix.verticalSigma = spec_.reportedVerticalSigma;

    if (!spec_.ideal) {
        const double period = 1.0 / spec_.outputRateHz;
        bias_.x() = gaussMarkovStep(bias_.x(), spec_.horizontalBiasSigma, period);
        bias_.y() = gaussMarkovStep(bias_.y(), spec_.horizontalBiasSigma, period);
        bias_.z() = gaussMarkovStep(bias_.z(), spec_.verticalBiasSigma, period);
        position += bias_ + Eigen::Vector3d(rng_.normal(0.0, spec_.horizontalNoiseSigma),
                                            rng_.normal(0.0, spec_.horizontalNoiseSigma),
                                            rng_.normal(0.0, spec_.verticalNoiseSigma));
        velocity += gaussianVector(rng_, spec_.velocityNoiseSigma);
        if (spec_.dropoutProbability > 0.0 && rng_.uniform() < spec_.dropoutProbability) {
            fix.valid = false;
            fix.fixType = 0;
        }
    }

    const env::Geodetic g = site_.enuToGeodetic(position);
    fix.velocityEnu = velocity;
    if (spec_.ideal) {
        fix.latitude = g.latitude;
        fix.longitude = g.longitude;
        fix.height = g.height;
        out_->publish(fix, ctx.t);
        return;
    }
    fix.latitude = quantize(g.latitude, spec_.angleQuantum);
    fix.longitude = quantize(g.longitude, spec_.angleQuantum);
    fix.height = quantize(g.height, 1.0e-3);  // [m] output resolution
    pendingFix_ = fix;
    pending_ = true;
}

}  // namespace sim::vehicle
