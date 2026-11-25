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
                     uint64_t seed, uint64_t constellationSeed)
    : Model(std::move(name), rateHz),
      truth_(truth),
      site_(site),
      antenna_(antennaFromBase),
      spec_(spec),
      rng_(seed),
      constellationRng_(constellationSeed) {
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

void GnssModel::stepConstellation() {
    // Placeholder: about every 20 s (0.5% of the 10 Hz solutions) the count moves by one.
    if (constellationRng_.uniform() < 0.005) {
        satellitesVisible_ += constellationRng_.uniform() < 0.5 ? -1 : 1;
        satellitesVisible_ = std::clamp(satellitesVisible_, 16, 32);
    }
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
    stepConstellation();
    fix.satellitesVisible = satellitesVisible_;
    fix.satellitesUsed = satellitesVisible_ - static_cast<int>(constellationRng_.uniform() * 3.0);  // 0..2 unused

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
            fix.satellitesUsed = 0;
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

GimbalFeedbackModel::GimbalFeedbackModel(std::string name, double rateHz, const HopperDynamics& truth,
                                         GimbalFeedbackSpec spec, uint64_t seed)
    : Model(std::move(name), rateHz), truth_(truth), spec_(spec), rng_(seed) {
    bias_.setZero();
    if (!spec_.ideal) {
        for (int i = 0; i < kSeaLevelEngineCount; ++i) {
            bias_(0, i) = rng_.normal(0.0, spec_.biasSigma);
            bias_(1, i) = rng_.normal(0.0, spec_.biasSigma);
        }
    }
}

void GimbalFeedbackModel::initialize(Bus& bus) { out_ = &bus.channel<GimbalFeedbackMsg>(channels::kGimbalFeedback); }

void GimbalFeedbackModel::update(const StepContext& ctx) {
    GimbalFeedbackMsg now;
    const EngineGimbals& actual = truth_.state().gimbal;  // actual positions, never commands
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        now.valid[i] = true;
        double pitch = actual(0, i);
        double yaw = actual(1, i);
        if (!spec_.ideal) {
            pitch = quantize(pitch + bias_(0, i) + rng_.normal(0.0, spec_.noiseSigma), spec_.quantum);
            yaw = quantize(yaw + bias_(1, i) + rng_.normal(0.0, spec_.noiseSigma), spec_.quantum);
        }
        now.pitch[i] = pitch;
        now.yaw[i] = yaw;
    }
    if (spec_.ideal) {
        out_->publish(now, ctx.t);
        return;
    }
    // One sample of latency: publish what was measured on the previous update.
    out_->publish(havePrevious_ ? previous_ : now, ctx.t);
    previous_ = now;
    havePrevious_ = true;
}

EngineFeedbackModel::EngineFeedbackModel(std::string name, double rateHz, const HopperDynamics& truth,
                                         EngineFeedbackSpec spec, uint64_t seed)
    : Model(std::move(name), rateHz), truth_(truth), spec_(spec), rng_(seed) {}

void EngineFeedbackModel::initialize(Bus& bus) { out_ = &bus.channel<EngineFeedbackMsg>(channels::kEngineFeedback); }

void EngineFeedbackModel::update(const StepContext& ctx) {
    EngineFeedbackMsg now;
    for (int i = 0; i < kEngineCount; ++i) {
        now.valid[i] = true;
        double throttle = truth_.state().throttle[i];  // actual, never the command
        if (!spec_.ideal) {
            throttle = std::max(0.0, quantize(throttle + rng_.normal(0.0, spec_.noiseSigma), spec_.quantum));
        }
        now.throttle[i] = throttle;
        now.state[i] = truth_.engineState(i);
    }
    out_->publish(now, ctx.t);
}

}  // namespace sim::vehicle
