#include "fsw/HopFlightComputer.hpp"

#include <algorithm>
#include <cmath>

namespace fsw {
namespace {

struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

Quaternion normalized(Quaternion q) {
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 1e-12) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

void rotate(const Quaternion& q, const double input[3], double output[3]) {
    const double tx = 2.0 * (q.y * input[2] - q.z * input[1]);
    const double ty = 2.0 * (q.z * input[0] - q.x * input[2]);
    const double tz = 2.0 * (q.x * input[1] - q.y * input[0]);
    output[0] = input[0] + q.w * tx + q.y * tz - q.z * ty;
    output[1] = input[1] + q.w * ty + q.z * tx - q.x * tz;
    output[2] = input[2] + q.w * tz + q.x * ty - q.y * tx;
}

Quaternion integrateBodyRate(Quaternion attitude, const double rate[3], double dt) {
    const double magnitude = std::sqrt(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]);
    if (magnitude <= 1e-12 || dt <= 0.0) {
        return attitude;
    }
    const double halfAngle = 0.5 * magnitude * dt;
    const double scale = std::sin(halfAngle) / magnitude;
    const Quaternion delta{std::cos(halfAngle), rate[0] * scale, rate[1] * scale, rate[2] * scale};
    return normalized(multiply(attitude, delta));
}

Quaternion attitudeFrom(const double q[4]) { return {q[0], q[1], q[2], q[3]}; }

void storeAttitude(const Quaternion& q, double output[4]) {
    output[0] = q.w;
    output[1] = q.x;
    output[2] = q.y;
    output[3] = q.z;
}

Quaternion gravityAligned(Quaternion attitude, const double specificForce[3]) {
    const double norm = std::sqrt(specificForce[0] * specificForce[0] +
                                  specificForce[1] * specificForce[1] +
                                  specificForce[2] * specificForce[2]);
    if (norm <= 1e-9) {
        return attitude;
    }

    const double upBody[3] = {specificForce[0] / norm, specificForce[1] / norm, specificForce[2] / norm};
    double upWorld[3];
    rotate(attitude, upBody, upWorld);
    const double cross[3] = {upWorld[1], -upWorld[0], 0.0};
    const double dot = std::clamp(upWorld[2], -1.0, 1.0);
    const double crossNorm = std::sqrt(cross[0] * cross[0] + cross[1] * cross[1]);
    if (crossNorm <= 1e-12) {
        if (dot < 0.0) {
            return normalized(multiply({0.0, 1.0, 0.0, 0.0}, attitude));
        }
        return attitude;
    }
    const Quaternion correction = normalized({1.0 + dot, cross[0], cross[1], cross[2]});
    return normalized(multiply(correction, attitude));
}

void rotationVector(const Quaternion& qIn, double output[3]) {
    Quaternion q = normalized(qIn);
    if (q.w < 0.0) {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }
    const double vectorNorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    if (vectorNorm <= 1e-12) {
        output[0] = output[1] = output[2] = 0.0;
        return;
    }
    const double angle = 2.0 * std::atan2(vectorNorm, std::clamp(q.w, -1.0, 1.0));
    const double scale = angle / vectorNorm;
    output[0] = q.x * scale;
    output[1] = q.y * scale;
    output[2] = q.z * scale;
}

}  // namespace

const char* toString(HopMode mode) {
    switch (mode) {
        case HopMode::Prelaunch: return "Prelaunch";
        case HopMode::Ignition: return "Ignition";
        case HopMode::Ascent: return "Ascent";
        case HopMode::Hover: return "Hover";
        case HopMode::Descent: return "Descent";
        case HopMode::FinalDescent: return "FinalDescent";
        case HopMode::Landed: return "Landed";
    }
    return "Unknown";
}

HopFlightComputer::HopFlightComputer(Hal& hal, HopConfig config)
    : hal_(hal),
      config_(config),
      estimator_(VerticalEstimator::Config{config.gravity, 4.0, 4.0}),
      velocityPid_(config.velocityGains),
      propellantEstimate_(config.initialPropellant) {}

void HopFlightComputer::tick() {
    const double t = hal_.timeSec();
    const double dt = (lastTime_ < 0.0) ? 1.0 / config_.rateHz : t - lastTime_;
    lastTime_ = t;

    updateNavigation(dt);
    updateModeLogic(t, dt);

    // Propellant bookkeeping from our own commands (no flow meter yet).
    if (command_.engineOn) {
        const double flow = command_.throttle * config_.maxThrust / (config_.ispSeaLevel * config_.gravity);
        propellantEstimate_ = std::max(0.0, propellantEstimate_ - flow * dt);
    }

    updateTvc();
    hal_.commandActuators(command_);

    Telemetry tm;
    tm.mode = static_cast<int>(mode_);
    tm.altitudeEstimate = estimator_.altitude();
    tm.verticalVelocityEstimate = estimator_.velocity();
    tm.altitudeSetpoint = altitudeSetpoint_;
    tm.velocitySetpoint = velocitySetpoint_;
    tm.throttleCommand = command_.throttle;
    tm.massEstimate = config_.dryMass + propellantEstimate_;
    hal_.publishTelemetry(tm);
}

void HopFlightComputer::updateNavigation(double dt) {
    const ImuSample imu = hal_.readImu();
    if (imu.valid && imu.stamp > lastImuStamp_) {
        const double imuDt = lastImuStamp_ < 0.0 ? dt : imu.stamp - lastImuStamp_;
        lastImuStamp_ = imu.stamp;
        for (int i = 0; i < 3; ++i) {
            angularRateBody_[i] = imu.angularRate[i];
        }

        Quaternion attitude = attitudeFrom(attitude_);
        const double specificForceMagnitude =
            std::sqrt(imu.specificForce[0] * imu.specificForce[0] +
                      imu.specificForce[1] * imu.specificForce[1] +
                      imu.specificForce[2] * imu.specificForce[2]);
        if (mode_ == HopMode::Prelaunch &&
            std::abs(specificForceMagnitude - config_.gravity) < 2.0) {
            attitude = gravityAligned(attitude, imu.specificForce);
        }
        attitude = integrateBodyRate(attitude, imu.angularRate, imuDt);
        storeAttitude(attitude, attitude_);

        const double specificForceBody[3] = {
            imu.specificForce[0], imu.specificForce[1], imu.specificForce[2]};
        double specificForceWorld[3];
        rotate(attitude, specificForceBody, specificForceWorld);
        estimator_.predict(specificForceWorld[2], dt);
        velocityWorld_[0] += specificForceWorld[0] * dt;
        velocityWorld_[1] += specificForceWorld[1] * dt;
        velocityWorld_[2] += (specificForceWorld[2] - config_.gravity) * dt;
    }

    const AltimeterSample alt = hal_.readAltimeter();
    if (alt.valid && alt.stamp > lastAltimeterStamp_) {
        lastAltimeterStamp_ = alt.stamp;
        estimator_.correct(alt.range, dt);
    }

    // Sitting on the pad the estimate is known: zero velocity.
    if (mode_ == HopMode::Prelaunch || mode_ == HopMode::Landed) {
        estimator_.reset(estimator_.altitude(), 0.0);
        velocityWorld_[0] = velocityWorld_[1] = velocityWorld_[2] = 0.0;
    }
}

void HopFlightComputer::updateTvc() {
    if (!command_.engineOn || config_.maxThrust <= 0.0 || config_.engineLeverArm <= 0.0 ||
        config_.gimbalLimit <= 0.0 || config_.vehicleRadius <= 0.0 || config_.vehicleLength <= 0.0) {
        command_.gimbalPitch = 0.0;
        command_.gimbalYaw = 0.0;
        return;
    }

    // IMU-only horizontal velocity is dead reckoning; a single engine has no roll authority.
    const double desiredAccelEast = std::clamp(-config_.horizontalVelocityKp * velocityWorld_[0],
                                               -config_.maxHorizontalAccel, config_.maxHorizontalAccel);
    const double desiredAccelNorth = std::clamp(-config_.horizontalVelocityKp * velocityWorld_[1],
                                                -config_.maxHorizontalAccel, config_.maxHorizontalAccel);
    const double desiredPitch = -std::atan2(desiredAccelNorth, config_.gravity);
    const double desiredYaw = std::atan2(desiredAccelEast, config_.gravity);
    const Quaternion qPitch{std::cos(desiredPitch * 0.5), std::sin(desiredPitch * 0.5), 0.0, 0.0};
    const Quaternion qYaw{std::cos(desiredYaw * 0.5), 0.0, std::sin(desiredYaw * 0.5), 0.0};
    const Quaternion targetAttitude = normalized(multiply(qYaw, qPitch));
    const Quaternion attitudeError = normalized(multiply(
        {targetAttitude.w, -targetAttitude.x, -targetAttitude.y, -targetAttitude.z},
        attitudeFrom(attitude_)));
    double error[3];
    rotationVector(attitudeError, error);

    const double mass = config_.dryMass + std::max(0.0, propellantEstimate_);
    const double transverseInertia = mass * (3.0 * config_.vehicleRadius * config_.vehicleRadius +
                                              config_.vehicleLength * config_.vehicleLength) / 12.0;
    const double thrust = std::max(config_.minThrottle, command_.throttle) * config_.maxThrust;
    const double gimbalGain = transverseInertia / (config_.engineLeverArm * thrust);

    command_.gimbalPitch = std::clamp(
        gimbalGain * (config_.attitudeKp * error[0] + config_.attitudeKd * angularRateBody_[0]),
        -config_.gimbalLimit, config_.gimbalLimit);
    command_.gimbalYaw = std::clamp(
        gimbalGain * (config_.attitudeKp * error[1] + config_.attitudeKd * angularRateBody_[1]),
        -config_.gimbalLimit, config_.gimbalLimit);
}

void HopFlightComputer::updateModeLogic(double t, double dt) {
    const double alt = estimator_.altitude();
    const double vel = estimator_.velocity();

    auto enter = [&](HopMode next) {
        mode_ = next;
        modeStartTime_ = t;
    };

    switch (mode_) {
        case HopMode::Prelaunch:
            command_ = ActuatorCommand{};
            if (t >= config_.ignitionTime) {
                enter(HopMode::Ignition);
            }
            break;

        case HopMode::Ignition:
            command_.engineOn = true;
            command_.throttle = config_.ignitionThrottle;
            if (alt > config_.liftoffAltitude) {
                altitudeSetpoint_ = alt;
                velocityPid_.reset();
                enter(HopMode::Ascent);
            }
            break;

        case HopMode::Ascent: {
            altitudeSetpoint_ = std::min(config_.targetAltitude, altitudeSetpoint_ + config_.ascentRate * dt);
            const double feedforward = (altitudeSetpoint_ < config_.targetAltitude) ? config_.ascentRate : 0.0;
            velocitySetpoint_ = std::clamp(config_.altitudeKp * (altitudeSetpoint_ - alt) + feedforward,
                                           -config_.maxSinkRate, config_.maxClimbRate);
            velocitySetpoint_ = std::min(velocitySetpoint_, maxClimbRateToStopWithin(config_.targetAltitude - alt));
            command_.throttle = computeThrottle(velocitySetpoint_, dt);
            if (altitudeSetpoint_ >= config_.targetAltitude && std::abs(alt - config_.targetAltitude) < 1.0) {
                enter(HopMode::Hover);
            }
            break;
        }

        case HopMode::Hover:
            altitudeSetpoint_ = config_.targetAltitude;
            velocitySetpoint_ = std::clamp(config_.altitudeKp * (altitudeSetpoint_ - alt),
                                           -config_.maxSinkRate, config_.maxClimbRate);
            velocitySetpoint_ = std::min(velocitySetpoint_, maxClimbRateToStopWithin(config_.targetAltitude - alt));
            command_.throttle = computeThrottle(velocitySetpoint_, dt);
            if (t - modeStartTime_ >= config_.hoverDuration) {
                enter(HopMode::Descent);
            }
            break;

        case HopMode::Descent: {
            altitudeSetpoint_ = std::max(config_.finalDescentAltitude, altitudeSetpoint_ - config_.descentRate * dt);
            const double feedforward = (altitudeSetpoint_ > config_.finalDescentAltitude) ? -config_.descentRate : 0.0;
            velocitySetpoint_ = std::clamp(config_.altitudeKp * (altitudeSetpoint_ - alt) + feedforward,
                                           -config_.maxSinkRate, config_.maxClimbRate);
            command_.throttle = computeThrottle(velocitySetpoint_, dt);
            if (alt < config_.finalDescentAltitude + 0.5) {
                enter(HopMode::FinalDescent);
            }
            break;
        }

        case HopMode::FinalDescent:
            altitudeSetpoint_ = 0.0;
            velocitySetpoint_ = -config_.finalDescentRate;
            command_.throttle = computeThrottle(velocitySetpoint_, dt);
            if (alt < config_.touchdownAltitude && std::abs(vel) < config_.touchdownSpeed) {
                touchdownTimer_ += dt;
            } else {
                touchdownTimer_ = 0.0;
            }
            if (touchdownTimer_ >= config_.touchdownConfirmTime) {
                command_ = ActuatorCommand{};  // engine off
                enter(HopMode::Landed);
            }
            break;

        case HopMode::Landed:
            command_ = ActuatorCommand{};
            velocitySetpoint_ = 0.0;
            break;
    }
}

double HopFlightComputer::computeThrottle(double velocitySetpoint, double dt) {
    const double accelCommand = velocityPid_.update(velocitySetpoint, estimator_.velocity(), dt);
    const double mass = config_.dryMass + propellantEstimate_;
    const double thrust = mass * (config_.gravity + accelCommand);
    return std::clamp(thrust / config_.maxThrust, config_.minThrottle, 1.0);
}

double HopFlightComputer::maxClimbRateToStopWithin(double distance) const {
    // At minimum throttle the net deceleration is g - Tmin/m. Climbing faster than
    // sqrt(2 a d) means the vehicle cannot stop before the target: it overshoots.
    const double mass = config_.dryMass + propellantEstimate_;
    const double brakingDecel = config_.gravity - config_.minThrottle * config_.maxThrust / mass;
    const double usable = std::max(0.1, brakingDecel) * config_.brakingMargin;
    return std::sqrt(2.0 * usable * std::max(0.0, distance));
}

}  // namespace fsw
