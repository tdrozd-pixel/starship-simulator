#include "fsw/HopFlightComputer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

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

void cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Body +z (the vehicle axis) expressed in world: its up component is cos(tilt).
double upComponentOfBodyZ(const Quaternion& q) { return 1.0 - 2.0 * (q.x * q.x + q.y * q.y); }

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
    const double crossUp[3] = {upWorld[1], -upWorld[0], 0.0};
    const double dot = std::clamp(upWorld[2], -1.0, 1.0);
    const double crossNorm = std::sqrt(crossUp[0] * crossUp[0] + crossUp[1] * crossUp[1]);
    if (crossNorm <= 1e-12) {
        if (dot < 0.0) {
            return normalized(multiply({0.0, 1.0, 0.0, 0.0}, attitude));
        }
        return attitude;
    }
    const Quaternion correction = normalized({1.0 + dot, crossUp[0], crossUp[1], crossUp[2]});
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
      targetFrame_(config.target),
      estimator_(VerticalEstimator::Config{config.gravity, 4.0, 4.0}),
      horizontal_(HorizontalEstimator::Config{}),
      velocityPid_(config.velocityGains),
      propellantEstimate_(config.initialPropellant) {}

void HopFlightComputer::tick() {
    const double t = hal_.timeSec();
    const double dt = (lastTime_ < 0.0) ? 1.0 / config_.rateHz : t - lastTime_;
    lastTime_ = t;

    updateNavigation(t, dt);
    updateModeLogic(t, dt);

    // Propellant bookkeeping from our own commands (no flow meter yet).
    if (enginesOn_ && config_.vacuumIsp > 0.0) {
        const double flowPerEngine = throttle_ * config_.vacuumThrust / (config_.vacuumIsp * config_.gravity);
        propellantEstimate_ = std::max(0.0, propellantEstimate_ - kEngineCount * flowPerEngine * dt);
    }

    updateTvc();
    hal_.commandActuators(command_);

    Telemetry tm;
    tm.mode = static_cast<int>(mode_);
    tm.altitudeEstimate = estimator_.altitude();
    tm.verticalVelocityEstimate = estimator_.velocity();
    tm.altitudeSetpoint = altitudeSetpoint_;
    tm.velocitySetpoint = velocitySetpoint_;
    tm.throttleCommand = enginesOn_ ? throttle_ : 0.0;
    tm.massEstimate = massEstimate();
    tm.horizontalNavValid = horizontal_.initialized();
    tm.eastEstimate = horizontal_.east();
    tm.northEstimate = horizontal_.north();
    tm.eastVelocityEstimate = horizontal_.velocityEast();
    tm.northVelocityEstimate = horizontal_.velocityNorth();
    tm.eastSetpoint = horizontalTarget_[0];
    tm.northSetpoint = horizontalTarget_[1];
    tm.eastAccelCommand = accelCommand_[0];
    tm.northAccelCommand = accelCommand_[1];
    tm.gnssFixType = gnssFixType_;
    hal_.publishTelemetry(tm);
}

void HopFlightComputer::updateNavigation(double t, double dt) {
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
        // Gravity is vertical in the local frame, so horizontal acceleration is the
        // horizontal specific force at the IMU, the navigation reference point.
        horizontal_.predict(specificForceWorld[0], specificForceWorld[1], dt);
    }

    const Quaternion q = attitudeFrom(attitude_);

    // The altimeter looks along body -z from the base: slant range times cos(tilt).
    const AltimeterSample alt = hal_.readAltimeter();
    if (alt.valid && alt.stamp > lastAltimeterStamp_) {
        lastAltimeterStamp_ = alt.stamp;
        estimator_.correct(alt.range * std::max(0.0, upComponentOfBodyZ(q)), dt);
    }

    const GnssSample gnss = hal_.readGnss();
    if (gnss.valid && gnss.stamp > lastGnssStamp_) {
        lastGnssStamp_ = gnss.stamp;
        gnssFixType_ = static_cast<int>(gnss.fixType);
        ++gnssFixCount_;

        double antennaEnu[3];
        targetFrame_.toEnu({gnss.latitude, gnss.longitude, gnss.height}, antennaEnu);

        // Lever arm: antenna offset from the IMU, rotated into the local frame.
        const double lever[3] = {config_.gnssAntennaPosition[0] - config_.imuPosition[0],
                                 config_.gnssAntennaPosition[1] - config_.imuPosition[1],
                                 config_.gnssAntennaPosition[2] - config_.imuPosition[2]};
        double leverWorld[3];
        rotate(q, lever, leverWorld);
        double omegaCrossLever[3];
        cross(angularRateBody_, lever, omegaCrossLever);
        double leverVelocityWorld[3];
        rotate(q, omegaCrossLever, leverVelocityWorld);

        // Latency: the fix describes fixTime; move it to now with the velocity estimate.
        const double age = std::max(0.0, t - gnss.fixTime);
        const double east = antennaEnu[0] - leverWorld[0] + horizontal_.velocityEast() * age;
        const double north = antennaEnu[1] - leverWorld[1] + horizontal_.velocityNorth() * age;
        const double sigma = std::max(0.005, gnss.horizontalSigma);
        horizontal_.correctPosition(east, north, sigma);
        horizontal_.correctVelocity(gnss.velocityEnu[0] - leverVelocityWorld[0],
                                    gnss.velocityEnu[1] - leverVelocityWorld[1], config_.gnssVelocitySigma);

        // GNSS levelling: if the attitude estimate is rotated by a small world-frame
        // angle phi from the truth, the rotated IMU reports an extra horizontal
        // acceleration g * (phi_y, -phi_x). The filter's bias state measures exactly
        // that, so rotate the attitude back by it and hand the bias back to zero.
        const double bias[2] = {horizontal_.accelBiasEast(), horizontal_.accelBiasNorth()};
        const double phiX = std::clamp(-bias[1] / config_.gravity, -0.01, 0.01);  // [rad] per update
        const double phiY = std::clamp(bias[0] / config_.gravity, -0.01, 0.01);
        const Quaternion correction = normalized({1.0, -0.5 * phiX, -0.5 * phiY, 0.0});
        storeAttitude(normalized(multiply(correction, attitudeFrom(attitude_))), attitude_);
        horizontal_.removeBias(phiY * config_.gravity, -phiX * config_.gravity);
        // TODO(phase 9, item 43): use GNSS height when the altimeter is out of range.
    }

    // Sitting on the pad the velocity is known: zero.
    if (mode_ == HopMode::Prelaunch || mode_ == HopMode::Landed) {
        estimator_.reset(estimator_.altitude(), 0.0);
        horizontal_.zeroVelocityUpdate();
    }
}

bool HopFlightComputer::alignedOverTarget() const {
    const double errEast = horizontalTarget_[0] - horizontal_.east();
    const double errNorth = horizontalTarget_[1] - horizontal_.north();
    const double speed = std::hypot(horizontal_.velocityEast(), horizontal_.velocityNorth());
    return translating_ && std::hypot(errEast, errNorth) < config_.landingAlignTolerance &&
           speed < config_.landingAlignSpeed;
}

void HopFlightComputer::updateTvc() {
    for (EngineCommand& e : command_.engines) {
        e.on = enginesOn_;
        e.throttle = enginesOn_ ? throttle_ : 0.0;
        e.gimbalPitch = 0.0;
        e.gimbalYaw = 0.0;
    }
    if (!enginesOn_ || config_.vacuumThrust <= 0.0 || config_.engineLeverArm <= 0.0 ||
        config_.gimbalLimit <= 0.0 || config_.transverseInertiaDry <= 0.0) {
        return;
    }

    // Horizontal guidance. Until liftoff the vehicle stays upright on the pad.
    double desiredAccel[2] = {0.0, 0.0};
    if (mode_ != HopMode::Ignition && horizontal_.initialized()) {
        const double err[2] = {horizontalTarget_[0] - horizontal_.east(),
                               horizontalTarget_[1] - horizontal_.north()};
        const double distance = std::hypot(err[0], err[1]);
        // Braking profile: approach no faster than the vehicle can stop using half
        // the allowed horizontal acceleration (v = sqrt(2 * a/2 * d)).
        const double speedLimit = std::min(config_.maxHorizontalSpeed,
                                           std::sqrt(config_.maxHorizontalAccel * distance));
        double velocitySetpoint[2] = {config_.horizontalPositionKp * err[0], config_.horizontalPositionKp * err[1]};
        const double setpointSpeed = std::hypot(velocitySetpoint[0], velocitySetpoint[1]);
        if (setpointSpeed > speedLimit && setpointSpeed > 1e-9) {
            velocitySetpoint[0] *= speedLimit / setpointSpeed;
            velocitySetpoint[1] *= speedLimit / setpointSpeed;
        }
        const double velocityError[2] = {velocitySetpoint[0] - horizontal_.velocityEast(),
                                         velocitySetpoint[1] - horizontal_.velocityNorth()};
        const double dt = 1.0 / config_.rateHz;
        for (int i = 0; i < 2; ++i) {
            horizontalTrim_[i] = std::clamp(horizontalTrim_[i] + config_.horizontalVelocityKi * velocityError[i] * dt,
                                            -config_.maxHorizontalTrim, config_.maxHorizontalTrim);
            desiredAccel[i] = config_.horizontalVelocityKp * velocityError[i] + horizontalTrim_[i];
        }
        const double accel = std::hypot(desiredAccel[0], desiredAccel[1]);
        if (accel > config_.maxHorizontalAccel) {
            desiredAccel[0] *= config_.maxHorizontalAccel / accel;
            desiredAccel[1] *= config_.maxHorizontalAccel / accel;
        }
    }

    accelCommand_[0] = desiredAccel[0];
    accelCommand_[1] = desiredAccel[1];

    // Tilt that produces that acceleration while thrust holds up the weight.
    const double desiredPitch = -std::atan2(desiredAccel[1], config_.gravity);
    const double desiredYaw = std::atan2(desiredAccel[0], config_.gravity);
    const Quaternion qPitch{std::cos(desiredPitch * 0.5), std::sin(desiredPitch * 0.5), 0.0, 0.0};
    const Quaternion qYaw{std::cos(desiredYaw * 0.5), 0.0, std::sin(desiredYaw * 0.5), 0.0};
    const Quaternion targetAttitude = normalized(multiply(qYaw, qPitch));
    const Quaternion attitudeError = normalized(multiply(
        {targetAttitude.w, -targetAttitude.x, -targetAttitude.y, -targetAttitude.z},
        attitudeFrom(attitude_)));
    double error[3];
    rotationVector(attitudeError, error);

    // Torque per unit gimbal from the mass properties we were given.
    const double propellantFraction =
        config_.initialPropellant > 0.0 ? std::max(0.0, propellantEstimate_) / config_.initialPropellant : 0.0;
    const double transverseInertia =
        config_.transverseInertiaDry + propellantFraction * config_.transverseInertiaPropellant;
    const double axialInertia = config_.axialInertiaDry + propellantFraction * config_.axialInertiaPropellant;
    const double thrustPerEngine = std::max(1.0, std::max(config_.minThrottle, throttle_) * config_.vacuumThrust -
                                                     config_.ambientPressure * config_.nozzleExitArea);
    const double totalThrust = kEngineCount * thrustPerEngine;

    // Common gimbal for pitch and yaw (small angle: torque = lever * thrust * angle).
    const double gimbalGain = transverseInertia / (config_.engineLeverArm * totalThrust);
    const double commonPitch = gimbalGain * (config_.attitudeKp * error[0] + config_.attitudeKd * angularRateBody_[0]);
    const double commonYaw = gimbalGain * (config_.attitudeKp * error[1] + config_.attitudeKd * angularRateBody_[1]);

    // Roll: tilt every engine tangentially by the same angle; torque = N * radius * thrust * angle.
    double roll = 0.0;
    if (config_.engineClusterRadius > 0.0) {
        const double rollGain = axialInertia / (totalThrust * config_.engineClusterRadius);
        roll = std::clamp(-rollGain * (config_.rollKp * error[2] + config_.rollKd * angularRateBody_[2]),
                          -config_.maxRollGimbal, config_.maxRollGimbal);
    }

    // Lateral thrust direction per engine in the body x-y plane, from the thrust
    // direction (cos p sin y, -sin p, cos p cos y).
    const double lateralX = std::sin(std::clamp(commonYaw, -config_.gimbalLimit, config_.gimbalLimit));
    const double lateralY = -std::sin(std::clamp(commonPitch, -config_.gimbalLimit, config_.gimbalLimit));
    for (int i = 0; i < kEngineCount; ++i) {
        const double azimuth = config_.engineAzimuth0 + 2.0 * std::numbers::pi * i / kEngineCount;
        const double lx = lateralX - roll * std::sin(azimuth);
        const double ly = lateralY + roll * std::cos(azimuth);
        const double pitch = -std::asin(std::clamp(ly, -1.0, 1.0));
        const double yaw = std::asin(std::clamp(lx / std::cos(pitch), -1.0, 1.0));
        command_.engines[i].gimbalPitch = std::clamp(pitch, -config_.gimbalLimit, config_.gimbalLimit);
        command_.engines[i].gimbalYaw = std::clamp(yaw, -config_.gimbalLimit, config_.gimbalLimit);
    }
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
            enginesOn_ = false;
            throttle_ = 0.0;
            // Go for ignition only with a navigation solution.
            if (t >= config_.ignitionTime && gnssFixCount_ >= config_.gnssFixesRequired &&
                horizontal_.initialized()) {
                horizontalTarget_[0] = horizontal_.east();
                horizontalTarget_[1] = horizontal_.north();
                enter(HopMode::Ignition);
            }
            break;

        case HopMode::Ignition:
            enginesOn_ = true;
            throttle_ = throttleForThrust(config_.ignitionThrustToWeight * massEstimate() * config_.gravity);
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
            throttle_ = computeThrottle(velocitySetpoint_, dt);
            // Clear the liftoff point vertically, then head for the target.
            if (!translating_ && alt > config_.translateAltitude) {
                translating_ = true;
                horizontalTarget_[0] = 0.0;
                horizontalTarget_[1] = 0.0;
            }
            if (altitudeSetpoint_ >= config_.targetAltitude && std::abs(alt - config_.targetAltitude) < 1.0) {
                enter(HopMode::Hover);
            }
            break;
        }

        case HopMode::Hover:
            translating_ = true;
            horizontalTarget_[0] = 0.0;
            horizontalTarget_[1] = 0.0;
            altitudeSetpoint_ = config_.targetAltitude;
            velocitySetpoint_ = std::clamp(config_.altitudeKp * (altitudeSetpoint_ - alt),
                                           -config_.maxSinkRate, config_.maxClimbRate);
            velocitySetpoint_ = std::min(velocitySetpoint_, maxClimbRateToStopWithin(config_.targetAltitude - alt));
            throttle_ = computeThrottle(velocitySetpoint_, dt);
            // TODO(phase 9, item 47): divert or abort if alignment never comes (propellant margin).
            if (t - modeStartTime_ >= config_.hoverDuration && alignedOverTarget()) {
                enter(HopMode::Descent);
            }
            break;

        case HopMode::Descent: {
            altitudeSetpoint_ = std::max(config_.finalDescentAltitude, altitudeSetpoint_ - config_.descentRate * dt);
            const double feedforward = (altitudeSetpoint_ > config_.finalDescentAltitude) ? -config_.descentRate : 0.0;
            velocitySetpoint_ = std::clamp(config_.altitudeKp * (altitudeSetpoint_ - alt) + feedforward,
                                           -config_.maxSinkRate, config_.maxClimbRate);
            throttle_ = computeThrottle(velocitySetpoint_, dt);
            if (alt < config_.finalDescentAltitude + 0.5) {
                enter(HopMode::FinalDescent);
            }
            break;
        }

        case HopMode::FinalDescent:
            altitudeSetpoint_ = 0.0;
            velocitySetpoint_ = -config_.finalDescentRate;
            throttle_ = computeThrottle(velocitySetpoint_, dt);
            if (alt < config_.touchdownAltitude && std::abs(vel) < config_.touchdownSpeed) {
                touchdownTimer_ += dt;
            } else {
                touchdownTimer_ = 0.0;
            }
            if (touchdownTimer_ >= config_.touchdownConfirmTime) {
                enginesOn_ = false;
                throttle_ = 0.0;
                enter(HopMode::Landed);
            }
            break;

        case HopMode::Landed:
            enginesOn_ = false;
            throttle_ = 0.0;
            velocitySetpoint_ = 0.0;
            break;
    }
}

double HopFlightComputer::massEstimate() const { return config_.dryMass + std::max(0.0, propellantEstimate_); }

double HopFlightComputer::throttleForThrust(double totalThrust) const {
    // Per engine: F = throttle * F_vac - p_amb * A_exit.
    const double perEngine = totalThrust / kEngineCount;
    const double throttle = (perEngine + config_.ambientPressure * config_.nozzleExitArea) / config_.vacuumThrust;
    return std::clamp(throttle, config_.minThrottle, 1.0);
}

double HopFlightComputer::minimumTotalThrust() const {
    return kEngineCount *
           std::max(0.0, config_.minThrottle * config_.vacuumThrust - config_.ambientPressure * config_.nozzleExitArea);
}

double HopFlightComputer::computeThrottle(double velocitySetpoint, double dt) {
    const double accelCommand = velocityPid_.update(velocitySetpoint, estimator_.velocity(), dt);
    // Only the vertical component of a tilted thrust vector holds up the weight.
    const double cosTilt = std::max(0.7, upComponentOfBodyZ(attitudeFrom(attitude_)));
    return throttleForThrust(massEstimate() * (config_.gravity + accelCommand) / cosTilt);
}

double HopFlightComputer::maxClimbRateToStopWithin(double distance) const {
    // At minimum throttle the net deceleration is g - Tmin/m. Climbing faster than
    // sqrt(2 a d) means the vehicle cannot stop before the target: it overshoots.
    const double brakingDecel = config_.gravity - minimumTotalThrust() / massEstimate();
    const double usable = std::max(0.1, brakingDecel) * config_.brakingMargin;
    return std::sqrt(2.0 * usable * std::max(0.0, distance));
}

}  // namespace fsw
