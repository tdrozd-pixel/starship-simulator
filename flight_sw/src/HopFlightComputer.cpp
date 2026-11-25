#include "fsw/HopFlightComputer.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "fsw/EngineAllocator.hpp"

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
        case HopMode::Descent: return "Descent";
        case HopMode::FinalDescent: return "FinalDescent";
        case HopMode::Landed: return "Landed";
        case HopMode::CatchApproach: return "CatchApproach";
        case HopMode::CatchHold: return "CatchHold";
        case HopMode::CatchFinal: return "CatchFinal";
        case HopMode::Caught: return "Caught";
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
      propellantEstimate_(config.initialPropellant) {
    landingMode_ = config_.landingMode;
    descentFloor_ = config_.finalDescentAltitude;
    horizontalSpeedCap_ = config_.maxHorizontalSpeed;
    stagingPoint_[0] = 0.0;  // Pad: the target is the origin of the target frame
    stagingPoint_[1] = 0.0;
    if (landingMode_ == LandingMode::Catch) {
        // Translation target: stagingDistance before the seat along the approach direction
        // (0: straight over the seat, so the descent into the arms is vertical).
        const CatchConfig& c = config_.catchConfig;
        const double length = std::hypot(c.approachDir[0], c.approachDir[1]);
        const double scale = length > 1e-9 ? c.stagingDistance / length : 0.0;
        stagingPoint_[0] = c.seat[0] - c.approachDir[0] * scale;
        stagingPoint_[1] = c.seat[1] - c.approachDir[1] * scale;
        horizontalSpeedCap_ = c.horizontalSpeedCap;
    }
}

void HopFlightComputer::tick() {
    const double t = hal_.timeSec();
    const double dt = (lastTime_ < 0.0) ? 1.0 / config_.rateHz : t - lastTime_;
    lastTime_ = t;

    gimbalFeedback_ = hal_.readGimbalFeedback();
    engineFeedback_ = hal_.readEngineFeedback();
    gnssNew_ = false;
    updateNavigation(t, dt);
    updatePinOffset(dt);
    updateModeLogic(t, dt);

    // Propellant bookkeeping from our own commands (no flow meter yet).
    if (enginesOn_ && config_.vacuumIsp > 0.0) {
        const double flowPerEngine = throttle_ * config_.vacuumThrust / (config_.vacuumIsp * config_.gravity);
        propellantEstimate_ = std::max(0.0, propellantEstimate_ - kSeaLevelEngineCount * flowPerEngine * dt);
    }
    if (config_.rvacVacuumIsp > 0.0) {
        // RVacs: from the previous cycle's command (they are off for the hop).
        for (int i = kFirstRvac; i < kEngineCount; ++i) {
            if (command_.engines[i].on) {
                const double flow = command_.engines[i].throttle * config_.rvacVacuumThrust /
                                    (config_.rvacVacuumIsp * config_.gravity);
                propellantEstimate_ = std::max(0.0, propellantEstimate_ - flow * dt);
            }
        }
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

    downlinkTelemetry(t);
    ++cycle_;
}

void HopFlightComputer::downlinkTelemetry(double t) {
    GncTelemetryInput& in = downlinkInput_;
    in.mode = static_cast<int>(mode_);
    in.enginesOn = enginesOn_;
    in.horizontalNavValid = horizontal_.initialized();
    in.translating = translating_;
    in.alignedOverTarget = alignedOverTarget();
    in.gnssReady = gnssFixCount_ >= config_.gnssFixesRequired;
    in.gnssFixCount = gnssFixCount_;
    in.timeInMode = t - modeStartTime_;
    in.propellantEstimate = propellantEstimate_;
    in.massEstimate = massEstimate();

    in.targetLatitude = config_.target.latitude;
    in.targetLongitude = config_.target.longitude;
    in.targetHeight = config_.target.height;
    in.targetAltitude = config_.targetAltitude;
    in.landingAlignTolerance = config_.landingAlignTolerance;
    in.translateAltitude = config_.translateAltitude;

    for (int i = 0; i < 4; ++i) {
        in.attitude[i] = attitude_[i];
    }
    in.imu = lastImu_;
    in.altitude = estimator_.altitude();
    in.verticalVelocity = estimator_.velocity();
    in.altimeter = lastAltimeter_;
    in.east = horizontal_.east();
    in.north = horizontal_.north();
    in.velocityEast = horizontal_.velocityEast();
    in.velocityNorth = horizontal_.velocityNorth();
    in.accelBiasEast = horizontal_.accelBiasEast();
    in.accelBiasNorth = horizontal_.accelBiasNorth();
    if (horizontal_.initialized()) {
        in.positionSigmaEast = horizontal_.positionSigma(0);
        in.positionSigmaNorth = horizontal_.positionSigma(1);
        in.velocitySigmaEast = horizontal_.velocitySigma(0);
        in.velocitySigmaNorth = horizontal_.velocitySigma(1);
        in.gnssInnovationEast = horizontal_.positionInnovation(0);
        in.gnssInnovationNorth = horizontal_.positionInnovation(1);
    }
    in.altimeterInnovation = estimator_.altitudeInnovation();
    in.gnssUpdateAge = lastGnssAcceptTime_ < 0.0 ? -1.0 : t - lastGnssAcceptTime_;
    in.altimeterUpdateAge = lastAltimeterAcceptTime_ < 0.0 ? -1.0 : t - lastAltimeterAcceptTime_;

    in.altitudeSetpoint = altitudeSetpoint_;
    in.velocitySetpoint = velocitySetpoint_;
    in.eastSetpoint = horizontalTarget_[0];
    in.northSetpoint = horizontalTarget_[1];
    in.eastAccelCommand = accelCommand_[0];
    in.northAccelCommand = accelCommand_[1];
    for (int i = 0; i < 3; ++i) {
        in.attitudeError[i] = attitudeError_[i];
    }
    in.command = command_;

    in.gimbal = gimbalFeedback_;
    in.engines = engineFeedback_;
    in.gnss = lastGnss_;
    in.gnssNew = gnssNew_;
    fillCatchTelemetry(t);
    downlink_.emit(hal_, in, cycle_, t);
}

double HopFlightComputer::gnssMeasurementTime(const GnssSample& gnss, double t) const {
    // Receivers report the time of validity; use it unless it is missing or in the future.
    if (gnss.fixTime >= 0.0 && gnss.fixTime <= gnss.stamp + 1.0e-9 && gnss.fixTime <= t + 1.0e-9) {
        return gnss.fixTime;
    }
    return gnss.stamp - config_.gnssLatency;
}

void HopFlightComputer::updateNavigation(double t, double dt) {
    const ImuSample imu = hal_.readImu();
    lastImu_ = imu;
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
        horizontal_.predict(specificForceWorld[0], specificForceWorld[1], dt, t);

        attitudeNewest_ = (attitudeNewest_ + 1) % HorizontalEstimator::kHistoryCapacity;
        attitudeCount_ = std::min(attitudeCount_ + 1, HorizontalEstimator::kHistoryCapacity);
        AttitudeRecord& record = attitudeHistory_[attitudeNewest_];
        record.time = t;
        for (int i = 0; i < 4; ++i) {
            record.q[i] = attitude_[i];
        }
        for (int i = 0; i < 3; ++i) {
            record.rateBody[i] = imu.angularRate[i];
        }
    }

    const Quaternion q = attitudeFrom(attitude_);

    // The altimeter looks along body -z from the base: slant range times cos(tilt).
    const AltimeterSample alt = hal_.readAltimeter();
    lastAltimeter_ = alt;
    if (alt.valid && alt.stamp > lastAltimeterStamp_) {
        lastAltimeterStamp_ = alt.stamp;
        lastAltimeterAcceptTime_ = t;
        estimator_.correct(alt.range * std::max(0.0, upComponentOfBodyZ(q)), dt);
    }

    const GnssSample gnss = hal_.readGnss();
    if (gnss.valid && gnss.stamp > lastGnssStamp_) {
        lastGnssStamp_ = gnss.stamp;
        lastGnssAcceptTime_ = t;
        lastGnss_ = gnss;
        gnssNew_ = true;
        gnssFixType_ = static_cast<int>(gnss.fixType);
        ++gnssFixCount_;

        double antennaEnu[3];
        targetFrame_.toEnu({gnss.latitude, gnss.longitude, gnss.height}, antennaEnu);

        // The solution describes the vehicle at its time of validity, not now: take the lever arm
        // from the attitude and rate of that time, then let the filter apply the fix at that time.
        const double measurementTime = gnssMeasurementTime(gnss, t);
        Quaternion qFix = q;  // [] body to world at the time of validity
        double rateFix[3] = {angularRateBody_[0], angularRateBody_[1], angularRateBody_[2]};  // [rad/s]
        double bestDelta = 1.0e30;
        for (int k = 0; k < attitudeCount_; ++k) {
            const AttitudeRecord& record = attitudeHistory_[(attitudeNewest_ - k + HorizontalEstimator::kHistoryCapacity) %
                                                            HorizontalEstimator::kHistoryCapacity];
            const double delta = std::abs(record.time - measurementTime);
            if (delta < bestDelta) {
                bestDelta = delta;
                qFix = attitudeFrom(record.q);
                for (int i = 0; i < 3; ++i) {
                    rateFix[i] = record.rateBody[i];
                }
            }
        }

        // Lever arm: antenna offset from the IMU, rotated into the local frame.
        const double lever[3] = {config_.gnssAntennaPosition[0] - config_.imuPosition[0],
                                 config_.gnssAntennaPosition[1] - config_.imuPosition[1],
                                 config_.gnssAntennaPosition[2] - config_.imuPosition[2]};
        double leverWorld[3];
        rotate(qFix, lever, leverWorld);
        double omegaCrossLever[3];
        cross(rateFix, lever, omegaCrossLever);
        double leverVelocityWorld[3];
        rotate(qFix, omegaCrossLever, leverVelocityWorld);

        const double sigma = std::max(0.005, gnss.horizontalSigma);
        horizontal_.correctDelayed(t, measurementTime, antennaEnu[0] - leverWorld[0], antennaEnu[1] - leverWorld[1], sigma,
                                   gnss.velocityEnu[0] - leverVelocityWorld[0],
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
        for (AttitudeRecord& record : attitudeHistory_) {  // the stored attitudes get the same correction
            storeAttitude(normalized(multiply(correction, attitudeFrom(record.q))), record.q);
        }
        horizontal_.removeBias(phiY * config_.gravity, -phiX * config_.gravity);
        // TODO(phase 9, item 43): use GNSS height when the altimeter is out of range.
    }

    // Sitting on the pad the velocity is known: zero.
    if (mode_ == HopMode::Prelaunch || mode_ == HopMode::Landed || mode_ == HopMode::Caught) {
        estimator_.reset(estimator_.altitude(), 0.0);
        horizontal_.zeroVelocityUpdate();
    }
}

void pinMidpointOffset(const double attitude[4], const double imuPosition[3], double pinHeight, double offsetEn[2]) {
    const double bodyDelta[3] = {-imuPosition[0], -imuPosition[1], pinHeight - imuPosition[2]};
    double world[3];
    rotate(attitudeFrom(attitude), bodyDelta, world);
    offsetEn[0] = world[0];
    offsetEn[1] = world[1];
}

void HopFlightComputer::updatePinOffset(double dt) {
    // Only the catch flight steers the pins; the pad landing keeps the IMU point (offset stays exactly zero).
    if (landingMode_ != LandingMode::Catch || mode_ == HopMode::Prelaunch) {
        return;
    }
    double raw[2];
    pinMidpointOffset(attitude_, config_.imuPosition, config_.catchConfig.pinHeight, raw);
    const double tau = config_.catchConfig.pinOffsetFilterTime;
    if (!pinOffsetValid_ || tau <= 0.0) {
        pinOffset_[0] = raw[0];
        pinOffset_[1] = raw[1];
        pinOffsetValid_ = true;
        return;
    }
    const double alpha = dt / (tau + dt);
    pinOffset_[0] += alpha * (raw[0] - pinOffset_[0]);
    pinOffset_[1] += alpha * (raw[1] - pinOffset_[1]);
}

double HopFlightComputer::horizontalError() const {
    return std::hypot(horizontalTarget_[0] - (horizontal_.east() + pinOffset_[0]),
                      horizontalTarget_[1] - (horizontal_.north() + pinOffset_[1]));
}

bool HopFlightComputer::alignedOverTarget() const {
    const double errEast = horizontalTarget_[0] - horizontal_.east();
    const double errNorth = horizontalTarget_[1] - horizontal_.north();
    const double speed = std::hypot(horizontal_.velocityEast(), horizontal_.velocityNorth());
    return translating_ && std::hypot(errEast, errNorth) < config_.landingAlignTolerance &&
           speed < config_.landingAlignSpeed;
}

void HopFlightComputer::updateTvc() {
    attitudeError_[0] = attitudeError_[1] = attitudeError_[2] = 0.0;
    for (int i = 0; i < kEngineCount; ++i) {
        const bool sea = i < kSeaLevelEngineCount;
        // The RVacs stay off for the hop (flow separation at sea level): power setting 0.
        command_.engines[i].on = sea && enginesOn_;
        command_.engines[i].throttle = (sea && enginesOn_) ? throttle_ : 0.0;
    }
    for (GimbalCommand& g : command_.gimbals) {
        g = GimbalCommand{};
    }
    if (!enginesOn_ || config_.vacuumThrust <= 0.0 || config_.engineLeverArm <= 0.0 ||
        config_.gimbalLimit <= 0.0 || config_.transverseInertiaDry <= 0.0) {
        return;
    }

    // Horizontal guidance. Until liftoff the vehicle stays upright on the pad.
    double desiredAccel[2] = {0.0, 0.0};
    if (mode_ != HopMode::Ignition && horizontal_.initialized()) {
        // Catch flight: the pin midpoint is steered to the target (pinOffset_ is zero otherwise).
        const double err[2] = {horizontalTarget_[0] - (horizontal_.east() + pinOffset_[0]),
                               horizontalTarget_[1] - (horizontal_.north() + pinOffset_[1])};
        const double distance = std::hypot(err[0], err[1]);
        // Braking profile: approach no faster than the vehicle can stop using half
        // the allowed horizontal acceleration (v = sqrt(2 * a/2 * d)).
        // The catch flight (until an abort) has its own gains and acceleration limit from liftoff on: the
        // attitude loop plus the GNSS latency limit how stiff the horizontal loops can be at a hover.
        const bool catchFlight = landingMode_ == LandingMode::Catch && mode_ != HopMode::Prelaunch;
        const double accelLimit = catchFlight ? config_.catchConfig.maxHorizontalAccel : config_.maxHorizontalAccel;
        const double speedLimit = std::min(horizontalSpeedCap_, std::sqrt(accelLimit * distance));
        const double positionKp = catchFlight ? config_.catchConfig.horizontalPositionKp : config_.horizontalPositionKp;
        const double velocityKp = catchFlight ? config_.catchConfig.horizontalVelocityKp : config_.horizontalVelocityKp;
        const double velocityKi = catchFlight ? config_.catchConfig.horizontalVelocityKi : config_.horizontalVelocityKi;
        double velocitySetpoint[2] = {positionKp * err[0], positionKp * err[1]};
        const double setpointSpeed = std::hypot(velocitySetpoint[0], velocitySetpoint[1]);
        if (setpointSpeed > speedLimit && setpointSpeed > 1e-9) {
            velocitySetpoint[0] *= speedLimit / setpointSpeed;
            velocitySetpoint[1] *= speedLimit / setpointSpeed;
        }
        const double velocityError[2] = {velocitySetpoint[0] - horizontal_.velocityEast(),
                                         velocitySetpoint[1] - horizontal_.velocityNorth()};
        const double dt = 1.0 / config_.rateHz;
        // The catch flight integrates only near its velocity setpoint: during the long translation the
        // integrator wound up (the acceleration limit was active) and overshot the seat by several metres.
        const bool integrate = !catchFlight || std::hypot(velocityError[0], velocityError[1]) < config_.catchConfig.trimEnableSpeedError;
        for (int i = 0; i < 2; ++i) {
            if (integrate) {
                horizontalTrim_[i] = std::clamp(horizontalTrim_[i] + velocityKi * velocityError[i] * dt,
                                                -config_.maxHorizontalTrim, config_.maxHorizontalTrim);
            }
            desiredAccel[i] = velocityKp * velocityError[i] + horizontalTrim_[i];
        }
        const double accel = std::hypot(desiredAccel[0], desiredAccel[1]);
        if (accel > accelLimit) {
            desiredAccel[0] *= accelLimit / accel;
            desiredAccel[1] *= accelLimit / accel;
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
    for (int i = 0; i < 3; ++i) {
        attitudeError_[i] = error[i];
    }

    // Requested torque from the attitude loops and the mass properties we were given [N m].
    const double propellantFraction =
        config_.initialPropellant > 0.0 ? std::max(0.0, propellantEstimate_) / config_.initialPropellant : 0.0;
    const double transverseInertia =
        config_.transverseInertiaDry + propellantFraction * config_.transverseInertiaPropellant;
    const double axialInertia = config_.axialInertiaDry + propellantFraction * config_.axialInertiaPropellant;

    AllocatorConfig allocator;
    allocator.leverArm = config_.engineLeverArm;
    allocator.ringRadius = config_.engineClusterRadius;
    allocator.azimuth0 = config_.engineAzimuth0;
    allocator.gimbalLimit = config_.gimbalLimit;
    allocator.rollGimbalBudget = config_.rollGimbalBudget;
    allocator.vacuumThrust = config_.vacuumThrust;
    allocator.nozzleExitArea = config_.nozzleExitArea;
    allocator.ambientPressure = config_.ambientPressure;
    allocator.minThrottle = config_.minThrottle;

    AllocatorRequest request;
    request.throttle = throttle_;
    request.torque[0] = -transverseInertia * (config_.attitudeKp * error[0] + config_.attitudeKd * angularRateBody_[0]);
    request.torque[1] = -transverseInertia * (config_.attitudeKp * error[1] + config_.attitudeKd * angularRateBody_[1]);
    request.torque[2] = -axialInertia * (config_.rollKp * error[2] + config_.rollKd * angularRateBody_[2]);

    const AllocatorResult result = allocateSeaLevelEngines(allocator, request);
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        command_.engines[i].throttle = result.throttle[i];
        command_.gimbals[i].pitch = result.pitch[i];
        command_.gimbals[i].yaw = result.yaw[i];
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
            const double translateAltitude =
                landingMode_ == LandingMode::Catch ? config_.catchConfig.translateAltitude : config_.translateAltitude;
            if (!translating_ && alt > translateAltitude) {
                translating_ = true;
                horizontalTarget_[0] = stagingPoint_[0];
                horizontalTarget_[1] = stagingPoint_[1];
            }
            // No hover: at the apex the vehicle turns straight into the descent.
            if (altitudeSetpoint_ >= config_.targetAltitude && std::abs(alt - config_.targetAltitude) < 1.0) {
                translating_ = true;
                horizontalTarget_[0] = stagingPoint_[0];
                horizontalTarget_[1] = stagingPoint_[1];
                enter(HopMode::Descent);
            }
            break;
        }

        case HopMode::Descent: {
            if (landingMode_ == LandingMode::Catch) {
                updateCatchDescent(t, dt);
                break;
            }
            // Propellant-efficient descent: fall as fast as the vehicle can still stop
            // from (a braking profile, the same physics as a hoverslam), correcting the
            // position error on the way down. The sink rate is also capped so the
            // remaining height cannot run out before the horizontal error is nulled.
            translating_ = true;
            const double height = std::max(0.0, alt - descentFloor_);
            const double profile = std::sqrt(2.0 * maxDescentBraking() * height);
            // Alignment gate: the lowest height allowed grows with the horizontal error,
            // so a vehicle that is still far off slows down instead of arriving off target.
            const double gateHeight = config_.alignmentGateSlope * horizontalError();
            const double gated = std::max(0.0, height - gateHeight) / config_.alignmentGateTime;
            const double sink = std::min({profile, gated, config_.maxSinkRate}) + config_.finalDescentRate;
            altitudeSetpoint_ = descentFloor_;
            velocitySetpoint_ = -sink;
            if (catchAborted_) {
                // Retreating from the tower after an abort: do not sink while still near the arms.
                const CatchConfig& c = config_.catchConfig;
                if (std::hypot(horizontal_.east() - c.seat[0], horizontal_.north() - c.seat[1]) < c.abortClearDistance) {
                    velocitySetpoint_ = c.abortClimbRate;
                }
            }
            throttle_ = computeThrottle(velocitySetpoint_, dt);
            if (alt < descentFloor_ + 0.5) {
                enter(HopMode::FinalDescent);
            }
            break;
        }

        case HopMode::FinalDescent:
            altitudeSetpoint_ = 0.0;
            // Slow to a stop above the target if the horizontal error is still large.
            velocitySetpoint_ = -config_.finalDescentRate *
                                std::clamp(1.0 - horizontalError() / config_.finalAlignError, 0.0, 1.0);
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

        case HopMode::CatchApproach:
        case HopMode::CatchHold:
        case HopMode::CatchFinal:
        case HopMode::Caught:
            updateCatchModes(t, dt);
            break;
    }
}

namespace {
constexpr double kMinSinkForPrediction = 0.05;  // [m/s] below this the time to the seat is not reported
}  // namespace

HopFlightComputer::CatchGeometry HopFlightComputer::catchGeometry() const {
    const CatchConfig& c = config_.catchConfig;
    CatchGeometry g;
    const double length = std::hypot(c.approachDir[0], c.approachDir[1]);
    const double d[2] = {length > 1e-9 ? c.approachDir[0] / length : 1.0, length > 1e-9 ? c.approachDir[1] / length : 0.0};
    const double b[2] = {d[1], -d[0]};  // approach side rotated +90 deg; with d = east, b = south
    // The pin midpoint (not the IMU point) against the seat.
    const double offset[2] = {horizontal_.east() + pinOffset_[0] - c.seat[0], horizontal_.north() + pinOffset_[1] - c.seat[1]};
    const double velocity[2] = {horizontal_.velocityEast(), horizontal_.velocityNorth()};
    g.alongRemaining = -(offset[0] * d[0] + offset[1] * d[1]);
    g.lateral = offset[0] * b[0] + offset[1] * b[1];
    g.horizontalError = std::hypot(offset[0], offset[1]);
    g.alongSpeed = velocity[0] * d[0] + velocity[1] * d[1];
    g.lateralSpeed = velocity[0] * b[0] + velocity[1] * b[1];
    const double seatPinHeight = c.seat[2] + c.pinHeight;  // [m] pin height when seated
    g.pinHeightAboveSeat = estimator_.altitude() + c.pinHeight - seatPinHeight;
    g.sinkRate = -estimator_.velocity();
    const Quaternion q = attitudeFrom(attitude_);
    g.tilt = std::acos(std::clamp(upComponentOfBodyZ(q), -1.0, 1.0));
    // Body +x in the world horizontal plane against the approach direction (the pins sit on body +-y).
    const double bodyX[2] = {1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y + q.w * q.z)};
    g.headingError = std::atan2(d[0] * bodyX[1] - d[1] * bodyX[0], d[0] * bodyX[0] + d[1] * bodyX[1]);
    g.funnel = c.funnelBase + c.funnelSlope * std::max(0.0, g.pinHeightAboveSeat);
    return g;
}

HopFlightComputer::CatchGates HopFlightComputer::evaluateCatchGates(const CatchGeometry& g, double horizontalSpeed) const {
    const CatchConfig& c = config_.catchConfig;
    const double h = std::max(0.0, g.pinHeightAboveSeat);
    const double profile = std::sqrt(c.contactSinkRate * c.contactSinkRate + 2.0 * c.descentDecel * h);
    CatchGates gates;
    gates.position = g.horizontalError <= g.funnel;
    gates.speed = horizontalSpeed <= c.speedGateBase + c.speedGateSlope * h && g.sinkRate <= profile + c.sinkGateMargin;
    gates.tilt = g.tilt <= c.tiltGateBase + c.tiltGateSlope * h && std::abs(g.headingError) <= c.headingGate;
    return gates;
}

bool HopFlightComputer::rtkOk(double t) const {
    // The required fix type, and solutions still arriving.
    return gnssFixType_ == config_.catchConfig.requiredFixType && lastGnssAcceptTime_ >= 0.0 &&
           t - lastGnssAcceptTime_ < config_.catchConfig.fixLossTime;
}

void HopFlightComputer::catchVertical(const CatchGeometry& g, double heightFloor, double dt) {
    // Sink rate command: the braking profile to the seat, slowed by the alignment gate while the horizontal error
    // is large, never below the contact sink rate (the descent always continues; a Ship that stays off the seat
    // point is aborted by the funnel or the timeout).
    const CatchConfig& c = config_.catchConfig;
    const double h = g.pinHeightAboveSeat;
    const double profile = std::sqrt(c.contactSinkRate * c.contactSinkRate + 2.0 * c.descentDecel * std::max(0.0, h));
    const double excess = std::max(0.0, g.horizontalError - c.gateFreeError);
    const double gated = excess > 0.0 ? std::max(0.0, h - heightFloor - c.gateSlope * excess) / c.gateTime : c.maxSinkRate;
    const double limit = std::min({profile, gated, c.maxSinkRate});
    const double sink = std::max(limit, c.contactSinkRate);
    altitudeSetpoint_ = c.seat[2];
    velocitySetpoint_ = -sink;
    // Following the profile the sink rate falls at descentDecel: feed that acceleration forward (the velocity
    // loop's lag would otherwise show as a sink rate above the profile near the seat).
    const bool onProfile = limit == profile && h > 0.0 && std::abs(estimator_.velocity() - velocitySetpoint_) < 2.0;
    throttle_ = computeThrottle(velocitySetpoint_, dt, onProfile ? c.descentDecel : 0.0);
}

void HopFlightComputer::updateCatchDescent(double t, double dt) {
    // High descent while translating to the seat point: falls on the profile, held above the entry height by the
    // alignment gate until the position error is small. No horizontal approach later: the Ship arrives over the seat.
    const CatchConfig& c = config_.catchConfig;
    const CatchGeometry g = catchGeometry();
    translating_ = true;
    catchVertical(g, c.entryHeight, dt);
    if (t - modeStartTime_ > c.stagingTimeout) {
        abortCatch(CatchAbortReason::Timeout, t);
        return;
    }
    if (g.pinHeightAboveSeat <= c.entryHeight + c.entryAltitudeBand && g.horizontalError < c.entryError) {
        rtkLostTimer_ = 0.0;
        violationTimer_ = 0.0;
        armsTriggerTime_ = -1.0;
        catchStartTime_ = t;
        mode_ = HopMode::CatchApproach;
        modeStartTime_ = t;
    }
}

void HopFlightComputer::abortCatch(CatchAbortReason reason, double t) {
    // Fall back to the pad landing: the existing Descent flies away from the tower to the pad
    // center, its alignment gate holding altitude while the error is large.
    landingMode_ = LandingMode::Pad;
    pinOffset_[0] = pinOffset_[1] = 0.0;  // the pad fallback steers the IMU point again
    catchAborted_ = true;
    abortReason_ = reason;
    horizontalTarget_[0] = 0.0;
    horizontalTarget_[1] = 0.0;
    horizontalSpeedCap_ = config_.maxHorizontalSpeed;
    descentFloor_ = config_.finalDescentAltitude;
    translating_ = true;
    touchdownTimer_ = 0.0;
    rtkLostTimer_ = 0.0;
    violationTimer_ = 0.0;
    mode_ = HopMode::Descent;
    modeStartTime_ = t;
}

void HopFlightComputer::updateCatchModes(double t, double dt) {
    const CatchConfig& c = config_.catchConfig;
    const double vel = estimator_.velocity();
    const CatchGeometry g = catchGeometry();
    const double speed = std::hypot(horizontal_.velocityEast(), horizontal_.velocityNorth());
    const double h = g.pinHeightAboveSeat;
    auto enter = [&](HopMode next) {
        mode_ = next;
        modeStartTime_ = t;
        violationTimer_ = 0.0;
    };

    if (mode_ == HopMode::Caught) {
        enginesOn_ = false;
        throttle_ = 0.0;
        velocitySetpoint_ = 0.0;
        return;
    }

    // Required fix: lose it for longer than fixLossTime and the catch is off.
    rtkLostTimer_ = rtkOk(t) ? 0.0 : rtkLostTimer_ + dt;
    const bool committed = h < c.commitHeight;
    if (!committed && rtkLostTimer_ > c.fixLossTime) {
        abortCatch(CatchAbortReason::RtkLost, t);
        return;
    }
    if (!committed && t - catchStartTime_ > c.timeout) {
        abortCatch(CatchAbortReason::Timeout, t);
        return;
    }

    // One vertical descent over the seat; the modes are height bands of it.
    catchVertical(g, 0.0, dt);
    if (armsTriggerTime_ < 0.0 && h <= c.armTriggerHeight) {
        armsTriggerTime_ = t;
    }
    if (mode_ == HopMode::CatchApproach && h <= c.armTriggerHeight) {
        enter(HopMode::CatchHold);
    }
    if (mode_ == HopMode::CatchHold && h <= c.finalHeight) {
        touchdownTimer_ = 0.0;
        enter(HopMode::CatchFinal);
    }

    const CatchGates gates = evaluateCatchGates(g, speed);
    const bool violated = !(gates.position && gates.speed && gates.tilt) && !committed;
    const CatchAbortReason reason = gates.position ? CatchAbortReason::Window : CatchAbortReason::Corridor;

    if (mode_ == HopMode::CatchFinal) {
        // Contact: the pins are at the seat and the descent has stopped (like a touchdown).
        if (h < c.contactHeight && std::abs(vel) < config_.touchdownSpeed) {
            touchdownTimer_ += dt;
        } else {
            touchdownTimer_ = 0.0;
        }
        if (touchdownTimer_ >= config_.touchdownConfirmTime) {
            enginesOn_ = false;
            throttle_ = 0.0;
            enter(HopMode::Caught);
            return;
        }
    }

    violationTimer_ = violated ? violationTimer_ + dt : 0.0;
    if (violationTimer_ >= c.abortConfirmTime) {
        abortCatch(reason, t);
    }
}

void HopFlightComputer::fillCatchTelemetry(double t) {
    const CatchConfig& c = config_.catchConfig;
    CatchTelemetryInput& out = downlinkInput_.catchInfo;
    out.landingMode = static_cast<int>(config_.landingMode);
    out.seatPoint[0] = c.seat[0];
    out.seatPoint[1] = c.seat[1];
    out.seatPoint[2] = c.seat[2];
    out.approachDir[0] = c.approachDir[0];
    out.approachDir[1] = c.approachDir[1];
    out.stagingDistance = c.stagingDistance;
    out.towerPosition[0] = c.towerPosition[0];
    out.towerPosition[1] = c.towerPosition[1];
    out.mastHalfWidth = c.mastHalfWidth;
    out.mastHeight = c.mastHeight;
    out.hingeHalfSpacing = c.hingeHalfSpacing;
    out.hingeSetback = c.hingeSetback;
    out.armLength = c.armLength;
    out.armOpenAngle = c.armOpenAngle;
    out.armClosedAngle = c.armClosedAngle;
    out.windowLateral = c.windowLateral;
    out.windowAlong = c.windowAlong;
    out.windowSink = c.windowSink;
    out.windowTilt = c.windowTilt;

    if (config_.landingMode != LandingMode::Catch) {
        return;  // Pad: phase Inactive, flags clear, measurements zero (set at construction)
    }

    const CatchGeometry g = catchGeometry();
    const double speed = std::hypot(horizontal_.velocityEast(), horizontal_.velocityNorth());
    out.alongRemaining = g.alongRemaining;
    out.lateralError = g.lateral;
    out.pinHeightAboveSeat = g.pinHeightAboveSeat;
    out.alongSpeed = g.alongSpeed;
    out.lateralSpeed = g.lateralSpeed;
    out.sinkRate = g.sinkRate;
    out.tilt = g.tilt;
    out.lateralLimit = g.funnel;
    // Time to the seat from the vertical motion: constant deceleration from the sink rate to the contact sink rate.
    out.timeToSeat = (g.sinkRate > kMinSinkForPrediction && g.pinHeightAboveSeat > 0.0)
                         ? 2.0 * g.pinHeightAboveSeat / (g.sinkRate + c.contactSinkRate)
                         : -1.0;

    const CatchGates gates = evaluateCatchGates(g, speed);
    out.positionOk = gates.position;
    out.speedOk = gates.speed;
    out.tiltOk = gates.tilt;
    out.rtkOk = rtkOk(t);
    out.corridorOk = std::abs(g.lateral) <= g.funnel;
    out.aborted = catchAborted_;
    out.go = out.positionOk && out.speedOk && out.tiltOk && out.rtkOk && out.corridorOk && !catchAborted_;
    out.abortReason = static_cast<int>(abortReason_);

    CatchPhase phase = CatchPhase::Inactive;
    if (catchAborted_) {
        phase = CatchPhase::Abort;
    } else {
        switch (mode_) {
            case HopMode::Ascent:
            case HopMode::Descent: phase = CatchPhase::Staging; break;
            case HopMode::CatchApproach: phase = CatchPhase::Approach; break;
            case HopMode::CatchHold: phase = CatchPhase::Hold; break;
            case HopMode::CatchFinal: phase = CatchPhase::Final; break;
            case HopMode::Caught: phase = CatchPhase::Caught; break;
            default: break;
        }
    }
    out.phase = static_cast<int>(phase);

    // Expected by the sequence (time since the arms-closing height), not measured: arm state is not
    // downlinked to the flight software.
    ArmsExpected arms = ArmsExpected::Open;
    if (!catchAborted_ && armsTriggerTime_ >= 0.0 && mode_ >= HopMode::CatchApproach) {
        arms = (t - armsTriggerTime_ < c.armsClosingTime) ? ArmsExpected::Closing : ArmsExpected::Closed;
    }
    out.armsExpected = static_cast<int>(arms);
}

double HopFlightComputer::massEstimate() const { return config_.dryMass + std::max(0.0, propellantEstimate_); }

double HopFlightComputer::throttleForThrust(double totalThrust) const {
    // Per engine: F = throttle * F_vac - p_amb * A_exit.
    const double perEngine = totalThrust / kSeaLevelEngineCount;
    const double throttle = (perEngine + config_.ambientPressure * config_.nozzleExitArea) / config_.vacuumThrust;
    return std::clamp(throttle, config_.minThrottle, 1.0);
}

double HopFlightComputer::minimumTotalThrust() const {
    return kSeaLevelEngineCount *
           std::max(0.0, config_.minThrottle * config_.vacuumThrust - config_.ambientPressure * config_.nozzleExitArea);
}

double HopFlightComputer::computeThrottle(double velocitySetpoint, double dt, double accelFeedforward) {
    const double accelCommand = velocityPid_.update(velocitySetpoint, estimator_.velocity(), dt) + accelFeedforward;
    // Only the vertical component of a tilted thrust vector holds up the weight.
    const double cosTilt = std::max(0.7, upComponentOfBodyZ(attitudeFrom(attitude_)));
    return throttleForThrust(massEstimate() * (config_.gravity + accelCommand) / cosTilt);
}

double HopFlightComputer::maxDescentBraking() const {
    // Net upward deceleration at full throttle, derated by the margin so the throttle
    // loop keeps authority.
    const double maxThrust =
        kSeaLevelEngineCount * std::max(0.0, config_.vacuumThrust - config_.ambientPressure * config_.nozzleExitArea);
    const double net = maxThrust / massEstimate() - config_.gravity;
    return std::max(0.1, net) * config_.descentBrakingMargin;
}

double HopFlightComputer::maxClimbRateToStopWithin(double distance) const {
    // At minimum throttle the net deceleration is g - Tmin/m. Climbing faster than
    // sqrt(2 a d) means the vehicle cannot stop before the target: it overshoots.
    const double brakingDecel = config_.gravity - minimumTotalThrust() / massEstimate();
    const double usable = std::max(0.1, brakingDecel) * config_.brakingMargin;
    return std::sqrt(2.0 * usable * std::max(0.0, distance));
}

}  // namespace fsw
