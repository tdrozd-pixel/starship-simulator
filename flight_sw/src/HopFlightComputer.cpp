#include "fsw/HopFlightComputer.hpp"

#include <algorithm>
#include <cmath>

namespace fsw {

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
        lastImuStamp_ = imu.stamp;
        estimator_.predict(imu.specificForce[2], dt);
    }

    const AltimeterSample alt = hal_.readAltimeter();
    if (alt.valid && alt.stamp > lastAltimeterStamp_) {
        lastAltimeterStamp_ = alt.stamp;
        estimator_.correct(alt.range, dt);
    }

    // Sitting on the pad the estimate is known: zero velocity.
    if (mode_ == HopMode::Prelaunch || mode_ == HopMode::Landed) {
        estimator_.reset(estimator_.altitude(), 0.0);
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
