#pragma once

#include "fsw/Hal.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

namespace fsw {

// Vertical hop: ignite, climb to a target altitude, hover, descend, land.
// Cascade control:
//   altitude error -> velocity setpoint (P + setpoint-rate feedforward)
//   velocity error -> acceleration command (PI)
//   acceleration   -> throttle via estimated mass and known max thrust
struct HopConfig {
    double rateHz = 100.0;
    double ignitionTime = 1.0;        // [s] after sim start
    double targetAltitude = 50.0;     // [m]
    double ascentRate = 5.0;          // [m/s] setpoint ramp
    double hoverDuration = 5.0;       // [s]
    double descentRate = 3.0;         // [m/s] setpoint ramp
    double finalDescentAltitude = 5.0;  // [m] switch to constant-rate descent
    double finalDescentRate = 1.0;    // [m/s]
    double liftoffAltitude = 0.3;     // [m] estimate above this counts as liftoff
    double ignitionThrottle = 0.6;    // held until liftoff
    // Touchdown = low altitude AND the vehicle has stopped moving, held for a while.
    // (Cutting the engine at an altitude instead drops the vehicle the rest of the way.)
    double touchdownAltitude = 0.3;      // [m]
    double touchdownSpeed = 0.25;        // [m/s]
    double touchdownConfirmTime = 0.3;   // [s]
    // Fraction of the braking deceleration available at minimum throttle that the
    // climb guidance plans to use. The throttle floor limits how hard a climb can stop.
    double brakingMargin = 0.6;

    // Vehicle knowledge the flight software is given (not read from the sim)
    double dryMass = 0.0;
    double initialPropellant = 0.0;
    double maxThrust = 0.0;
    double ispSeaLevel = 0.0;
    double minThrottle = 0.0;
    double gravity = 9.80665;

    double altitudeKp = 0.8;  // [1/s]
    double maxClimbRate = 6.0;
    double maxSinkRate = 4.0;
    PidGains velocityGains{2.0, 0.6, 0.0, -4.0, 6.0, 2.0};  // output: accel [m/s^2]
};

enum class HopMode : int {
    Prelaunch = 0,
    Ignition = 1,
    Ascent = 2,
    Hover = 3,
    Descent = 4,
    FinalDescent = 5,
    Landed = 6,
};

const char* toString(HopMode mode);

class HopFlightComputer {
public:
    HopFlightComputer(Hal& hal, HopConfig config);

    // Called once per flight software cycle (config.rateHz).
    void tick();

    HopMode mode() const { return mode_; }
    double altitudeEstimate() const { return estimator_.altitude(); }
    double velocityEstimate() const { return estimator_.velocity(); }

private:
    void updateNavigation(double dt);
    void updateModeLogic(double t, double dt);
    double computeThrottle(double velocitySetpoint, double dt);
    double maxClimbRateToStopWithin(double distance) const;

    Hal& hal_;
    HopConfig config_;
    VerticalEstimator estimator_;
    Pid velocityPid_;

    HopMode mode_ = HopMode::Prelaunch;
    double lastTime_ = -1.0;
    double modeStartTime_ = 0.0;
    double lastImuStamp_ = -1.0;
    double lastAltimeterStamp_ = -1.0;
    double altitudeSetpoint_ = 0.0;
    double velocitySetpoint_ = 0.0;
    double propellantEstimate_ = 0.0;
    double touchdownTimer_ = 0.0;
    ActuatorCommand command_;
};

}  // namespace fsw
