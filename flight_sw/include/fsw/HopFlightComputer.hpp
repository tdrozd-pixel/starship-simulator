#pragma once

#include "fsw/Geodesy.hpp"
#include "fsw/Hal.hpp"
#include "fsw/HorizontalEstimator.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

namespace fsw {

// Hop with a divert: ignite, climb, translate to the landing target given as
// WGS-84 coordinates, hover over it, descend, land on it.
//
// Vertical cascade:
//   altitude error -> velocity setpoint (P + setpoint-rate feedforward)
//   velocity error -> acceleration command (PI)
//   acceleration   -> throttle via estimated mass and the known engine thrust model
// Horizontal cascade (GNSS + IMU navigation in the target's local ENU frame):
//   position error -> velocity setpoint (P, limited by a braking profile)
//   velocity error -> acceleration -> tilt -> attitude PD -> common engine gimbal
// Roll: differential gimbal across the three engines.
struct HopConfig {
    double rateHz = 100.0;
    double ignitionTime = 1.0;        // [s] after sim start (earliest; also needs GNSS)
    int gnssFixesRequired = 10;       // valid fixes before ignition is allowed
    double targetAltitude = 50.0;     // [m]
    double ascentRate = 5.0;          // [m/s] setpoint ramp
    double hoverDuration = 5.0;       // [s] minimum
    double descentRate = 3.0;         // [m/s] setpoint ramp
    double finalDescentAltitude = 5.0;  // [m] switch to constant-rate descent
    double finalDescentRate = 1.0;    // [m/s]
    double liftoffAltitude = 0.3;     // [m] estimate above this counts as liftoff
    double ignitionThrustToWeight = 1.15;  // held until liftoff
    // Touchdown = low altitude AND the vehicle has stopped moving, held for a while.
    // (Cutting the engine at an altitude instead drops the vehicle the rest of the way.)
    double touchdownAltitude = 0.3;      // [m]
    double touchdownSpeed = 0.25;        // [m/s]
    double touchdownConfirmTime = 0.3;   // [s]
    // Fraction of the braking deceleration available at minimum throttle that the
    // climb guidance plans to use. The throttle floor limits how hard a climb can stop.
    double brakingMargin = 0.6;

    // Landing target (mission data loaded before flight).
    GeodeticPosition target;
    double translateAltitude = 20.0;     // [m] hold over the liftoff point below this
    double landingAlignTolerance = 0.3;  // [m] horizontal error allowed before descent
    double landingAlignSpeed = 0.2;      // [m/s] horizontal speed allowed before descent

    // Vehicle knowledge the flight software is given (not read from the sim).
    double dryMass = 0.0;            // [kg]
    double initialPropellant = 0.0;  // [kg]
    double vacuumThrust = 0.0;       // [N] per engine at full rated flow
    double nozzleExitArea = 0.0;     // [m^2] per engine
    double vacuumIsp = 0.0;          // [s]
    double ambientPressure = 101325.0;  // [Pa] assumed at the pad (no air data sensor yet)
    double minThrottle = 0.0;
    double gravity = 9.80665;
    double engineLeverArm = 0.0;        // [m] CG to the gimbal plane
    double engineClusterRadius = 0.0;   // [m]
    double engineAzimuth0 = 0.0;        // [rad] azimuth of engine 0 from body +x toward +y
    double gimbalLimit = 0.0;           // [rad]
    double transverseInertiaDry = 0.0;         // [kg m^2] about the CG
    double transverseInertiaPropellant = 0.0;  // [kg m^2] at the initial load
    double axialInertiaDry = 0.0;              // [kg m^2]
    double axialInertiaPropellant = 0.0;       // [kg m^2] at the initial load
    double gnssAntennaPosition[3] = {};        // [m] body frame, from the base
    // Navigation reference point = the IMU, where acceleration is measured. Guidance
    // puts this point over the target; upright, the base is directly below it.
    double imuPosition[3] = {};                // [m] body frame, from the base
    double gnssVelocitySigma = 0.05;           // [m/s] assumed GNSS velocity accuracy

    double altitudeKp = 0.8;  // [1/s]
    double maxClimbRate = 6.0;
    double maxSinkRate = 4.0;
    PidGains velocityGains{2.0, 0.6, 0.0, -4.0, 6.0, 2.0};  // output: accel [m/s^2]
    // The Ship is slow in attitude (lever arm ~9.5 m against a ~10 m radius of
    // gyration; a gimbal deflection first pushes the base the wrong way), so the
    // horizontal loops are kept well below the attitude loop bandwidth.
    double horizontalPositionKp = 0.08;  // [1/s]
    double maxHorizontalSpeed = 3.0;     // [m/s]
    double horizontalVelocityKp = 0.3;   // [1/s]
    // Trims steady disturbances: wind drag, and the tilt error of the gyro-integrated
    // attitude (about 0.2 deg after a minute with the placeholder IMU), which a level
    // controller cannot see. TODO(phase 9, item 43): GNSS-aided attitude in the EKF.
    double horizontalVelocityKi = 0.06;  // [1/s^2]
    double maxHorizontalTrim = 0.3;      // [m/s^2] integrator limit
    double maxHorizontalAccel = 0.6;     // [m/s^2]
    double attitudeKp = 2.25;            // [1/s^2]
    double attitudeKd = 3.0;             // [1/s]
    double rollKp = 0.2;                 // [1/s^2]
    double rollKd = 0.8;                 // [1/s]
    double maxRollGimbal = 2.0 * 3.14159265358979323846 / 180.0;  // [rad] differential share
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
    const HorizontalEstimator& horizontal() const { return horizontal_; }

private:
    void updateNavigation(double t, double dt);
    void updateModeLogic(double t, double dt);
    void updateTvc();
    double computeThrottle(double velocitySetpoint, double dt);
    double throttleForThrust(double totalThrust) const;
    double minimumTotalThrust() const;
    double massEstimate() const;
    double maxClimbRateToStopWithin(double distance) const;
    bool alignedOverTarget() const;

    Hal& hal_;
    HopConfig config_;
    LocalEnuFrame targetFrame_;
    VerticalEstimator estimator_;
    HorizontalEstimator horizontal_;
    Pid velocityPid_;

    double attitude_[4] = {1.0, 0.0, 0.0, 0.0};  // estimated body to world
    double angularRateBody_[3] = {};              // measured body [rad/s]
    HopMode mode_ = HopMode::Prelaunch;
    double lastTime_ = -1.0;
    double modeStartTime_ = 0.0;
    double lastImuStamp_ = -1.0;
    double lastAltimeterStamp_ = -1.0;
    double lastGnssStamp_ = -1.0;
    int gnssFixCount_ = 0;
    int gnssFixType_ = 0;
    double altitudeSetpoint_ = 0.0;
    double velocitySetpoint_ = 0.0;
    double horizontalTarget_[2] = {};  // [m] east, north in the target frame
    double horizontalTrim_[2] = {};    // [m/s^2] velocity loop integrator
    double accelCommand_[2] = {};      // [m/s^2] last horizontal acceleration command
    bool translating_ = false;
    double propellantEstimate_ = 0.0;
    double touchdownTimer_ = 0.0;
    bool enginesOn_ = false;
    double throttle_ = 0.0;
    ActuatorCommand command_;
};

}  // namespace fsw
