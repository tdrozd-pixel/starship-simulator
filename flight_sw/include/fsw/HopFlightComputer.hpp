#pragma once

#include <cmath>
#include <cstdint>

#include "fsw/Geodesy.hpp"
#include "fsw/GncTelemetry.hpp"
#include "fsw/Hal.hpp"
#include "fsw/HorizontalEstimator.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

namespace fsw {

// Hop with a divert: ignite, climb, translate to the landing target given as
// WGS-84 coordinates, then descend on a braking profile while correcting position.
//
// Vertical cascade:
//   altitude error -> velocity setpoint (P + setpoint-rate feedforward)
//   velocity error -> acceleration command (PI)
//   acceleration   -> throttle via estimated mass and the known engine thrust model
// Horizontal cascade (GNSS + IMU navigation in the target's local ENU frame):
//   position error -> velocity setpoint (P, limited by a braking profile)
//   velocity error -> acceleration -> tilt -> attitude PD -> common engine gimbal
// Torque allocation (EngineAllocator.hpp): the attitude loops request a body torque; roll comes
// from tangential gimbal on the three sea-level engines (a reserved share of the gimbal range),
// pitch/yaw from common gimbal first and differential throttle when the gimbal saturates.
// The three Raptor Vacuums are commanded off for the hop (they cannot run at sea level).
// Where the hop ends. Pad: today's braking descent onto the pad center. Catch: translate to a point
// over the tower seat early, then descend vertically between the open arms and settle onto the
// cradles (catch tower feature).
enum class LandingMode : int {
    Pad = 0,
    Catch = 1,
};

// Mission data for the catch (loaded before flight, like the landing target). All positions
// are in the target frame (local ENU, origin at the landing target, z up). Defaults are the
// values the sim derives from TowerParams/HopperParams (sim_vehicle/TowerParams.hpp); the
// scenario builder overwrites them. Unused when landingMode == Pad.
//
// Sequence: Ascent translates to the seat point; Descent keeps translating while it falls on the
// braking profile; CatchApproach, CatchHold and CatchFinal are height bands of one vertical
// descent over the seat point (no horizontal approach): Approach from entryHeight, Hold (arms
// expected closing) from armTriggerHeight, Final from finalHeight. All gates below are about 2x
// the physical catch window (tower side), tapering with height above the seat.
struct CatchConfig {
    double seat[3] = {42.0, 0.0, 45.25};  // [m] east, north, seated base altitude
    double approachDir[2] = {1.0, 0.0};   // [] unit vector east, north (the tower side; defines along/lateral)
    double pinHeight = 30.0;              // [m] catch pin height above the base
    // The pin midpoint (body +z * pinHeight) leads the IMU point through tilt. Guidance steers the pin midpoint to
    // the seat using its horizontal offset from the IMU point, first-order filtered with this time constant.
    // Placeholder tuning, chosen from 100-seed batches: the raw offset (0) and 0.5 to 1 s hurt the head wind case
    // (tilt rate leads the pins); 4 s keeps the quasi-steady wind trim and gave the best crosswind results.
    double pinOffsetFilterTime = 4.0;     // [s]
    double stagingDistance = 0.0;         // [m] translation target is this far before the seat, along -approachDir (0: over the seat)
    double translateAltitude = 60.0;      // [m] Ascent starts translating to the seat above this altitude
    double entryHeight = 90.0;            // [m] base height above the seat where Descent hands over to CatchApproach
    double entryAltitudeBand = 5.0;       // [m] ... the handover may happen this far above entryHeight
    double entryError = 6.0;              // [m] ... if the horizontal error to the seat is below this
    double horizontalSpeedCap = 5.0;      // [m/s] cap on the horizontal velocity setpoint in the catch flight
    double maxHorizontalAccel = 1.0;      // [m/s^2] horizontal acceleration limit in the catch flight
    // Vertical profile: sink rate = sqrt(contactSinkRate^2 + 2 * descentDecel * height above the seat),
    // so the Ship arrives at the seat at contactSinkRate after a constant deceleration. Throttle
    // authority (about 3.9 m/s^2 net at full throttle on 450 t) leaves margin over descentDecel.
    double descentDecel = 1.5;            // [m/s^2] profile deceleration
    double contactSinkRate = 0.5;         // [m/s] sink rate at the seat (inside the 1.5 m/s catch window)
    double maxSinkRate = 30.0;            // [m/s] sink rate cap
    // Alignment gate: sink is also limited to (height - floor - gateSlope * (error - gateFreeError)) / gateTime,
    // so a Ship that is still off the seat point slows down instead of arriving off target.
    double gateSlope = 10.0;              // [m of height per m of error]
    double gateFreeError = 3.0;           // [m] error below this does not slow the descent
    double gateTime = 2.0;                // [s]
    // Height bands, pin height above the seat.
    double armTriggerHeight = 45.0;       // [m] the tower starts closing the arms at about this height (mission data)
    double armsClosingTime = 6.0;         // [s] arm closing time, tower servo estimate 5.7 s plus margin; the profile gives at least this from armTriggerHeight
    double finalHeight = 8.0;             // [m] CatchFinal below this
    double commitHeight = 1.0;            // [m] below this height aborts are no longer allowed
    double contactHeight = 0.4;           // [m] pin height above the seat that counts as contact (with a stopped descent)
    // Gates (about 2x the physical window, widening with height). Violations persist abortConfirmTime to abort.
    double funnelBase = 1.2;              // [m] horizontal error allowed at the seat (catch window 0.7 m)
    double funnelSlope = 0.06;            // [m/m] widening per metre of height above the seat (7 m at 100 m)
    double speedGateBase = 1.0;           // [m/s] horizontal speed allowed at the seat (catch window 0.5 m/s)
    double speedGateSlope = 0.05;         // [m/s per m] widening with height
    double sinkGateMargin = 1.5;          // [m/s] sink rate allowed above the profile (about the 1.5 m/s window)
    double tiltGateBase = 6.0 * 3.14159265358979323846 / 180.0;    // [rad] tilt at the seat (catch window 3 deg)
    double tiltGateSlope = 0.1 * 3.14159265358979323846 / 180.0;   // [rad per m] widening with height
    double headingGate = 6.0 * 3.14159265358979323846 / 180.0;     // [rad] heading error (catch window 3 deg)
    double abortConfirmTime = 0.6;        // [s] a gate violation must persist this long to abort
    double windowLateral = 0.7;           // [m] catch window, lateral offset from the pad center (tower side, telemetry only)
    double windowAlong = 1.5;             // [m] catch window, offset along the axis (telemetry only)
    double windowSink = 1.5;              // [m/s] catch window, sink rate
    double windowHorizontalSpeed = 0.5;   // [m/s] catch window, horizontal speed
    double windowTilt = 3.0 * 3.14159265358979323846 / 180.0;     // [rad] catch window, tilt
    double windowHeading = 3.0 * 3.14159265358979323846 / 180.0;  // [rad] catch window, heading error
    double stagingTimeout = 60.0;         // [s] Descent time limit while catching (the alignment gate can hold the sink)
    double timeout = 40.0;                // [s] limit for CatchApproach to Caught
    int requiredFixType = 2;              // GnssFixType::RtkFixed; catch phases need it
    double fixLossTime = 4.0;             // [s] without the required fix (or fresh solutions) before aborting
    // After an abort the pins may sit over closed arms: climb while this close to the seat, so the
    // retreat to the pad does not descend onto the arms. TODO(phase 7, item 40): estimates.
    double abortClearDistance = 8.0;      // [m] horizontal distance from the seat to be clear of the arms
    double abortClimbRate = 1.0;          // [m/s] climb rate while closer than that
    // Horizontal loop gains used in the whole catch flight from liftoff on (Pad flights and aborted
    // catches use the HopConfig gains). TODO(phase 7, item 40): estimates tuned in the catch scenario.
    double horizontalPositionKp = 0.2;    // [1/s]
    double horizontalVelocityKp = 0.4;    // [1/s]
    double horizontalVelocityKi = 0.1;    // [1/s^2]
    double trimEnableSpeedError = 1.0;    // [m/s] the velocity loop integrates only while the horizontal velocity error is below this
    // Tower geometry for the catch_config downlink (informational; guidance does not use it).
    double towerPosition[2] = {60.0, 0.0};  // [m] east, north
    double mastHalfWidth = 6.0;             // [m]
    double mastHeight = 145.0;              // [m]
    double hingeHalfSpacing = 8.0;          // [m]
    double hingeSetback = 6.0;              // [m]
    double armLength = 16.0;                // [m]
    double armOpenAngle = 20.0 * 3.14159265358979323846 / 180.0;    // [rad] outward from the approach axis
    double armClosedAngle = 4.8 * 3.14159265358979323846 / 180.0;   // [rad] inward from the approach axis

    // Nominal time from this height above the seat to the seat on the profile [s] (pure).
    double nominalTimeToSeat(double height) const {
        const double h = height > 0.0 ? height : 0.0;
        return (std::sqrt(contactSinkRate * contactSinkRate + 2.0 * descentDecel * h) - contactSinkRate) / descentDecel;
    }
};

struct HopConfig {
    double rateHz = 100.0;
    double ignitionTime = 1.0;        // [s] after sim start (earliest; also needs GNSS)
    int gnssFixesRequired = 10;       // valid fixes before ignition is allowed
    double targetAltitude = 250.0;     // [m]
    double ascentRate = 100.0;          // [m/s] setpoint ramp
    // Fraction of the full-throttle net deceleration the descent braking profile plans to use.
    double descentBrakingMargin = 0.6;
    // Descent gate: do not get lower than slope * horizontal error [m of height per m of
    // error]; closing on that limit takes at least gateTime seconds.
    double alignmentGateSlope = 3.0;
    double alignmentGateTime = 2.0;      // [s]
    double finalAlignError = 1.0;        // [m] final descent stops above this error
    double finalDescentAltitude = 1.0;  // [m] switch to constant-rate descent
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
    double translateAltitude = 200.0;     // [m] hold over the liftoff point below this
    double landingAlignTolerance = 10;  // [m] horizontal error allowed before descent
    double landingAlignSpeed = 10;      // [m/s] horizontal speed allowed before descent

    // Vehicle knowledge the flight software is given (not read from the sim).
    double dryMass = 0.0;            // [kg]
    double initialPropellant = 0.0;  // [kg]
    double vacuumThrust = 0.0;       // [N] per engine at full rated flow
    double nozzleExitArea = 0.0;     // [m^2] per engine
    double vacuumIsp = 0.0;          // [s]
    // Raptor Vacuum (outer ring, fixed). The hop never lights them; they are here so the
    // propellant bookkeeping stays right if they are ever commanded on.
    double rvacVacuumThrust = 0.0;   // [N] per engine at full rated flow
    double rvacVacuumIsp = 0.0;      // [s]
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
    // Delay from a GNSS solution's time of validity to its arrival here. Used only when a sample
    // carries no usable time of validity (the receiver reports one, and the filter uses that).
    // 50 ms is the receiver spec the flight computer is configured for (generic receiver
    // figure, estimate; the simulator's receiver model is built to the same number, and a test
    // pins the two together). Flight software never reads the sensor model.
    double gnssLatency = 0.05;                 // [s]

    double altitudeKp = 0.8;  // [1/s]
    double maxClimbRate = 100.0;
    double maxSinkRate = 40.0;
    PidGains velocityGains{2.0, 0.6, 0.0, -4.0, 6.0, 2.0};  // output: accel [m/s^2]
    // The Ship is slow in attitude (lever arm ~9.5 m against a ~10 m radius of
    // gyration; a gimbal deflection first pushes the base the wrong way), so the
    // horizontal loops are kept well below the attitude loop bandwidth.
    double horizontalPositionKp = 0.1;  // [1/s] was .08
    double maxHorizontalSpeed = 100.0;     // [m/s]
    double horizontalVelocityKp = 0.4;   // [1/s] was .3
    // Trims steady disturbances: wind drag, and the tilt error of the gyro-integrated
    // attitude (about 0.2 deg after a minute with the placeholder IMU), which a level
    // controller cannot see. TODO(phase 9, item 43): GNSS-aided attitude in the EKF.
    double horizontalVelocityKi = 0.14;  // [1/s^2] was 0.06
    double maxHorizontalTrim = 1.0;      // [m/s^2] integrator limit
    double maxHorizontalAccel = 1.5;     // [m/s^2]
    double attitudeKp = 2.25;            // [1/s^2]
    double attitudeKd = 2.0;             // [1/s]
    double rollKp = 0.2;                 // [1/s^2]
    double rollKd = 0.8;                 // [1/s]
    // Share of the gimbal range reserved for roll (tangential tilt). Pitch/yaw common gimbal
    // gets gimbalLimit minus this, so roll always has headroom.
    double rollGimbalBudget = 2.0 * 3.14159265358979323846 / 180.0;  // [rad]

    // Catch tower (see CatchConfig). Pad mode never reads catchConfig.
    LandingMode landingMode = LandingMode::Pad;
    CatchConfig catchConfig;
};

enum class HopMode : int {
    Prelaunch = 0,
    Ignition = 1,
    Ascent = 2,
    // 3 was Hover, removed: the hop goes from Ascent straight into Descent. The value
    // stays reserved so the downlinked mode numbers do not change.
    Descent = 4,
    FinalDescent = 5,
    Landed = 6,
    // Catch landing (LandingMode::Catch only): height bands of one vertical descent over the seat
    // (Approach from entryHeight, Hold = arms closing, Final = last stretch), then Caught.
    CatchApproach = 7,
    CatchHold = 8,
    CatchFinal = 9,
    Caught = 10,
};

const char* toString(HopMode mode);

// Horizontal offset (east, north) [m] of the catch pin midpoint from the IMU point, in world ENU. The pins sit at
// body (0, 0, pinHeight) above the base and the IMU at imuPosition (body frame, from the base); `attitude` is the
// body-to-world quaternion (w, x, y, z).
void pinMidpointOffset(const double attitude[4], const double imuPosition[3], double pinHeight, double offsetEn[2]);

class HopFlightComputer {
public:
    HopFlightComputer(Hal& hal, HopConfig config);

    // Called once per flight software cycle (config.rateHz).
    void tick();

    HopMode mode() const { return mode_; }
    // Runtime landing mode: starts as config.landingMode, becomes Pad when a catch is aborted.
    LandingMode landingMode() const { return landingMode_; }
    bool catchAborted() const { return catchAborted_; }
    CatchAbortReason catchAbortReason() const { return abortReason_; }
    double altitudeEstimate() const { return estimator_.altitude(); }
    double velocityEstimate() const { return estimator_.velocity(); }
    const HorizontalEstimator& horizontal() const { return horizontal_; }

private:
    void updateNavigation(double t, double dt);
    void updateModeLogic(double t, double dt);
    void updateTvc();
    void updatePinOffset(double dt);
    double computeThrottle(double velocitySetpoint, double dt, double accelFeedforward = 0.0);
    double throttleForThrust(double totalThrust) const;
    double minimumTotalThrust() const;
    double massEstimate() const;
    void downlinkTelemetry(double t);
    double maxClimbRateToStopWithin(double distance) const;
    double maxDescentBraking() const;
    bool alignedOverTarget() const;
    double horizontalError() const;
    // Catch guidance (LandingMode::Catch only).
    struct CatchGeometry {
        double alongRemaining = 0.0;  // [m] along-axis offset, positive = short of the seat (toward the approach side)
        double horizontalError = 0.0; // [m] horizontal distance to the seat point
        double lateral = 0.0;         // [m] positive toward +b
        double alongSpeed = 0.0;      // [m/s]
        double lateralSpeed = 0.0;    // [m/s]
        double pinHeightAboveSeat = 0.0;  // [m]
        double sinkRate = 0.0;        // [m/s] positive down
        double tilt = 0.0;            // [rad]
        double headingError = 0.0;    // [rad]
        double funnel = 0.0;          // [m] horizontal error allowed at this height
    };
    CatchGeometry catchGeometry() const;
    struct CatchGates {
        bool position = true;  // horizontal error inside the funnel
        bool speed = true;     // horizontal speed and sink rate inside their gates
        bool tilt = true;      // tilt and heading inside their gates
    };
    CatchGates evaluateCatchGates(const CatchGeometry& g, double horizontalSpeed) const;
    bool rtkOk(double t) const;
    void updateCatchDescent(double t, double dt);
    void catchVertical(const CatchGeometry& g, double heightFloor, double dt);
    void abortCatch(CatchAbortReason reason, double t);
    void updateCatchModes(double t, double dt);
    double gnssMeasurementTime(const GnssSample& gnss, double t) const;
    void fillCatchTelemetry(double t);

    Hal& hal_;
    HopConfig config_;
    LocalEnuFrame targetFrame_;
    VerticalEstimator estimator_;
    HorizontalEstimator horizontal_;
    Pid velocityPid_;

    double attitude_[4] = {1.0, 0.0, 0.0, 0.0};  // estimated body to world
    // Attitude history, one entry per IMU update, so a delayed GNSS fix gets its lever arm
    // from the attitude and rate of its own time of validity.
    struct AttitudeRecord {
        double time = 0.0;                // [s]
        double q[4] = {1.0, 0.0, 0.0, 0.0};  // estimated body to world
        double rateBody[3] = {};          // [rad/s]
    };
    AttitudeRecord attitudeHistory_[HorizontalEstimator::kHistoryCapacity];
    int attitudeNewest_ = 0;
    int attitudeCount_ = 0;
    double angularRateBody_[3] = {};              // measured body [rad/s]
    HopMode mode_ = HopMode::Prelaunch;
    LandingMode landingMode_ = LandingMode::Pad;
    bool catchAborted_ = false;
    CatchAbortReason abortReason_ = CatchAbortReason::None;
    double stagingPoint_[2] = {};        // [m] east, north: where Ascent translates to (pad: the target)
    double descentFloor_ = 0.0;          // [m] Descent altitude floor
    double horizontalSpeedCap_ = 0.0;    // [m/s] cap on the horizontal velocity setpoint
    double rtkLostTimer_ = 0.0;          // [s]
    double violationTimer_ = 0.0;        // [s]
    double armsTriggerTime_ = -1.0;      // [s] flight time the arms-closing height was reached, negative: not yet
    double catchStartTime_ = 0.0;        // [s] flight time CatchApproach was entered
    double lastTime_ = -1.0;
    double modeStartTime_ = 0.0;
    double lastImuStamp_ = -1.0;
    double lastAltimeterStamp_ = -1.0;
    double lastGnssStamp_ = -1.0;
    double lastGnssAcceptTime_ = -1.0;       // [s] flight time of the last accepted GNSS fix, negative: none
    double lastAltimeterAcceptTime_ = -1.0;  // [s] flight time of the last accepted altimeter reading, negative: none
    int gnssFixCount_ = 0;
    int gnssFixType_ = 0;
    double altitudeSetpoint_ = 0.0;
    double velocitySetpoint_ = 0.0;
    double horizontalTarget_[2] = {};  // [m] east, north in the target frame
    double horizontalTrim_[2] = {};    // [m/s^2] velocity loop integrator
    double pinOffset_[2] = {};         // [m] east, north: catch pin midpoint minus IMU point (filtered; zero for pad landings)
    bool pinOffsetValid_ = false;
    double accelCommand_[2] = {};      // [m/s^2] last horizontal acceleration command
    bool translating_ = false;
    double propellantEstimate_ = 0.0;
    double touchdownTimer_ = 0.0;
    bool enginesOn_ = false;
    double throttle_ = 0.0;
    ActuatorCommand command_;

    // Downlink state (read only by telemetry; control never looks at it).
    GncTelemetry downlink_;
    GncTelemetryInput downlinkInput_;
    uint32_t cycle_ = 0;
    double attitudeError_[3] = {};  // [rad] body, zero when not computed
    ImuSample lastImu_;
    AltimeterSample lastAltimeter_;
    GimbalFeedbackSample gimbalFeedback_;
    EngineFeedbackSample engineFeedback_;
    GnssSample lastGnss_;  // latest accepted solution
    bool gnssNew_ = false;
};

}  // namespace fsw
