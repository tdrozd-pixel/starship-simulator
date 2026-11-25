#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sim/core/Bus.hpp"
#include "sim/core/ContinuousSystem.hpp"
#include "sim/env/Atmosphere.hpp"
#include "sim/env/Gravity.hpp"
#include "sim/env/Weather.hpp"
#include "sim/vehicle/HopperParams.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

using EngineVector = Eigen::Matrix<double, kEngineCount, 1>;  // all six engines
using EngineGimbals = Eigen::Matrix<double, 2, kSeaLevelEngineCount>;  // sea-level engines; row 0 pitch, row 1 yaw [rad]
using EngineState = fsw::EngineState;

// Frames: local ENU world frame (z up), body frame with +z along the vehicle
// axis toward the nose. q rotates body vectors into the world frame.
// `position` and `velocity` are the center of mass. Use basePosition() for the
// vehicle base (the reference point of the truth stream, altimeter and contact).
struct HopperState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();         // CG, world [m]
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // CG, world [m/s]
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  // body -> world
    Eigen::Vector3d angularRate = Eigen::Vector3d::Zero();      // body [rad/s]
    double propellantMass = 0.0;                               // [kg]
    EngineVector throttle = EngineVector::Zero();              // actual, per engine (0..2 SL, 3..5 RVac)
    EngineGimbals gimbal = EngineGimbals::Zero();              // actual, per sea-level engine [rad]
};

struct HopperDeriv {
    Eigen::Vector3d positionDot = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocityDot = Eigen::Vector3d::Zero();
    Eigen::Vector4d attitudeDot = Eigen::Vector4d::Zero();  // quaternion coeffs (x, y, z, w)
    Eigen::Vector3d angularRateDot = Eigen::Vector3d::Zero();
    double propellantMassDot = 0.0;
    EngineVector throttleDot = EngineVector::Zero();
    EngineGimbals gimbalDot = EngineGimbals::Zero();
};

HopperDeriv operator+(const HopperDeriv& a, const HopperDeriv& b);
HopperDeriv operator*(double s, const HopperDeriv& d);
HopperState addScaled(const HopperState& x, const HopperDeriv& dx, double h);

struct TouchdownReport {
    bool occurred = false;
    double time = 0.0;
    double verticalSpeed = 0.0;    // [m/s], positive down
    double horizontalSpeed = 0.0;  // [m/s]
    double east = 0.0;             // base position at touchdown [m]
    double north = 0.0;            // [m]
};

// Placeholder mass properties: dry mass as a thin cylindrical shell centered at
// its CG, propellant as a solid column. The CG is fixed at the initial load.
// TODO(phase 1, item 8): tanks, live fill levels, moving CG.
struct HopperMassProperties {
    double cgHeight = 0.0;                     // [m] above base
    double transverseInertiaDry = 0.0;         // [kg m^2] about the vehicle CG
    double axialInertiaDry = 0.0;              // [kg m^2]
    double transverseInertiaPropellant = 0.0;  // [kg m^2] at the initial load
    double axialInertiaPropellant = 0.0;       // [kg m^2] at the initial load
};

HopperMassProperties hopperMassProperties(const HopperParams& params);

// What holds the vehicle. Free has no numeric effect. HeldByArms: caught by the tower arms
// (pose frozen, velocity and rates zero). Crashed: frozen in place (placeholder,
// TODO(phase 7, item 40): structural damage). Actuators and propellant keep integrating.
enum class Constraint { Free, HeldByArms, Crashed };

// 6DOF rigid body with three gimbaled sea-level engines and three fixed Raptor Vacuums,
// per-engine throttle and gimbal actuator dynamics, placeholder aerodynamics, and a simple ground contact.
// TODO(phase 1, item 7): variable-mass terms (Idot, CG motion, jet damping).
class HopperDynamics final : public ContinuousSystem {
public:
    // Build `initial` with stateOnPad() to start standing on the ground.
    HopperDynamics(const HopperParams& params, const HopperState& initial);

    void initialize(Bus& bus) override;
    void prepare(double t) override;
    void integrate(double t, double dt) override;
    bool halted() const override { return halted_; }

    // Truth, for sensor models and logging only. Flight software never sees this.
    const HopperState& state() const { return x_; }
    double mass() const { return params_.dryMass + x_.propellantMass; }
    bool engineRunning() const;  // any engine
    bool engineRunning(int i) const { return engineRunning_[i]; }
    // Commanded on but refused (RVac flow separation at high ambient pressure).
    bool engineInhibited(int i) const { return engineInhibited_[i]; }
    EngineState engineState(int i) const;
    // Mean over the three sea-level engines (the RVacs are not included).
    double meanThrottle() const { return x_.throttle.head<kSeaLevelEngineCount>().mean(); }
    Eigen::Vector2d meanGimbal() const { return x_.gimbal.rowwise().mean(); }
    Eigen::Vector3d accelerationWorld() const { return lastDeriv_.velocityDot; }
    Eigen::Vector3d specificForceBody() const;  // what an ideal accelerometer at the CG reads
    // A point given in the body frame relative to the base, in world coordinates.
    Eigen::Vector3d pointWorld(const Eigen::Vector3d& bodyFromBase) const;
    Eigen::Vector3d pointVelocityWorld(const Eigen::Vector3d& bodyFromBase) const;
    Eigen::Vector3d basePosition() const { return pointWorld(Eigen::Vector3d::Zero()); }
    double maxAltitude() const { return maxAltitude_; }  // of the base
    // Catch pins (two, at +/- catchPinOffset along body y, catchPinHeight above the base), pin 0 on +y.
    Eigen::Vector3d pinWorld(int i) const { return pointWorld(pinBody(i)); }
    Eigen::Vector3d pinVelocityWorld(int i) const { return pointVelocityWorld(pinBody(i)); }
    // Constrain the vehicle: shift the pose by `cgShift` (world [m]), zero velocity and angular rate,
    // and from then on integrate only actuators and propellant. Called by the tower model.
    void hold(Constraint constraint, const Eigen::Vector3d& cgShift);
    // Teleport to `state` (scenario setup and tests only, never from the step loop).
    void setState(const HopperState& state) {
        x_ = state;
        onGround_ = baseHeightOf(x_) <= 1e-9;
    }
    Constraint constraint() const { return constraint_; }
    // Constrained and no engine running for `enginesOff` [s], or constrained for `afterHold` [s]: halt.
    void setHoldHaltTimes(double enginesOff, double afterHold) {
        haltEnginesOff_ = enginesOff;
        haltAfterHold_ = afterHold;
    }
    const TouchdownReport& touchdown() const { return touchdown_; }
    const HopperParams& params() const { return params_; }
    const HopperMassProperties& massProperties() const { return massProperties_; }

    // Initial state with the base at `baseEnu` (on the ground) and the given attitude.
    static HopperState stateOnPad(const HopperParams& params, const Eigen::Vector3d& baseEnu,
                                  const Eigen::Quaterniond& attitude);

private:
    HopperDeriv derivatives(const HopperState& x, const ActuatorCommandMsg& u) const;
    Eigen::Matrix3d inertia(double propellantMass) const;
    Eigen::Vector3d pinBody(int i) const {
        return Eigen::Vector3d(0.0, i == 0 ? params_.catchPinOffset : -params_.catchPinOffset, params_.catchPinHeight);
    }
    double baseHeightOf(const HopperState& x) const;
    void applyGroundContact(double t);

    HopperParams params_;
    HopperMassProperties massProperties_;
    Eigen::Matrix<double, 3, kEngineCount> enginePositions_;  // thrust application points from the CG, body [m]
    // Per-engine constants (structure of arrays), filled from the params.
    double ratedVacuumThrust_[kEngineCount] = {};  // [N]
    double exitArea_[kEngineCount] = {};           // [m^2]
    double vacuumIsp_[kEngineCount] = {};          // [s]
    double minThrottle_[kEngineCount] = {};        // fraction
    double ignitionDelay_[kEngineCount] = {};      // [s]
    double throttleTau_[kEngineCount] = {};        // [s]
    double throttleRate_[kEngineCount] = {};       // [1/s]
    double maxAmbientPressure_[kEngineCount] = {}; // [Pa] start inhibit above this
    HopperState x_;
    HopperDeriv lastDeriv_;
    env::StandardAtmosphere1976 atmosphere_;
    env::UniformGravity gravity_;
    env::WeatherSample weather_;

    const Channel<ActuatorCommandMsg>* commandChannel_ = nullptr;
    const Channel<env::WeatherSample>* weatherChannel_ = nullptr;
    Channel<env::WeatherConditions>* weatherConditionsChannel_ = nullptr;
    ActuatorCommandMsg command_;

    bool engineRunning_[kEngineCount] = {};
    bool engineInhibited_[kEngineCount] = {};
    bool ignitionPending_[kEngineCount] = {};
    double ignitionAt_[kEngineCount] = {};

    bool onGround_ = true;
    bool hasFlown_ = false;
    bool halted_ = false;
    Constraint constraint_ = Constraint::Free;
    double heldTime_ = 0.0;      // [s] since hold()
    double noEngineTime_ = 0.0;  // [s] constrained with no engine running
    double haltEnginesOff_ = 2.0;  // [s]
    double haltAfterHold_ = 20.0;  // [s]
    double maxAltitude_ = 0.0;
    TouchdownReport touchdown_;
};

}  // namespace sim::vehicle
