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

using EngineVector = Eigen::Matrix<double, kEngineCount, 1>;
using EngineGimbals = Eigen::Matrix<double, 2, kEngineCount>;  // row 0 pitch, row 1 yaw [rad]

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
    EngineVector throttle = EngineVector::Zero();              // actual, per engine
    EngineGimbals gimbal = EngineGimbals::Zero();              // actual, per engine [rad]
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

// 6DOF rigid body with three gimbaled engines, per-engine throttle and gimbal
// actuator dynamics, placeholder aerodynamics, and a simple ground contact.
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
    double meanThrottle() const { return x_.throttle.mean(); }
    Eigen::Vector2d meanGimbal() const { return x_.gimbal.rowwise().mean(); }
    Eigen::Vector3d accelerationWorld() const { return lastDeriv_.velocityDot; }
    Eigen::Vector3d specificForceBody() const;  // what an ideal accelerometer at the CG reads
    // A point given in the body frame relative to the base, in world coordinates.
    Eigen::Vector3d pointWorld(const Eigen::Vector3d& bodyFromBase) const;
    Eigen::Vector3d pointVelocityWorld(const Eigen::Vector3d& bodyFromBase) const;
    Eigen::Vector3d basePosition() const { return pointWorld(Eigen::Vector3d::Zero()); }
    double maxAltitude() const { return maxAltitude_; }  // of the base
    const TouchdownReport& touchdown() const { return touchdown_; }
    const HopperParams& params() const { return params_; }
    const HopperMassProperties& massProperties() const { return massProperties_; }

    // Initial state with the base at `baseEnu` (on the ground) and the given attitude.
    static HopperState stateOnPad(const HopperParams& params, const Eigen::Vector3d& baseEnu,
                                  const Eigen::Quaterniond& attitude);

private:
    HopperDeriv derivatives(const HopperState& x, const ActuatorCommandMsg& u) const;
    Eigen::Matrix3d inertia(double propellantMass) const;
    double baseHeightOf(const HopperState& x) const;
    void applyGroundContact(double t);

    HopperParams params_;
    HopperMassProperties massProperties_;
    Eigen::Matrix<double, 3, kEngineCount> enginePositions_;  // gimbal points from the CG, body [m]
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
    bool ignitionPending_[kEngineCount] = {};
    double ignitionAt_[kEngineCount] = {};

    bool onGround_ = true;
    bool hasFlown_ = false;
    bool halted_ = false;
    double maxAltitude_ = 0.0;
    TouchdownReport touchdown_;
};

}  // namespace sim::vehicle
