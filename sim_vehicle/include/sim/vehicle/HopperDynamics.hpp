#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sim/core/Bus.hpp"
#include "sim/core/ContinuousSystem.hpp"
#include "sim/env/Atmosphere.hpp"
#include "sim/env/Gravity.hpp"
#include "sim/vehicle/HopperParams.hpp"
#include "sim/vehicle/Messages.hpp"

namespace sim::vehicle {

// Frames: local ENU world frame (z up), body frame with +z along the vehicle
// axis toward the nose. q rotates body vectors into the world frame.
struct HopperState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();         // world [m]
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();         // world [m/s]
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();  // body -> world
    Eigen::Vector3d angularRate = Eigen::Vector3d::Zero();      // body [rad/s]
    double propellantMass = 0.0;                               // [kg]
    double throttle = 0.0;                                     // actual, fraction
    Eigen::Vector2d gimbal = Eigen::Vector2d::Zero();           // actual pitch, yaw [rad]
};

struct HopperDeriv {
    Eigen::Vector3d positionDot = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocityDot = Eigen::Vector3d::Zero();
    Eigen::Vector4d attitudeDot = Eigen::Vector4d::Zero();  // quaternion coeffs (x, y, z, w)
    Eigen::Vector3d angularRateDot = Eigen::Vector3d::Zero();
    double propellantMassDot = 0.0;
    double throttleDot = 0.0;
    Eigen::Vector2d gimbalDot = Eigen::Vector2d::Zero();
};

HopperDeriv operator+(const HopperDeriv& a, const HopperDeriv& b);
HopperDeriv operator*(double s, const HopperDeriv& d);
HopperState addScaled(const HopperState& x, const HopperDeriv& dx, double h);

struct TouchdownReport {
    bool occurred = false;
    double time = 0.0;
    double verticalSpeed = 0.0;    // [m/s], positive down
    double horizontalSpeed = 0.0;  // [m/s]
};

// 6DOF rigid body with a single gimballed engine, throttle and gimbal actuator
// dynamics, placeholder drag, and a simple ground contact.
// TODO(phase 1, item 7): variable-mass terms (Idot, CG motion, jet damping).
class HopperDynamics final : public ContinuousSystem {
public:
    HopperDynamics(const HopperParams& params, const HopperState& initial);

    void initialize(Bus& bus) override;
    void prepare(double t) override;
    void integrate(double t, double dt) override;
    bool halted() const override { return halted_; }

    // Truth, for sensor models and logging only. Flight software never sees this.
    const HopperState& state() const { return x_; }
    double mass() const { return params_.dryMass + x_.propellantMass; }
    bool engineRunning() const { return engineRunning_; }
    Eigen::Vector3d accelerationWorld() const { return lastDeriv_.velocityDot; }
    Eigen::Vector3d specificForceBody() const;  // what an ideal accelerometer at the CG reads
    double maxAltitude() const { return maxAltitude_; }
    const TouchdownReport& touchdown() const { return touchdown_; }
    const HopperParams& params() const { return params_; }

private:
    HopperDeriv derivatives(const HopperState& x, const ActuatorCommandMsg& u) const;
    Eigen::Matrix3d inertia(double mass) const;
    void applyGroundContact(double t);

    HopperParams params_;
    HopperState x_;
    HopperDeriv lastDeriv_;
    env::StandardAtmosphere1976 atmosphere_;
    env::UniformGravity gravity_;

    const Channel<ActuatorCommandMsg>* commandChannel_ = nullptr;
    ActuatorCommandMsg command_;

    bool engineRunning_ = false;
    bool ignitionPending_ = false;
    double ignitionAt_ = 0.0;

    bool onGround_ = true;
    bool hasFlown_ = false;
    bool halted_ = false;
    double maxAltitude_ = 0.0;
    TouchdownReport touchdown_;
};

}  // namespace sim::vehicle
