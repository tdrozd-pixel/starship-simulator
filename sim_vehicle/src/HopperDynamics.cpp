#include "sim/vehicle/HopperDynamics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "sim/core/Integrator.hpp"
#include "sim/core/Profiling.hpp"

namespace sim::vehicle {

HopperDeriv operator+(const HopperDeriv& a, const HopperDeriv& b) {
    HopperDeriv r;
    r.positionDot = a.positionDot + b.positionDot;
    r.velocityDot = a.velocityDot + b.velocityDot;
    r.attitudeDot = a.attitudeDot + b.attitudeDot;
    r.angularRateDot = a.angularRateDot + b.angularRateDot;
    r.propellantMassDot = a.propellantMassDot + b.propellantMassDot;
    r.throttleDot = a.throttleDot + b.throttleDot;
    r.gimbalDot = a.gimbalDot + b.gimbalDot;
    return r;
}

HopperDeriv operator*(double s, const HopperDeriv& d) {
    HopperDeriv r;
    r.positionDot = s * d.positionDot;
    r.velocityDot = s * d.velocityDot;
    r.attitudeDot = s * d.attitudeDot;
    r.angularRateDot = s * d.angularRateDot;
    r.propellantMassDot = s * d.propellantMassDot;
    r.throttleDot = s * d.throttleDot;
    r.gimbalDot = s * d.gimbalDot;
    return r;
}

HopperState addScaled(const HopperState& x, const HopperDeriv& dx, double h) {
    HopperState r;
    r.position = x.position + h * dx.positionDot;
    r.velocity = x.velocity + h * dx.velocityDot;
    r.attitude.coeffs() = x.attitude.coeffs() + h * dx.attitudeDot;
    r.attitude.normalize();
    r.angularRate = x.angularRate + h * dx.angularRateDot;
    r.propellantMass = x.propellantMass + h * dx.propellantMassDot;
    r.throttle = x.throttle + h * dx.throttleDot;
    r.gimbal = x.gimbal + h * dx.gimbalDot;
    return r;
}

namespace {

double firstOrderRate(double current, double target, double timeConstant, double rateLimit) {
    const double rate = (target - current) / timeConstant;
    return std::clamp(rate, -rateLimit, rateLimit);
}

}  // namespace

HopperDynamics::HopperDynamics(const HopperParams& params, const HopperState& initial)
    : params_(params), x_(initial) {
    onGround_ = x_.position.z() <= 0.0;
}

void HopperDynamics::initialize(Bus& bus) {
    commandChannel_ = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
}

Eigen::Matrix3d HopperDynamics::inertia(double mass) const {
    // Placeholder: solid cylinder scaled by current mass.
    // TODO(phase 1, item 8): build from component masses and live tank fill levels.
    const double r2 = params_.radius * params_.radius;
    const double l2 = params_.length * params_.length;
    const double transverse = mass * (3.0 * r2 + l2) / 12.0;
    const double axial = 0.5 * mass * r2;
    return Eigen::Vector3d(transverse, transverse, axial).asDiagonal();
}

HopperDeriv HopperDynamics::derivatives(const HopperState& x, const ActuatorCommandMsg& u) const {
    HopperDeriv d;
    const double mass = params_.dryMass + std::max(0.0, x.propellantMass);
    const Eigen::Matrix3d R = x.attitude.toRotationMatrix();

    // Thrust (body frame), engine at the gimbal point below the CG
    const bool thrusting = engineRunning_ && x.propellantMass > 0.0;
    const double thrust = thrusting ? std::max(0.0, x.throttle) * params_.maxThrust : 0.0;
    const double pitch = x.gimbal.x();
    const double yaw = x.gimbal.y();
    const Eigen::Vector3d thrustDir(std::cos(pitch) * std::sin(yaw), -std::sin(pitch), std::cos(pitch) * std::cos(yaw));
    const Eigen::Vector3d thrustBody = thrust * thrustDir;
    const Eigen::Vector3d engineArm(0.0, 0.0, -params_.engineLeverArm);
    Eigen::Vector3d torqueBody = engineArm.cross(thrustBody);

    // Drag (placeholder): opposes velocity, constant Cd on the cross-section
    const env::AtmosphereSample air = atmosphere_.at(std::max(0.0, x.position.z()));
    const double area = std::numbers::pi * params_.radius * params_.radius;
    const double speed = x.velocity.norm();
    const Eigen::Vector3d drag = -0.5 * air.density * params_.dragCoefficient * area * speed * x.velocity;

    Eigen::Vector3d forceWorld = R * thrustBody + mass * gravity_.acceleration(x.position) + drag;

    // Ground contact: the pad pushes back and holds the vehicle still.
    // TODO(phase 7): real leg contact as spring-dampers.
    if (onGround_) {
        forceWorld.x() = 0.0;
        forceWorld.y() = 0.0;
        forceWorld.z() = std::max(0.0, forceWorld.z());
        torqueBody.setZero();
    }

    d.positionDot = x.velocity;
    d.velocityDot = forceWorld / mass;

    const Eigen::Matrix3d I = inertia(mass);
    const Eigen::Vector3d& w = x.angularRate;
    d.angularRateDot = I.ldlt().solve(torqueBody - w.cross(I * w));

    const Eigen::Quaterniond omega(0.0, w.x(), w.y(), w.z());
    d.attitudeDot = 0.5 * (x.attitude * omega).coeffs();

    d.propellantMassDot = thrusting ? -thrust / (params_.ispSeaLevel * env::kStandardGravity) : 0.0;

    // Actuators: first-order response with rate limits
    const double throttleTarget = engineRunning_ ? std::clamp(u.throttle, params_.minThrottle, 1.0) : 0.0;
    d.throttleDot = firstOrderRate(x.throttle, throttleTarget, params_.throttleTimeConstant, params_.throttleRateLimit);

    const Eigen::Vector2d gimbalTarget(std::clamp(u.gimbalPitch, -params_.gimbalLimit, params_.gimbalLimit),
                                       std::clamp(u.gimbalYaw, -params_.gimbalLimit, params_.gimbalLimit));
    for (int i = 0; i < 2; ++i) {
        d.gimbalDot[i] = firstOrderRate(x.gimbal[i], gimbalTarget[i], params_.gimbalTimeConstant, params_.gimbalRateLimit);
    }
    return d;
}

void HopperDynamics::prepare(double t) {
    SIM_ZONE("HopperDynamics::prepare");
    if (commandChannel_ != nullptr && commandChannel_->latest().valid()) {
        command_ = commandChannel_->latest().data;
    }

    // Engine start/stop sequencing (placeholder for the full Raptor start sequence)
    if (!command_.engineOn || x_.propellantMass <= 0.0) {
        engineRunning_ = false;
        ignitionPending_ = false;
    } else if (!engineRunning_ && !ignitionPending_) {
        ignitionPending_ = true;
        ignitionAt_ = t + params_.ignitionDelay;
    }
    if (ignitionPending_ && t >= ignitionAt_) {
        ignitionPending_ = false;
        engineRunning_ = true;
    }

    lastDeriv_ = derivatives(x_, command_);
}

Eigen::Vector3d HopperDynamics::specificForceBody() const {
    const Eigen::Vector3d g = gravity_.acceleration(x_.position);
    return x_.attitude.conjugate() * (lastDeriv_.velocityDot - g);
}

void HopperDynamics::integrate(double t, double dt) {
    SIM_ZONE("HopperDynamics::integrate");
    // Pick up any command the flight software published this step.
    if (commandChannel_ != nullptr && commandChannel_->latest().valid()) {
        command_ = commandChannel_->latest().data;
    }

    x_ = rk4Step(x_, t, dt, [this](const HopperState& s, double) { return derivatives(s, command_); });
    x_.propellantMass = std::max(0.0, x_.propellantMass);
    x_.throttle = std::max(0.0, x_.throttle);

    applyGroundContact(t + dt);
    maxAltitude_ = std::max(maxAltitude_, x_.position.z());

    if (touchdown_.occurred && !engineRunning_ && (t + dt) - touchdown_.time > 2.0) {
        halted_ = true;
    }
}

void HopperDynamics::applyGroundContact(double t) {
    if (x_.position.z() > 0.01) {
        onGround_ = false;
        if (x_.position.z() > 0.5) {
            hasFlown_ = true;
        }
        return;
    }
    if (x_.position.z() <= 0.0) {
        if (!onGround_ && hasFlown_ && !touchdown_.occurred) {
            touchdown_.occurred = true;
            touchdown_.time = t;
            touchdown_.verticalSpeed = -x_.velocity.z();
            touchdown_.horizontalSpeed = x_.velocity.head<2>().norm();
        }
        onGround_ = true;
        x_.position.z() = 0.0;
        x_.velocity.setZero();
        x_.angularRate.setZero();
    }
}

}  // namespace sim::vehicle
