#include "sim/vehicle/HopperDynamics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

HopperMassProperties hopperMassProperties(const HopperParams& p) {
    HopperMassProperties m;
    m.cgHeight = p.cgHeight();
    const double r2 = p.radius * p.radius;

    // Dry: thin cylindrical shell of the full length, centered at the dry CG.
    const double dryOffset = p.dryCgHeight - m.cgHeight;
    m.transverseInertiaDry =
        p.dryMass * (0.5 * r2 + p.length * p.length / 12.0) + p.dryMass * dryOffset * dryOffset;
    m.axialInertiaDry = p.dryMass * r2;

    // Propellant: solid column (rotates with the tank wall; no slosh yet, item 13).
    const double propOffset = p.propellantCgHeight - m.cgHeight;
    const double lp = p.propellantColumnLength;
    m.transverseInertiaPropellant =
        p.initialPropellant * (3.0 * r2 + lp * lp) / 12.0 + p.initialPropellant * propOffset * propOffset;
    m.axialInertiaPropellant = 0.5 * p.initialPropellant * r2;
    return m;
}

HopperState HopperDynamics::stateOnPad(const HopperParams& params, const Eigen::Vector3d& baseEnu,
                                       const Eigen::Quaterniond& attitude) {
    HopperState s;
    s.attitude = attitude;
    s.propellantMass = params.initialPropellant;
    s.position = baseEnu + attitude * Eigen::Vector3d(0.0, 0.0, params.cgHeight());
    return s;
}

HopperDynamics::HopperDynamics(const HopperParams& params, const HopperState& initial)
    : params_(params), massProperties_(hopperMassProperties(params)), x_(initial) {
    // The RVacs are fixed; their thrust acts in the same plane as the gimbal points (estimate).
    const double gimbalZ = params_.gimbalHeight - massProperties_.cgHeight;
    for (int i = 0; i < kEngineCount; ++i) {
        const bool sea = i < kSeaLevelEngineCount;
        const double azimuth = sea ? params_.engineAzimuth0 + 2.0 * std::numbers::pi * i / kSeaLevelEngineCount
                                   : params_.rvacAzimuth0 +
                                         2.0 * std::numbers::pi * (i - kFirstRvac) / kRvacCount;
        const double radius = sea ? params_.engineClusterRadius : params_.rvacClusterRadius;
        enginePositions_.col(i) = Eigen::Vector3d(radius * std::cos(azimuth), radius * std::sin(azimuth), gimbalZ);
        ratedVacuumThrust_[i] = sea ? params_.vacuumThrust() : params_.rvacVacuumThrust;
        exitArea_[i] = sea ? params_.nozzleExitArea() : params_.rvacNozzleExitArea();
        vacuumIsp_[i] = sea ? params_.vacuumIsp : params_.rvacVacuumIsp;
        minThrottle_[i] = sea ? params_.minThrottle : params_.rvacMinThrottle;
        ignitionDelay_[i] = sea ? params_.ignitionDelay : params_.rvacIgnitionDelay;
        throttleTau_[i] = sea ? params_.throttleTimeConstant : params_.rvacThrottleTimeConstant;
        throttleRate_[i] = sea ? params_.throttleRateLimit : params_.rvacThrottleRateLimit;
        maxAmbientPressure_[i] = sea ? std::numeric_limits<double>::infinity() : params_.rvacMaxAmbientPressure;
    }
    onGround_ = baseHeightOf(x_) <= 1e-9;
    maxAltitude_ = std::max(0.0, baseHeightOf(x_));
}

void HopperDynamics::initialize(Bus& bus) {
    commandChannel_ = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
    weatherChannel_ = &bus.channel<env::WeatherSample>("weather");
    weatherConditionsChannel_ = &bus.channel<env::WeatherConditions>("weather_conditions");
}

bool HopperDynamics::engineRunning() const {
    for (bool running : engineRunning_) {
        if (running) {
            return true;
        }
    }
    return false;
}

EngineState HopperDynamics::engineState(int i) const {
    if (engineRunning_[i]) {
        return EngineState::Running;
    }
    if (ignitionPending_[i]) {
        return EngineState::Starting;
    }
    return engineInhibited_[i] ? EngineState::Inhibited : EngineState::Off;
}

double HopperDynamics::baseHeightOf(const HopperState& x) const {
    return (x.position + x.attitude * Eigen::Vector3d(0.0, 0.0, -massProperties_.cgHeight)).z();
}

Eigen::Vector3d HopperDynamics::pointWorld(const Eigen::Vector3d& bodyFromBase) const {
    const Eigen::Vector3d fromCg = bodyFromBase - Eigen::Vector3d(0.0, 0.0, massProperties_.cgHeight);
    return x_.position + x_.attitude * fromCg;
}

Eigen::Vector3d HopperDynamics::pointVelocityWorld(const Eigen::Vector3d& bodyFromBase) const {
    const Eigen::Vector3d fromCg = bodyFromBase - Eigen::Vector3d(0.0, 0.0, massProperties_.cgHeight);
    return x_.velocity + x_.attitude * x_.angularRate.cross(fromCg);
}

Eigen::Matrix3d HopperDynamics::inertia(double propellantMass) const {
    const double fraction =
        params_.initialPropellant > 0.0 ? std::max(0.0, propellantMass) / params_.initialPropellant : 0.0;
    const double transverse =
        massProperties_.transverseInertiaDry + fraction * massProperties_.transverseInertiaPropellant;
    const double axial = massProperties_.axialInertiaDry + fraction * massProperties_.axialInertiaPropellant;
    return Eigen::Vector3d(transverse, transverse, axial).asDiagonal();
}

HopperDeriv HopperDynamics::derivatives(const HopperState& x, const ActuatorCommandMsg& u) const {
    HopperDeriv d;
    const double mass = params_.dryMass + std::max(0.0, x.propellantMass);
    const Eigen::Matrix3d R = x.attitude.toRotationMatrix();
    const env::AtmosphereSample air = atmosphere_.at(std::max(0.0, baseHeightOf(x)));

    // Thrust: per engine F = throttle * F_vac - p_amb * A_exit, applied at the gimbal point.
    const bool havePropellant = x.propellantMass > 0.0;
    Eigen::Vector3d forceBody = Eigen::Vector3d::Zero();
    Eigen::Vector3d torqueBody = Eigen::Vector3d::Zero();
    double massFlow = 0.0;
    for (int i = 0; i < kEngineCount; ++i) {
        if (!engineRunning_[i] || !havePropellant) {
            continue;
        }
        const double throttle = std::max(0.0, x.throttle[i]);
        const double thrust = std::max(0.0, throttle * ratedVacuumThrust_[i] - air.pressure * exitArea_[i]);
        Eigen::Vector3d dir(0.0, 0.0, 1.0);  // RVacs are fixed
        if (i < kSeaLevelEngineCount) {
            const double pitch = x.gimbal(0, i);
            const double yaw = x.gimbal(1, i);
            dir = Eigen::Vector3d(std::cos(pitch) * std::sin(yaw), -std::sin(pitch), std::cos(pitch) * std::cos(yaw));
        }
        const Eigen::Vector3d f = thrust * dir;
        forceBody += f;
        torqueBody += enginePositions_.col(i).cross(f);
        massFlow += throttle * ratedVacuumThrust_[i] / (vacuumIsp_[i] * env::kStandardGravity);
    }

    // Aerodynamics (placeholder for item 17): axial drag on the base area plus
    // cross-flow drag on the side area, acting at the center of pressure.
    const Eigen::Vector3d airVelocityBody = R.transpose() * (x.velocity - weather_.wind);
    const double axialSpeed = airVelocityBody.z();
    const Eigen::Vector3d crossflow(airVelocityBody.x(), airVelocityBody.y(), 0.0);
    const double baseArea = std::numbers::pi * params_.radius * params_.radius;
    const double sideArea = 2.0 * params_.radius * params_.length;
    const Eigen::Vector3d aeroBody =
        Eigen::Vector3d(0.0, 0.0, -0.5 * air.density * params_.axialDragCoefficient * baseArea * std::abs(axialSpeed) * axialSpeed) -
        0.5 * air.density * params_.crossflowDragCoefficient * sideArea * crossflow.norm() * crossflow;
    const Eigen::Vector3d cpFromCg(0.0, 0.0, params_.centerOfPressureHeight - massProperties_.cgHeight);
    forceBody += aeroBody;
    torqueBody += cpFromCg.cross(aeroBody);

    Eigen::Vector3d forceWorld = R * forceBody + mass * gravity_.acceleration(x.position);

    // Ground contact: the pad pushes back and holds the vehicle still.
    // TODO(phase 7, item 40): real contact (legs or catch arms) as spring-dampers.
    if (onGround_) {
        forceWorld.x() = 0.0;
        forceWorld.y() = 0.0;
        forceWorld.z() = std::max(0.0, forceWorld.z());
        torqueBody.setZero();
    }

    d.positionDot = x.velocity;
    d.velocityDot = forceWorld / mass;

    const Eigen::Matrix3d I = inertia(x.propellantMass);
    const Eigen::Vector3d& w = x.angularRate;
    d.angularRateDot = I.ldlt().solve(torqueBody - w.cross(I * w));

    const Eigen::Quaterniond omega(0.0, w.x(), w.y(), w.z());
    d.attitudeDot = 0.5 * (x.attitude * omega).coeffs();

    d.propellantMassDot = -massFlow;

    // Actuators: first-order response with rate limits, per engine.
    for (int i = 0; i < kEngineCount; ++i) {
        const EngineCommandMsg& c = u.engines[i];
        const double throttleTarget = engineRunning_[i] ? std::clamp(c.throttle, minThrottle_[i], 1.0) : 0.0;
        d.throttleDot[i] = firstOrderRate(x.throttle[i], throttleTarget, throttleTau_[i], throttleRate_[i]);
    }
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        const GimbalCommandMsg& g = u.gimbals[i];
        const double targets[2] = {std::clamp(g.pitch, -params_.gimbalLimit, params_.gimbalLimit),
                                   std::clamp(g.yaw, -params_.gimbalLimit, params_.gimbalLimit)};
        for (int axis = 0; axis < 2; ++axis) {
            d.gimbalDot(axis, i) = firstOrderRate(x.gimbal(axis, i), targets[axis], params_.gimbalTimeConstant,
                                                  params_.gimbalRateLimit);
        }
    }
    if (constraint_ != Constraint::Free) {
        // Held by the arms or crashed: the rigid body does not move.
        d.positionDot.setZero();
        d.velocityDot.setZero();
        d.attitudeDot.setZero();
        d.angularRateDot.setZero();
    }
    return d;
}

void HopperDynamics::prepare(double t) {
    SIM_ZONE("HopperDynamics::prepare");
    const Eigen::Vector3d ambientWind =
        (weatherChannel_ != nullptr && weatherChannel_->latest().valid())
            ? weatherChannel_->latest().data.wind
            : Eigen::Vector3d::Zero();
    weatherConditionsChannel_->publish(
        env::WeatherConditions{std::max(0.0, baseHeightOf(x_)), (x_.velocity - ambientWind).norm()}, t);

    if (commandChannel_ != nullptr && commandChannel_->latest().valid()) {
        command_ = commandChannel_->latest().data;
    }

    // Engine start/stop sequencing (placeholder for the Raptor start sequence, item 9).
    // Start inhibit: an engine whose ambient pressure limit is exceeded (RVac flow
    // separation) refuses to run: it stays off (or shuts down) and reports Inhibited.
    const double ambientPressure = atmosphere_.at(std::max(0.0, baseHeightOf(x_))).pressure;
    for (int i = 0; i < kEngineCount; ++i) {
        engineInhibited_[i] = command_.engines[i].on && ambientPressure > maxAmbientPressure_[i];
        if (!command_.engines[i].on || x_.propellantMass <= 0.0 || engineInhibited_[i]) {
            engineRunning_[i] = false;
            ignitionPending_[i] = false;
        } else if (!engineRunning_[i] && !ignitionPending_[i]) {
            ignitionPending_[i] = true;
            ignitionAt_[i] = t + ignitionDelay_[i];
        }
        if (ignitionPending_[i] && t >= ignitionAt_[i]) {
            ignitionPending_[i] = false;
            engineRunning_[i] = true;
        }
    }

    lastDeriv_ = derivatives(x_, command_);
}

Eigen::Vector3d HopperDynamics::specificForceBody() const {
    const Eigen::Vector3d g = gravity_.acceleration(x_.position);
    return x_.attitude.conjugate() * (lastDeriv_.velocityDot - g);
}

void HopperDynamics::integrate(double t, double dt) {
    SIM_ZONE("HopperDynamics::integrate");
    if (weatherChannel_ != nullptr && weatherChannel_->latest().valid()) {
        weather_ = weatherChannel_->latest().data;
    }

    // Pick up any command the flight software published this step.
    if (commandChannel_ != nullptr && commandChannel_->latest().valid()) {
        command_ = commandChannel_->latest().data;
    }

    const HopperState before = x_;
    x_ = rk4Step(x_, t, dt, [this](const HopperState& s, double) { return derivatives(s, command_); });
    if (constraint_ != Constraint::Free) {
        // Keep the pose bit-exact (the quaternion renormalization would otherwise drift it by an ulp).
        x_.position = before.position;
        x_.velocity = before.velocity;
        x_.attitude = before.attitude;
        x_.angularRate = before.angularRate;
    }
    x_.propellantMass = std::max(0.0, x_.propellantMass);
    x_.throttle = x_.throttle.cwiseMax(0.0);

    applyGroundContact(t + dt);
    maxAltitude_ = std::max(maxAltitude_, baseHeightOf(x_));

    if (touchdown_.occurred && !engineRunning() && (t + dt) - touchdown_.time > 2.0) {
        halted_ = true;
    }
    if (constraint_ != Constraint::Free) {
        heldTime_ += dt;
        noEngineTime_ = engineRunning() ? 0.0 : noEngineTime_ + dt;
        if (noEngineTime_ >= haltEnginesOff_ || heldTime_ >= haltAfterHold_) {
            halted_ = true;
        }
    }
}

void HopperDynamics::hold(Constraint constraint, const Eigen::Vector3d& cgShift) {
    if (constraint_ == Constraint::Free && constraint != Constraint::Free) {
        heldTime_ = 0.0;
        noEngineTime_ = 0.0;
    }
    constraint_ = constraint;
    x_.position += cgShift;
    x_.velocity.setZero();
    x_.angularRate.setZero();
}

void HopperDynamics::applyGroundContact(double t) {
    const double baseHeight = baseHeightOf(x_);
    if (baseHeight > 0.01) {
        onGround_ = false;
        if (baseHeight > 0.5) {
            hasFlown_ = true;
        }
        return;
    }
    if (baseHeight <= 0.0) {
        if (!onGround_ && hasFlown_ && !touchdown_.occurred) {
            const Eigen::Vector3d base = basePosition();
            touchdown_.occurred = true;
            touchdown_.time = t;
            touchdown_.verticalSpeed = -x_.velocity.z();
            touchdown_.horizontalSpeed = x_.velocity.head<2>().norm();
            touchdown_.east = base.x();
            touchdown_.north = base.y();
        }
        onGround_ = true;
        x_.position.z() -= baseHeight;
        x_.velocity.setZero();
        x_.angularRate.setZero();
    }
}

}  // namespace sim::vehicle
