#include "sim/vehicle/Tower.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace sim::vehicle {

PinContact classifyPin(const TowerParams& tower, double pinRadius, double along, double across, double z) {
    // The pin center must be inside the rail slab (arm thickness below the top, one pin radius above).
    if (z < tower.railHeight - tower.armThickness || z > tower.railHeight + pinRadius) {
        return PinContact::None;
    }
    if (std::abs(along - tower.cradleDistance) <= tower.cradleHalfLength && std::abs(across) <= tower.cradleHalfWidth) {
        return PinContact::Cradle;
    }
    if (std::abs(across) <= tower.armHalfWidth + pinRadius && along >= -pinRadius &&
        along <= tower.armLength + pinRadius) {
        return PinContact::ArmBody;
    }
    return PinContact::None;
}

CatchResult evaluateContact(const TowerParams& tower, double pinRadius, const ContactInput& in) {
    bool cradle = false;
    for (int i = 0; i < 2; ++i) {
        const PinContact c = classifyPin(tower, pinRadius, in.along[i], in.across[i], in.z[i]);
        if (c == PinContact::ArmBody) {
            return CatchResult::ArmStrike;
        }
        cradle = cradle || c == PinContact::Cradle;
    }
    if (!cradle) {
        return CatchResult::None;
    }
    // First contact in a cradle: both pins must be inside the catch window.
    bool ok = in.closure >= tower.windowClosure && in.sinkRate <= tower.windowSink &&
              in.horizontalSpeed <= tower.windowHorizontalSpeed && in.tilt <= tower.windowTilt &&
              in.headingError <= tower.windowHeading && in.angularRate <= tower.windowAngularRate;
    for (int i = 0; i < 2; ++i) {
        ok = ok && std::abs(in.across[i]) <= tower.windowLateral &&
             std::abs(in.along[i] - tower.cradleDistance) <= tower.windowAlong;
    }
    return ok ? CatchResult::Caught : CatchResult::ArmStrike;
}

bool shouldCloseArms(const TowerParams& tower, double horizontal, double height, double sink) {
    if (horizontal > tower.closeTriggerRadius || height < tower.closeTriggerMinHeight ||
        height > tower.closeTriggerMaxHeight || sink < tower.closeTriggerMinSink) {
        return false;
    }
    const bool low = height <= tower.closeTriggerHeight;
    const bool predicted = height > 0.0 && 2.0 * height / sink <= tower.closeTriggerLeadTime;
    return low || predicted;
}

bool hullTouchesArm(const TowerParams& tower, double hullRadius, double psi, double side, double along, double lateral) {
    const double phi = tower.phi(psi);
    const double c = std::cos(phi);
    const double s = std::sin(phi);
    const double relAlong = along - tower.hingeSetback;
    const double relLateral = lateral - side * tower.hingeHalfSpacing;
    // Arm-local coordinates (same transform as the pin contact test).
    const double armAlong = relAlong * c - side * relLateral * s;
    const double armAcross = relAlong * side * s + relLateral * c;
    const double dx = std::max({-armAlong, 0.0, armAlong - tower.armLength});
    const double dy = std::max(std::abs(armAcross) - tower.armHalfWidth, 0.0);
    return dx * dx + dy * dy <= hullRadius * hullRadius;
}

TowerModel::TowerModel(std::string name, double rateHz, HopperDynamics& vehicle, const TowerParams& tower)
    : Model(std::move(name), rateHz), vehicle_(vehicle), tower_(tower) {
    const HopperParams& v = vehicle.params();
    pinRadius_ = v.catchPinRadius;
    pinOffset_ = v.catchPinOffset;
    length_ = v.length;
    hullRadius_ = v.radius;
    psiClosed_ = tower_.psiClosed(pinOffset_);
    seat_ = tower_.seatPointEnu(v);
    vehicle_.setHoldHaltTimes(tower_.haltAfterEnginesOff, tower_.haltAfterHold);
}

void TowerModel::update(const StepContext& ctx) {
    stepServo(ctx.dt);
    if (vehicle_.constraint() != Constraint::Free) {
        return;
    }
    if (result_ == CatchResult::None) {
        updateStateMachine(ctx.dt);
        checkContacts();
    }
    if (result_ == CatchResult::None) {
        checkMast();
    }
    if (result_ == CatchResult::None) {
        checkHullArms();
    }
}

void TowerModel::stepServo(double dt) {
    const bool closing =
        state_ == TowerState::Closing || state_ == TowerState::Closed || state_ == TowerState::Holding;
    const double command = closing ? psiClosed_ : 0.0;
    for (double& psi : psi_) {
        const double rate = std::clamp(tower_.servoGain * (command - psi), -tower_.servoRateLimit, tower_.servoRateLimit);
        psi += rate * dt;
    }
}

bool TowerModel::outsideZone() const {
    const Eigen::Vector3d mid = 0.5 * (vehicle_.pinWorld(0) + vehicle_.pinWorld(1));
    const double horizontal = std::hypot(mid.x() - seat_.east, mid.y() - seat_.north);
    return horizontal > tower_.zoneRadius || mid.z() - seat_.pinHeight > tower_.zoneHeightAbove || seat_.pinHeight - mid.z() > tower_.zoneHeightBelow;
}

void TowerModel::updateStateMachine(double dt) {
    stateTime_ += dt;
    const bool outside = outsideZone();
    outsideTime_ = outside ? outsideTime_ + dt : 0.0;
    const auto change = [this](TowerState next) {
        state_ = next;
        stateTime_ = 0.0;
        outsideTime_ = 0.0;
    };
    switch (state_) {
        case TowerState::Open: {
            const Eigen::Vector3d mid = 0.5 * (vehicle_.pinWorld(0) + vehicle_.pinWorld(1));
            const Eigen::Vector3d vel = 0.5 * (vehicle_.pinVelocityWorld(0) + vehicle_.pinVelocityWorld(1));
            const double horizontal = std::hypot(mid.x() - seat_.east, mid.y() - seat_.north);
            const double height = mid.z() - seat_.pinHeight;
            if (shouldCloseArms(tower_, horizontal, height, -vel.z())) {
                change(TowerState::Closing);
            }
            break;
        }
        case TowerState::Closing: {
            const bool closed = std::abs(psi_[0] - psiClosed_) <= tower_.closedTolerance &&
                                std::abs(psi_[1] - psiClosed_) <= tower_.closedTolerance;
            if (closed) {
                change(TowerState::Closed);
            } else if (outsideTime_ >= tower_.zoneExitTime) {
                change(TowerState::Opening);
            }
            break;
        }
        case TowerState::Closed:
            if (outsideTime_ >= tower_.zoneExitTime || stateTime_ >= tower_.closedTimeout) {
                change(TowerState::Opening);
            }
            break;
        case TowerState::Opening:
            if (psi_[0] < tower_.closedTolerance && psi_[1] < tower_.closedTolerance) {
                change(TowerState::Open);
            }
            break;
        case TowerState::Holding:
            break;
    }
}

void TowerModel::checkContacts() {
    // Tower frame: a = approachSide, b = a rotated +90 deg. Arm 0 on the +b side, arm 1 on -b.
    const double ax = tower_.approachSide[0];
    const double ay = tower_.approachSide[1];
    const double bx = -ay;
    const double by = ax;
    ContactInput in;
    double padError[2][2] = {};  // pin to pad center, world east/north [m]
    double zSum = 0.0;
    for (int i = 0; i < 2; ++i) {
        const Eigen::Vector3d p = vehicle_.pinWorld(i);
        const double dx = p.x() - tower_.position[0];
        const double dy = p.y() - tower_.position[1];
        const double alongT = dx * ax + dy * ay;
        const double lateralT = dx * bx + dy * by;
        const int arm = lateralT >= 0.0 ? 0 : 1;
        const double side = arm == 0 ? 1.0 : -1.0;
        const double phi = tower_.phi(psi_[arm]);
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        const double relAlong = alongT - tower_.hingeSetback;
        const double relLateral = lateralT - side * tower_.hingeHalfSpacing;
        // Arm direction (c, -side*s) and its perpendicular (side*s, c) in (along, lateral).
        in.along[i] = relAlong * c - side * relLateral * s;
        in.across[i] = relAlong * side * s + relLateral * c;
        in.z[i] = p.z();
        zSum += p.z();
        // Pad center of this arm in world coordinates.
        const double padAlong = tower_.hingeSetback + tower_.cradleDistance * c;
        const double padLateral = side * (tower_.hingeHalfSpacing - tower_.cradleDistance * s);
        padError[i][0] = tower_.position[0] + padAlong * ax + padLateral * bx - p.x();
        padError[i][1] = tower_.position[1] + padAlong * ay + padLateral * by - p.y();
    }
    in.closure = std::min(psi_[0], psi_[1]) / psiClosed_;
    // Speeds at the pins (the contact points), not the base: the Ship rotates about its CG and the base is 22 m
    // from it, so a small angular rate shows as a base speed the cradles never feel.
    const Eigen::Vector3d v = 0.5 * (vehicle_.pinVelocityWorld(0) + vehicle_.pinVelocityWorld(1));
    in.sinkRate = -v.z();
    in.horizontalSpeed = std::hypot(v.x(), v.y());
    const HopperState& x = vehicle_.state();
    const Eigen::Matrix3d R = x.attitude.toRotationMatrix();
    in.tilt = std::acos(std::clamp(R(2, 2), -1.0, 1.0));
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    const double wanted = std::atan2(-ay, -ax);  // body +x points opposite to the approach side vector
    in.headingError = std::abs(std::remainder(yaw - wanted, 2.0 * std::numbers::pi));
    in.angularRate = x.angularRate.norm();

    const CatchResult r = evaluateContact(tower_, pinRadius_, in);
    if (r == CatchResult::None) {
        return;
    }
    result_ = r;
    if (r == CatchResult::Caught) {
        // Seat the pins on the cradles: a small snap (inside the catch window) of the pin midpoint.
        const Eigen::Vector3d shift(0.5 * (padError[0][0] + padError[1][0]), 0.5 * (padError[0][1] + padError[1][1]),
                                    tower_.railHeight + pinRadius_ - 0.5 * zSum);
        vehicle_.hold(Constraint::HeldByArms, shift);
        state_ = TowerState::Holding;
        stateTime_ = 0.0;
    } else {
        vehicle_.hold(Constraint::Crashed, Eigen::Vector3d::Zero());
    }
}

void TowerModel::checkMast() {
    // The hull as three discs of the hull radius (base, CG, nose) against the square mast.
    const Eigen::Vector3d points[3] = {vehicle_.pointWorld(Eigen::Vector3d::Zero()), vehicle_.state().position,
                                       vehicle_.pointWorld(Eigen::Vector3d(0.0, 0.0, length_))};
    const double ax = tower_.approachSide[0];
    const double ay = tower_.approachSide[1];
    for (const Eigen::Vector3d& p : points) {
        if (p.z() < 0.0 || p.z() > tower_.mastHeight) {
            continue;
        }
        const double dx = p.x() - tower_.position[0];
        const double dy = p.y() - tower_.position[1];
        const double alongT = std::abs(dx * ax + dy * ay);
        const double lateralT = std::abs(-dx * ay + dy * ax);
        const double ex = std::max(alongT - tower_.mastHalfWidth, 0.0);
        const double ey = std::max(lateralT - tower_.mastHalfWidth, 0.0);
        if (ex * ex + ey * ey <= hullRadius_ * hullRadius_) {
            result_ = CatchResult::TowerStrike;
            vehicle_.hold(Constraint::Crashed, Eigen::Vector3d::Zero());
            return;
        }
    }
}

void TowerModel::checkHullArms() {
    // The hull as a vertical cylinder about its axis (base to nose): sample the axis where it crosses the arm
    // slab (rail top down to the arm thickness). The tilt of the axis is small, so the plan section is taken as
    // a disc of the hull radius (the true ellipse is wider by 1/cos(tilt), under 0.2% at 3 degrees).
    const Eigen::Vector3d base = vehicle_.pointWorld(Eigen::Vector3d::Zero());
    const Eigen::Vector3d nose = vehicle_.pointWorld(Eigen::Vector3d(0.0, 0.0, length_));
    const double zTop = tower_.railHeight;
    const double zBottom = tower_.railHeight - tower_.armThickness;
    const double dz = nose.z() - base.z();
    if (std::abs(dz) < 1e-6 || std::max(base.z(), nose.z()) < zBottom || std::min(base.z(), nose.z()) > zTop) {
        return;
    }
    const double ax = tower_.approachSide[0];
    const double ay = tower_.approachSide[1];
    const double bx = -ay;
    const double by = ax;
    for (const double z : {zBottom, 0.5 * (zBottom + zTop), zTop}) {
        const double u = (z - base.z()) / dz;
        if (u < 0.0 || u > 1.0) {
            continue;
        }
        const Eigen::Vector3d p = base + u * (nose - base);
        const double dx = p.x() - tower_.position[0];
        const double dy = p.y() - tower_.position[1];
        const double along = dx * ax + dy * ay;
        const double lateral = dx * bx + dy * by;
        for (int arm = 0; arm < 2; ++arm) {
            if (hullTouchesArm(tower_, hullRadius_, psi_[arm], arm == 0 ? 1.0 : -1.0, along, lateral)) {
                result_ = CatchResult::ArmStrike;
                vehicle_.hold(Constraint::Crashed, Eigen::Vector3d::Zero());
                return;
            }
        }
    }
}

}  // namespace sim::vehicle
