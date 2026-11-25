#include "sim/vehicle/TruthSnapshotPublisher.hpp"

#include <utility>

namespace sim::vehicle {

TruthSnapshotPublisher::TruthSnapshotPublisher(std::string name, double rateHz, const HopperDynamics& truth,
                                               TruthSnapshotSink& sink)
    : Model(std::move(name), rateHz), truth_(truth), sink_(sink) {}

void TruthSnapshotPublisher::update(const StepContext& ctx) {
    TruthSnapshot s;
    fill(s, ctx.step, ctx.t);
    sink_.push(s);
}

void TruthSnapshotPublisher::publishFinal(uint64_t step, double simTime) {
    TruthSnapshot s;
    fill(s, step, simTime);
    s.flags |= kSnapHalted;
    sink_.push(s);
}

void TruthSnapshotPublisher::fill(TruthSnapshot& s, uint64_t step, double simTime) const {
    const HopperState& x = truth_.state();
    s.step = step;
    s.simTime = simTime;
    s.flags = 0;
    if (truth_.engineRunning()) {
        s.flags |= kSnapEngineRunning;
    }
    if (truth_.touchdown().occurred) {
        s.flags |= kSnapTouchdown;
    }
    s.armAngle[0] = 0.0;
    s.armAngle[1] = 0.0;
    s.towerState = 0;
    s.catchResult = 0;
    if (tower_ != nullptr) {
        s.armAngle[0] = tower_->armAngle(0);
        s.armAngle[1] = tower_->armAngle(1);
        s.towerState = static_cast<uint32_t>(tower_->state());
        s.catchResult = static_cast<uint32_t>(tower_->result());
        if (tower_->result() == CatchResult::Caught) {
            s.flags |= kSnapCaught;
        } else if (tower_->result() != CatchResult::None) {
            s.flags |= kSnapCrashed;
        }
    }
    // The truth stream's reference point is the vehicle base, not the CG.
    const Eigen::Vector3d base = truth_.basePosition();
    const Eigen::Vector3d baseVelocity = truth_.pointVelocityWorld(Eigen::Vector3d::Zero());
    for (int i = 0; i < 3; ++i) {
        s.position[i] = base[i];
        s.velocity[i] = baseVelocity[i];
        s.angularRate[i] = x.angularRate[i];
    }
    s.attitude[0] = x.attitude.w();
    s.attitude[1] = x.attitude.x();
    s.attitude[2] = x.attitude.y();
    s.attitude[3] = x.attitude.z();
    // Actual actuator positions, averaged over the engines (protocol v1 has one engine).
    // TODO(phase 0, item 1): per-engine actuator fields in truth stream v2.
    const Eigen::Vector2d gimbal = truth_.meanGimbal();
    s.throttle = truth_.meanThrottle();
    s.gimbalPitch = gimbal.x();
    s.gimbalYaw = gimbal.y();
    s.propellantMass = x.propellantMass;
    s.totalMass = truth_.mass();
    for (int i = 0; i < kEngineCount; ++i) {
        s.engineThrottle[i] = x.throttle[i];
        s.engineRunning[i] = truth_.engineRunning(i);
    }
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        s.engineGimbalPitch[i] = x.gimbal(0, i);
        s.engineGimbalYaw[i] = x.gimbal(1, i);
    }
}

}  // namespace sim::vehicle
