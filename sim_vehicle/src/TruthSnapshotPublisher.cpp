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
    for (int i = 0; i < 3; ++i) {
        s.position[i] = x.position[i];
        s.velocity[i] = x.velocity[i];
        s.angularRate[i] = x.angularRate[i];
    }
    s.attitude[0] = x.attitude.w();
    s.attitude[1] = x.attitude.x();
    s.attitude[2] = x.attitude.y();
    s.attitude[3] = x.attitude.z();
    s.throttle = x.throttle;  // actual, from the actuator model
    s.gimbalPitch = x.gimbal.x();
    s.gimbalYaw = x.gimbal.y();
    s.propellantMass = x.propellantMass;
    s.totalMass = truth_.mass();
}

}  // namespace sim::vehicle
