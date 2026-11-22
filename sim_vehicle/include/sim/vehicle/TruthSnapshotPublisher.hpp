#pragma once

#include <cstdint>
#include <string>

#include "sim/core/Model.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace sim::vehicle {

// Reads truth (read-only, no RNG) and hands snapshots to a sink at a fixed
// rate. Never allocates in update(). Add it last so flights are unchanged.
class TruthSnapshotPublisher final : public Model {
public:
    TruthSnapshotPublisher(std::string name, double rateHz, const HopperDynamics& truth, TruthSnapshotSink& sink);

    void update(const StepContext& ctx) override;

    // Pushes one more snapshot with the halted flag set. `step` must be greater
    // than the step of every periodic snapshot (Simulation::stepCount() after the
    // run is, since ctx.step is the index of a step that has already executed).
    void publishFinal(uint64_t step, double simTime);

private:
    void fill(TruthSnapshot& s, uint64_t step, double simTime) const;

    const HopperDynamics& truth_;
    TruthSnapshotSink& sink_;
};

}  // namespace sim::vehicle
