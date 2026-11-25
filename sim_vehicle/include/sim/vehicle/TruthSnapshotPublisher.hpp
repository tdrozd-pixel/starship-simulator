#pragma once

#include <cstdint>
#include <string>

#include "sim/core/Model.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/Tower.hpp"
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
    // Optional tower whose arm angles, state and catch result go into the snapshots (non-owning).
    // Null reports zeros, Open, None.
    void setTower(const TowerModel* tower) { tower_ = tower; }

    void publishFinal(uint64_t step, double simTime);

private:
    void fill(TruthSnapshot& s, uint64_t step, double simTime) const;

    const HopperDynamics& truth_;
    TruthSnapshotSink& sink_;
    const TowerModel* tower_ = nullptr;
};

}  // namespace sim::vehicle
