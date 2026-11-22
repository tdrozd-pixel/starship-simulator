#include <cmath>
#include <vector>

#include "TestFramework.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace {

using sim::vehicle::TruthSnapshot;

class VectorSink final : public sim::vehicle::TruthSnapshotSink {
public:
    VectorSink() { snapshots.reserve(20000); }
    void push(const TruthSnapshot& s) noexcept override { snapshots.push_back(s); }
    std::vector<TruthSnapshot> snapshots;
};

struct SnapshotRun {
    sim::vehicle::HopScenario scenario;
    VectorSink sink;
};

void runHopUnpaced(SnapshotRun& run, uint64_t seed, bool withSink) {
    sim::vehicle::HopScenarioOptions options;
    options.seed = seed;
    if (withSink) {
        options.snapshotSink = &run.sink;
    }
    run.scenario = sim::vehicle::buildHopScenario(options);
    sim::PacerOptions pacer;
    pacer.realtime = false;
    pacer.maxSimTime = 120.0;
    sim::runPaced(*run.scenario.sim, pacer);
}

}  // namespace

TEST_CASE(publisher_runs_at_100hz) {
    SnapshotRun run;
    runHopUnpaced(run, 1, true);
    const auto& snaps = run.sink.snapshots;
    CHECK(!snaps.empty());
    const double simTime = run.scenario.sim->time();
    const double expected = simTime * 100.0 + 1.0;
    CHECK(std::abs(static_cast<double>(snaps.size()) - expected) <= 2.0);
    for (std::size_t i = 0; i < snaps.size(); ++i) {
        CHECK(snaps[i].step == i * 10);
        CHECK(snaps[i].simTime == static_cast<double>(snaps[i].step) * 0.001);
        if (i > 0) {
            CHECK(snaps[i].simTime > snaps[i - 1].simTime);
        }
    }
}

TEST_CASE(publisher_fields_match_truth) {
    SnapshotRun run;
    runHopUnpaced(run, 1, true);
    const TruthSnapshot& s = run.sink.snapshots.back();
    const auto& vehicle = *run.scenario.vehicle;
    const auto& x = vehicle.state();
    // The last periodic snapshot is at most one publish interval before the end,
    // and the vehicle is stopped on the ground by then.
    // The stream's reference point is the vehicle base, on the ground by now.
    CHECK_NEAR(s.position[2], vehicle.basePosition().z(), 1e-6);
    CHECK_NEAR(s.position[0], vehicle.basePosition().x(), 1e-6);
    CHECK_NEAR(s.propellantMass, x.propellantMass, 1e-6);
    CHECK_NEAR(s.totalMass, vehicle.mass(), 1e-6);
    CHECK_NEAR(s.throttle, vehicle.meanThrottle(), 0.01);  // snapshot may precede the final step slightly
    CHECK_NEAR(s.gimbalPitch, vehicle.meanGimbal().x(), 0.01);
    CHECK_NEAR(s.gimbalYaw, vehicle.meanGimbal().y(), 0.01);
    const double q = std::sqrt(s.attitude[0] * s.attitude[0] + s.attitude[1] * s.attitude[1] +
                               s.attitude[2] * s.attitude[2] + s.attitude[3] * s.attitude[3]);
    CHECK_NEAR(q, 1.0, 1e-6);
    CHECK((s.flags & sim::vehicle::kSnapTouchdown) != 0);
    CHECK((s.flags & sim::vehicle::kSnapHalted) == 0);
    CHECK(s.runId == 0);

    // Mid-flight: the engine flag and a non-zero actual throttle agree.
    bool sawEngine = false;
    for (const TruthSnapshot& m : run.sink.snapshots) {
        if ((m.flags & sim::vehicle::kSnapEngineRunning) != 0) {
            sawEngine = true;
            CHECK(m.throttle > 0.0);
        }
    }
    CHECK(sawEngine);
}

TEST_CASE(publisher_does_not_change_flight) {
    SnapshotRun with;
    SnapshotRun without;
    runHopUnpaced(with, 7, true);
    runHopUnpaced(without, 7, false);
    CHECK(with.scenario.vehicle->maxAltitude() == without.scenario.vehicle->maxAltitude());
    CHECK(with.scenario.vehicle->touchdown().time == without.scenario.vehicle->touchdown().time);
    CHECK(with.scenario.vehicle->touchdown().verticalSpeed == without.scenario.vehicle->touchdown().verticalSpeed);
    CHECK(without.scenario.snapshotPublisher == nullptr);
    const auto& a = with.scenario.vehicle->state();
    const auto& b = without.scenario.vehicle->state();
    for (int i = 0; i < 3; ++i) {
        CHECK(a.position[i] == b.position[i]);
        CHECK(a.velocity[i] == b.velocity[i]);
    }
    CHECK(a.attitude.w() == b.attitude.w() && a.attitude.x() == b.attitude.x() &&
          a.attitude.y() == b.attitude.y() && a.attitude.z() == b.attitude.z());
    CHECK(a.propellantMass == b.propellantMass);
    CHECK(with.scenario.sim->stepCount() == without.scenario.sim->stepCount());
}

TEST_CASE(publish_final_sets_halted) {
    SnapshotRun run;
    runHopUnpaced(run, 1, true);
    const uint64_t lastPeriodicStep = run.sink.snapshots.back().step;
    const std::size_t before = run.sink.snapshots.size();
    run.scenario.snapshotPublisher->publishFinal(run.scenario.sim->stepCount(), run.scenario.sim->time());
    CHECK(run.sink.snapshots.size() == before + 1);
    const TruthSnapshot& f = run.sink.snapshots.back();
    CHECK(f.step > lastPeriodicStep);
    CHECK((f.flags & sim::vehicle::kSnapHalted) != 0);
}
