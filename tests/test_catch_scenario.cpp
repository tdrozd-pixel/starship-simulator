// Catch tower scenario tests: the Pad flight is unchanged, and a smoke test of the catch flight.
#include <cstdint>

#include "TestFramework.hpp"
#include "sim/vehicle/HopScenario.hpp"

// The existing Pad-mode flight must not change when the catch tower feature lands: the seed-1
// default hop is pinned bit for bit. The constants were recorded from the baseline before the
// catch work started (hex floats, exact). Stage A makes the default landing mode Catch; this
// test must then set options.landing = Pad explicitly.
// Do not update these constants to follow a Pad guidance change unless that change is intended.
// Re-recorded on purpose when the GNSS fusion became delayed-measurement (lever arm at the fix's own
// attitude, update applied at the fix time and replayed): the estimator is shared with the Pad flight.
// Before: time 0x1.c78f5c28f5c29p+5 (56.94 s), 58946 steps, miss east -0.102 m north -0.422 m.
TEST_CASE(pad_mode_flight_unchanged) {
    sim::vehicle::HopScenarioOptions options;
    options.seed = 1;
    options.landing = fsw::LandingMode::Pad;
    auto scenario = sim::vehicle::buildHopScenario(options);
    while (!scenario.sim->halted() && scenario.sim->time() < 150.0) {
        scenario.sim->step();
    }
    const auto& td = scenario.vehicle->touchdown();
    CHECK(td.occurred);
    CHECK(td.time == 0x1.cd9ba5e353f7dp+5);
    CHECK(td.east == -0x1.396c74b329718p-2);
    CHECK(td.north == -0x1.988f2d95c372ap-2);
    CHECK(td.verticalSpeed == 0x1.a7883ba266a4ep-2);
    CHECK(td.horizontalSpeed == 0x1.2deb101d32066p-5);
    CHECK(scenario.sim->stepCount() == 59702u);
}

namespace {

struct CatchRun {
    bool halted = false;
    uint64_t steps = 0;
    int towerState = -1;
    int result = -1;
    double east = 0.0;
    double north = 0.0;
    double altitude = 0.0;
    double propellant = 0.0;
    int mode = -1;
};

// The default (Catch) landing without the wind gust.
CatchRun flyCatch(uint64_t seed) {
    sim::vehicle::HopScenarioOptions options;
    options.seed = seed;
    options.weather.gusts.clear();
    auto scenario = sim::vehicle::buildHopScenario(options);
    while (!scenario.sim->halted() && scenario.sim->time() < 200.0) {
        scenario.sim->step();
    }
    CatchRun run;
    run.halted = scenario.sim->halted();
    run.steps = scenario.sim->stepCount();
    run.towerState = static_cast<int>(scenario.tower->state());
    run.result = static_cast<int>(scenario.tower->result());
    const Eigen::Vector3d base = scenario.vehicle->basePosition();
    run.east = base.x();
    run.north = base.y();
    run.altitude = base.z();
    run.propellant = scenario.vehicle->state().propellantMass;
    run.mode = static_cast<int>(scenario.fsw->computer().mode());
    return run;
}

}  // namespace

// Smoke test of the catch scenario: it halts, two runs of the same seed are bit identical, and the
// tower state and catch result are valid enum values. Whether the Ship is caught is flight
// performance (tests/CLAUDE.md): run sim_cli and look at the result instead of asserting it here.
TEST_CASE(catch_scenario_halts_and_is_deterministic) {
    const CatchRun a = flyCatch(1);
    const CatchRun b = flyCatch(1);
    CHECK(a.halted);
    CHECK(a.towerState >= static_cast<int>(sim::vehicle::TowerState::Open));
    CHECK(a.towerState <= static_cast<int>(sim::vehicle::TowerState::Opening));
    CHECK(a.result >= static_cast<int>(sim::vehicle::CatchResult::None));
    CHECK(a.result <= static_cast<int>(sim::vehicle::CatchResult::TowerStrike));
    CHECK(a.mode >= 0 && a.mode <= 10);
    CHECK(a.steps == b.steps);
    CHECK(a.towerState == b.towerState);
    CHECK(a.result == b.result);
    CHECK(a.mode == b.mode);
    CHECK(a.east == b.east);
    CHECK(a.north == b.north);
    CHECK(a.altitude == b.altitude);
    CHECK(a.propellant == b.propellant);
}
