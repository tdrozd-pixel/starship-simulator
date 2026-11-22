#include "TestFramework.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"

namespace {

struct HopResult {
    bool landed;
    double maxAltitude;
    double touchdownSpeed;
    double touchdownTime;
};

HopResult runHop(uint64_t seed, bool idealSensors) {
    sim::vehicle::HopScenarioOptions options;
    options.seed = seed;
    options.idealSensors = idealSensors;
    auto scenario = sim::vehicle::buildHopScenario(options);

    sim::PacerOptions pacer;
    pacer.realtime = false;
    pacer.maxSimTime = 120.0;
    sim::runPaced(*scenario.sim, pacer);

    const auto& td = scenario.vehicle->touchdown();
    return {scenario.fsw->computer().mode() == fsw::HopMode::Landed, scenario.vehicle->maxAltitude(),
            td.verticalSpeed, td.time};
}

}  // namespace

TEST_CASE(hop_lands_softly_with_ideal_sensors) {
    const HopResult r = runHop(1, true);
    CHECK(r.landed);
    CHECK_NEAR(r.maxAltitude, 50.0, 2.0);
    CHECK(r.touchdownSpeed > 0.0);
    CHECK(r.touchdownSpeed < 1.5);
}

TEST_CASE(hop_lands_softly_with_modeled_sensors) {
    for (uint64_t seed = 1; seed <= 5; ++seed) {
        const HopResult r = runHop(seed, false);
        CHECK(r.landed);
        CHECK_NEAR(r.maxAltitude, 50.0, 3.0);
        CHECK(r.touchdownSpeed < 2.0);
    }
}

TEST_CASE(hop_is_deterministic_for_a_seed) {
    const HopResult a = runHop(42, false);
    const HopResult b = runHop(42, false);
    CHECK(a.touchdownSpeed == b.touchdownSpeed);
    CHECK(a.touchdownTime == b.touchdownTime);
    CHECK(a.maxAltitude == b.maxAltitude);
}
