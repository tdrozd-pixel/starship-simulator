#include "TestFramework.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

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

TEST_CASE(hop_weather_changes_air_relative_drag) {
    sim::vehicle::HopScenarioOptions options;
    options.idealSensors = true;
    auto scenario = sim::vehicle::buildHopScenario(options);

    for (int i = 0; i < 3000; ++i) {
        scenario.sim->step();
    }

    CHECK(scenario.weather->current().meanWind.x() > 0.0);
    CHECK(scenario.weather->current().wind.x() > 0.0);
    CHECK(scenario.vehicle->state().position.x() > 0.0);
}

TEST_CASE(hop_tvc_recovers_from_initial_tilt) {
    sim::vehicle::HopScenarioOptions options;
    options.idealSensors = true;
    options.initialAttitude =
        Eigen::Quaterniond(Eigen::AngleAxisd(6.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(-4.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitY()));
    options.initialAngularRate = Eigen::Vector3d(0.03, -0.02, 0.0);
    options.weather.profile = sim::env::WindProfile();
    options.weather.turbulence.sigma.setZero();

    auto scenario = sim::vehicle::buildHopScenario(options);
    double maxGimbal = 0.0;
    for (int i = 0; i < 8000; ++i) {
        scenario.sim->step();
        maxGimbal = std::max(maxGimbal, scenario.vehicle->state().gimbal.norm());
    }

    const double finalTilt = Eigen::AngleAxisd(scenario.vehicle->state().attitude).angle();
    CHECK(maxGimbal > 0.01);
    CHECK(finalTilt < 6.0 * std::numbers::pi / 180.0);
}
