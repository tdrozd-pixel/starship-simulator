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
    double touchdownHorizontalSpeed;
    double touchdownTime;
    double missDistance;  // base from the target at touchdown [m]
    double maxTiltDeg;
};

HopResult runHop(sim::vehicle::HopScenarioOptions options) {
    auto scenario = sim::vehicle::buildHopScenario(options);
    double maxTilt = 0.0;
    while (!scenario.sim->halted() && scenario.sim->time() < 150.0) {
        scenario.sim->step();
        const double cosTilt = (scenario.vehicle->state().attitude * Eigen::Vector3d::UnitZ()).z();
        maxTilt = std::max(maxTilt, std::acos(std::clamp(cosTilt, -1.0, 1.0)));
    }
    const auto& td = scenario.vehicle->touchdown();
    return {scenario.fsw->computer().mode() == fsw::HopMode::Landed,
            scenario.vehicle->maxAltitude(),
            td.verticalSpeed,
            td.horizontalSpeed,
            td.time,
            std::hypot(td.east, td.north),
            maxTilt * 180.0 / std::numbers::pi};
}

HopResult runHop(uint64_t seed, bool idealSensors) {
    sim::vehicle::HopScenarioOptions options;
    options.seed = seed;
    options.idealSensors = idealSensors;
    return runHop(options);
}

}  // namespace

TEST_CASE(hop_is_deterministic_for_a_seed) {
    const HopResult a = runHop(42, false);
    const HopResult b = runHop(42, false);
    CHECK(a.touchdownSpeed == b.touchdownSpeed);
    CHECK(a.touchdownTime == b.touchdownTime);
    CHECK(a.maxAltitude == b.maxAltitude);
    CHECK(a.missDistance == b.missDistance);
}

TEST_CASE(hop_weather_changes_air_relative_drag) {
    // Same flight with and without wind: the wind pushes the vehicle downwind (east)
    // before the position loop catches up.
    auto eastAfterLiftoff = [](bool wind) {
        sim::vehicle::HopScenarioOptions options;
        options.idealSensors = true;
        options.weather.gusts.clear();
        if (!wind) {
            options.weather.profile = sim::env::WindProfile();
            options.weather.turbulence.sigma.setZero();
        }
        auto scenario = sim::vehicle::buildHopScenario(options);
        for (int i = 0; i < 8000; ++i) {
            scenario.sim->step();
        }
        CHECK(scenario.vehicle->maxAltitude() > 5.0);
        return std::pair{scenario.vehicle->basePosition().x(), scenario.weather->current().meanWind.x()};
    };
    const auto [eastWind, meanWind] = eastAfterLiftoff(true);
    const auto [eastCalm, calmWind] = eastAfterLiftoff(false);
    CHECK(meanWind > 0.0);
    CHECK(calmWind == 0.0);
    CHECK(eastWind > eastCalm);
}

TEST_CASE(hop_tvc_recovers_from_initial_tilt) {
    sim::vehicle::HopScenarioOptions options;
    options.landing = fsw::LandingMode::Pad;  // the catch flight translates (and tilts) from 60 m; this tests the TVC alone
    options.idealSensors = true;
    options.initialAttitude =
        Eigen::Quaterniond(Eigen::AngleAxisd(6.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(-4.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitY()));
    options.initialAngularRate = Eigen::Vector3d(0.03, -0.02, 0.0);
    options.weather.profile = sim::env::WindProfile();
    options.weather.turbulence.sigma.setZero();
    options.weather.gusts.clear();

    auto scenario = sim::vehicle::buildHopScenario(options);
    double maxGimbal = 0.0;
    for (int i = 0; i < 12000; ++i) {
        scenario.sim->step();
        maxGimbal = std::max(maxGimbal, scenario.vehicle->state().gimbal.cwiseAbs().maxCoeff());
    }

    const double finalTilt = Eigen::AngleAxisd(scenario.vehicle->state().attitude).angle();
    CHECK(maxGimbal > 0.01);
    CHECK(finalTilt < 6.0 * std::numbers::pi / 180.0);
}

TEST_CASE(hop_controls_roll_with_differential_gimbal) {
    sim::vehicle::HopScenarioOptions options;
    options.idealSensors = true;
    options.initialAngularRate = Eigen::Vector3d(0.0, 0.0, 0.02);  // [rad/s] roll rate at liftoff
    options.weather.profile = sim::env::WindProfile();
    options.weather.turbulence.sigma.setZero();
    options.weather.gusts.clear();

    auto scenario = sim::vehicle::buildHopScenario(options);
    for (int i = 0; i < 25000; ++i) {
        scenario.sim->step();
    }
    CHECK(std::abs(scenario.vehicle->state().angularRate.z()) < 0.005);
}

TEST_CASE(hop_engines_produce_rated_sea_level_thrust) {
    const sim::vehicle::HopperParams p;
    // F = throttle * F_vac - p_amb * A_exit gives the rated thrust at full throttle, sea level.
    CHECK_NEAR(p.vacuumThrust() - sim::vehicle::kSeaLevelPressure * p.nozzleExitArea(), p.seaLevelThrust, 1e-6);
    CHECK_NEAR(p.seaLevelThrust, 280.0 * 9.80665e3, 1e-6);
    // All three at minimum throttle cannot hold up the hop mass (otherwise it cannot land).
    const double minThrust = 3.0 * (p.minThrottle * p.vacuumThrust() - sim::vehicle::kSeaLevelPressure * p.nozzleExitArea());
    CHECK(minThrust < (p.dryMass + p.initialPropellant) * 9.80665 * 0.8);
}
