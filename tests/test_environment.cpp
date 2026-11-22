#include "TestFramework.hpp"
#include "sim/env/Atmosphere.hpp"
#include "sim/env/Weather.hpp"

// Reference values from the U.S. Standard Atmosphere 1976 tables.
TEST_CASE(us76_sea_level) {
    const sim::env::StandardAtmosphere1976 atm;
    const auto s = atm.at(0.0);
    CHECK_NEAR(s.temperature, 288.15, 1e-6);
    CHECK_NEAR(s.pressure, 101325.0, 1e-3);
    CHECK_NEAR(s.density, 1.2250, 1e-4);
    CHECK_NEAR(s.speedOfSound, 340.29, 0.01);
}

TEST_CASE(us76_tropopause_and_stratosphere) {
    const sim::env::StandardAtmosphere1976 atm;
    const auto s11 = atm.at(11000.0);  // geometric
    CHECK_NEAR(s11.temperature, 216.77, 0.01);
    CHECK_NEAR(s11.pressure, 22700.0, 5.0);
    CHECK_NEAR(s11.density, 0.36480, 1e-4);

    const auto s30 = atm.at(30000.0);
    CHECK_NEAR(s30.temperature, 226.51, 0.01);
    CHECK_NEAR(s30.pressure, 1197.0, 1.0);
    CHECK_NEAR(s30.density, 0.018410, 1e-5);
}

TEST_CASE(us76_is_monotonic_in_density) {
    const sim::env::StandardAtmosphere1976 atm;
    double last = atm.at(0.0).density;
    for (double h = 500.0; h <= 120000.0; h += 500.0) {
        const double rho = atm.at(h).density;
        CHECK(rho < last);
        last = rho;
    }
}

TEST_CASE(wind_profile_is_piecewise_linear_and_clamped) {
    sim::env::WindProfile profile({
        {0.0, Eigen::Vector3d(5.0, 0.0, 0.0)},
        {1000.0, Eigen::Vector3d(15.0, 2.0, 0.0)},
    });

    const Eigen::Vector3d low = profile.at(-100.0);
    const Eigen::Vector3d middle = profile.at(500.0);
    const Eigen::Vector3d high = profile.at(2000.0);

    CHECK_NEAR(low.x(), 5.0, 1e-12);
    CHECK_NEAR(middle.x(), 10.0, 1e-12);
    CHECK_NEAR(middle.y(), 1.0, 1e-12);
    CHECK_NEAR(high.x(), 15.0, 1e-12);
}

TEST_CASE(one_cosine_gust_starts_zero_peaks_and_ends_zero) {
    sim::env::GustParameters parameters;
    parameters.amplitude = Eigen::Vector3d(10.0, 0.0, 0.0);
    parameters.startTime = 5.0;
    parameters.duration = 10.0;
    const sim::env::Gust gust(parameters);

    CHECK_NEAR(gust.at(5.0).x(), 0.0, 1e-12);
    CHECK_NEAR(gust.at(10.0).x(), 10.0, 1e-12);
    CHECK_NEAR(gust.at(15.0).x(), 0.0, 1e-12);
    CHECK_NEAR(gust.at(4.0).x(), 0.0, 1e-12);
}

TEST_CASE(dryden_turbulence_is_seeded_and_bounded) {
    sim::env::TurbulenceParameters parameters;
    parameters.sigma = Eigen::Vector3d::Constant(2.0);
    parameters.scaleLength = Eigen::Vector3d::Constant(100.0);

    sim::env::DrydenTurbulence a(sim::deriveSeed(42, "weather_a"), parameters);
    sim::env::DrydenTurbulence b(sim::deriveSeed(42, "weather_a"), parameters);
    sim::env::DrydenTurbulence c(sim::deriveSeed(42, "weather_b"), parameters);
    bool equal = true;
    bool different = false;
    double rmsSquared = 0.0;
    for (int i = 0; i < 10000; ++i) {
        const Eigen::Vector3d va = a.step(0.001, 30.0);
        const Eigen::Vector3d vb = b.step(0.001, 30.0);
        const Eigen::Vector3d vc = c.step(0.001, 30.0);
        equal = equal && va.isApprox(vb, 1e-12);
        different = different || !va.isApprox(vc, 1e-12);
        rmsSquared += va.squaredNorm();
        CHECK(std::isfinite(va.x()) && std::isfinite(va.y()) && std::isfinite(va.z()));
    }
    CHECK(equal);
    CHECK(different);
    CHECK(rmsSquared / 10000.0 < 20.0);
}
