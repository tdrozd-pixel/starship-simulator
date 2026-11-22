#include "TestFramework.hpp"
#include "sim/env/Atmosphere.hpp"

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
