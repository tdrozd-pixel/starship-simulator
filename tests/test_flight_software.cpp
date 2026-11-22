#include "TestFramework.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

TEST_CASE(pid_proportional_only) {
    fsw::Pid pid({2.0, 0.0, 0.0});
    CHECK_NEAR(pid.update(1.0, 0.25, 0.01), 1.5, 1e-12);
}

TEST_CASE(pid_output_clamps_and_integrator_does_not_wind_up) {
    fsw::Pid pid({1.0, 10.0, 0.0, -1.0, 1.0, 100.0});
    for (int i = 0; i < 1000; ++i) {
        CHECK(pid.update(100.0, 0.0, 0.01) <= 1.0);
    }
    // After a long saturation, the integrator must not hold the output high.
    const double out = pid.update(0.0, 0.5, 0.01);
    CHECK(out < 0.0);
}

TEST_CASE(pid_integral_removes_steady_state_error) {
    // Plant: x' = u - 1 (constant disturbance). PI should drive x to the setpoint.
    fsw::Pid pid({2.0, 1.0, 0.0, -10.0, 10.0, 10.0});
    double x = 0.0;
    for (int i = 0; i < 20000; ++i) {
        const double u = pid.update(1.0, x, 0.001);
        x += (u - 1.0) * 0.001;
    }
    CHECK_NEAR(x, 1.0, 1e-3);
}

TEST_CASE(vertical_estimator_tracks_constant_climb) {
    fsw::VerticalEstimator est({9.80665, 4.0, 4.0});
    const double dt = 0.01;
    for (int i = 1; i <= 1000; ++i) {
        const double t = i * dt;
        est.predict(9.80665, dt);  // unaccelerated
        est.correct(2.0 * t, dt);  // climbing at 2 m/s
    }
    CHECK_NEAR(est.velocity(), 2.0, 0.05);
    CHECK_NEAR(est.altitude(), 20.0, 0.1);
}
