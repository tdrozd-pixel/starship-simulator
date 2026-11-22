#include "TestFramework.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

namespace {

class TestHal final : public fsw::Hal {
public:
    double timeSec() const override { return time; }
    fsw::ImuSample readImu() override { return imu; }
    fsw::AltimeterSample readAltimeter() override { return altimeter; }
    void commandActuators(const fsw::ActuatorCommand& value) override { command = value; }
    void publishTelemetry(const fsw::Telemetry& value) override { telemetry = value; }

    double time = 0.0;
    fsw::ImuSample imu;
    fsw::AltimeterSample altimeter;
    fsw::ActuatorCommand command;
    fsw::Telemetry telemetry;
};

}  // namespace

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

TEST_CASE(hop_flight_computer_commands_tvc_against_measured_rotation) {
    TestHal hal;
    hal.altimeter.valid = true;
    hal.imu.valid = true;
    hal.imu.specificForce[2] = 9.80665;
    hal.imu.angularRate[0] = 0.1;
    hal.imu.angularRate[1] = -0.1;

    fsw::HopConfig config;
    config.ignitionTime = 0.0;
    config.dryMass = 20000.0;
    config.initialPropellant = 10000.0;
    config.maxThrust = 600000.0;
    config.ispSeaLevel = 330.0;
    config.minThrottle = 0.4;
    config.vehicleRadius = 4.5;
    config.vehicleLength = 20.0;
    config.engineLeverArm = 8.0;
    config.gimbalLimit = 15.0 * 3.14159265358979323846 / 180.0;
    fsw::HopFlightComputer computer(hal, config);

    hal.time = 0.01;
    hal.imu.stamp = hal.time;
    computer.tick();
    hal.time = 0.02;
    hal.imu.stamp = hal.time;
    computer.tick();

    CHECK(hal.command.engineOn);
    CHECK(hal.command.gimbalPitch > 0.0);
    CHECK(hal.command.gimbalYaw < 0.0);
    CHECK(std::abs(hal.command.gimbalPitch) <= config.gimbalLimit);
    CHECK(std::abs(hal.command.gimbalYaw) <= config.gimbalLimit);
}
