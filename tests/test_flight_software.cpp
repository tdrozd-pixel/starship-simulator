#include "TestFramework.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "fsw/Pid.hpp"
#include "fsw/VerticalEstimator.hpp"

#include <vector>

namespace {

class TestHal final : public fsw::Hal {
public:
    double timeSec() const override { return time; }
    fsw::ImuSample readImu() override { return imu; }
    fsw::AltimeterSample readAltimeter() override { return altimeter; }
    fsw::GnssSample readGnss() override { return gnss; }
    void commandActuators(const fsw::ActuatorCommand& value) override { command = value; }
    fsw::GimbalFeedbackSample readGimbalFeedback() override { return gimbal; }
    fsw::EngineFeedbackSample readEngineFeedback() override { return engineFeedback; }
    void publishTelemetry(const fsw::Telemetry& value) override { telemetry = value; }
    void downlink(const uint8_t* packet, std::size_t length) override {
        packets.emplace_back(packet, packet + length);
    }

    double time = 0.0;
    fsw::ImuSample imu;
    fsw::AltimeterSample altimeter;
    fsw::GnssSample gnss;
    fsw::ActuatorCommand command;
    fsw::Telemetry telemetry;
    fsw::GimbalFeedbackSample gimbal;
    fsw::EngineFeedbackSample engineFeedback;
    std::vector<std::vector<uint8_t>> packets;
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

namespace {

fsw::HopConfig testConfig() {
    fsw::HopConfig config;
    config.ignitionTime = 0.0;
    config.gnssFixesRequired = 1;
    config.target = {0.4537, -1.6957, -22.0};
    config.dryMass = 150000.0;
    config.initialPropellant = 450000.0;
    config.vacuumThrust = 2.88e6;
    config.nozzleExitArea = 1.33;
    config.vacuumIsp = 350.0;
    config.minThrottle = 0.4;
    config.engineLeverArm = 9.5;
    config.engineClusterRadius = 1.55;
    config.engineAzimuth0 = 1.5707963267948966;
    config.gimbalLimit = 15.0 * 3.14159265358979323846 / 180.0;
    config.transverseInertiaDry = 4.7e7;
    config.transverseInertiaPropellant = 1.2e7;
    config.axialInertiaDry = 3.0e6;
    config.axialInertiaPropellant = 4.6e6;
    return config;
}

void feedGnssAtTarget(TestHal& hal, const fsw::HopConfig& config) {
    hal.gnss.valid = true;
    hal.gnss.stamp = hal.time;
    hal.gnss.fixTime = hal.time;
    hal.gnss.fixType = fsw::GnssFixType::RtkFixed;
    hal.gnss.latitude = config.target.latitude;
    hal.gnss.longitude = config.target.longitude;
    hal.gnss.height = config.target.height;
    hal.gnss.horizontalSigma = 0.02;
}

}  // namespace

TEST_CASE(hop_flight_computer_waits_for_gnss_before_ignition) {
    TestHal hal;
    hal.imu.valid = true;
    hal.imu.specificForce[2] = 9.80665;
    fsw::HopConfig config = testConfig();
    config.gnssFixesRequired = 5;
    fsw::HopFlightComputer computer(hal, config);
    for (int i = 1; i <= 20; ++i) {
        hal.time = 0.01 * i;
        hal.imu.stamp = hal.time;
        computer.tick();
    }
    CHECK(computer.mode() == fsw::HopMode::Prelaunch);
    CHECK(!hal.command.engines[0].on);
    for (int i = 21; i <= 30; ++i) {
        hal.time = 0.01 * i;
        hal.imu.stamp = hal.time;
        feedGnssAtTarget(hal, config);
        computer.tick();
    }
    CHECK(computer.mode() == fsw::HopMode::Ignition);
    CHECK(hal.command.engines[0].on && hal.command.engines[1].on && hal.command.engines[2].on);
    // Liftoff throttle gives about 1.15 thrust-to-weight with the pressure-corrected thrust model.
    const double perEngine = hal.command.engines[0].throttle * config.vacuumThrust -
                             config.ambientPressure * config.nozzleExitArea;
    CHECK_NEAR(3.0 * perEngine / (600000.0 * 9.80665), 1.15, 0.01);
}

TEST_CASE(hop_flight_computer_commands_tvc_against_measured_rotation) {
    TestHal hal;
    hal.altimeter.valid = true;
    hal.imu.valid = true;
    hal.imu.specificForce[2] = 9.80665;
    hal.imu.angularRate[0] = 0.1;
    hal.imu.angularRate[1] = -0.1;

    const fsw::HopConfig config = testConfig();
    fsw::HopFlightComputer computer(hal, config);

    for (int i = 1; i <= 2; ++i) {
        hal.time = 0.01 * i;
        hal.imu.stamp = hal.time;
        feedGnssAtTarget(hal, config);
        computer.tick();
    }

    for (int i = 0; i < fsw::kSeaLevelEngineCount; ++i) {
        CHECK(hal.command.engines[i].on);
    }
    for (int i = fsw::kFirstRvac; i < fsw::kEngineCount; ++i) {  // RVacs stay off for the hop
        CHECK(!hal.command.engines[i].on);
        CHECK(hal.command.engines[i].throttle == 0.0);
    }
    for (const fsw::GimbalCommand& g : hal.command.gimbals) {
        CHECK(g.pitch > 0.0);
        CHECK(g.yaw < 0.0);
        CHECK(std::abs(g.pitch) <= config.gimbalLimit);
        CHECK(std::abs(g.yaw) <= config.gimbalLimit);
    }
}

TEST_CASE(hop_flight_computer_counters_roll_rate_with_differential_gimbal) {
    TestHal hal;
    hal.imu.valid = true;
    hal.imu.specificForce[2] = 9.80665;
    hal.imu.angularRate[2] = 0.05;  // rolling +z
    const fsw::HopConfig config = testConfig();
    fsw::HopFlightComputer computer(hal, config);
    for (int i = 1; i <= 2; ++i) {
        hal.time = 0.01 * i;
        hal.imu.stamp = hal.time;
        feedGnssAtTarget(hal, config);
        computer.tick();
    }
    // Net roll torque from the commanded tangential tilts must oppose the rate:
    // engine i at azimuth a, lateral thrust (sin yaw, -sin pitch): torque_z ~ r x lateral.
    double torqueZ = 0.0;
    for (int i = 0; i < fsw::kSeaLevelEngineCount; ++i) {
        const double a = config.engineAzimuth0 + 2.0 * 3.14159265358979323846 * i / fsw::kSeaLevelEngineCount;
        const double lx = std::sin(hal.command.gimbals[i].yaw);
        const double ly = -std::sin(hal.command.gimbals[i].pitch);
        torqueZ += std::cos(a) * ly - std::sin(a) * lx;
    }
    CHECK(torqueZ < 0.0);
}

TEST_CASE(hop_flight_computer_downlinks_all_periodic_packets_on_cycle_zero) {
    TestHal hal;
    hal.imu.valid = true;
    hal.imu.specificForce[2] = 9.80665;
    const fsw::HopConfig config = testConfig();
    fsw::HopFlightComputer computer(hal, config);
    hal.time = 0.01;
    hal.imu.stamp = hal.time;
    computer.tick();
    // No GNSS solution yet: status, config, navigation, guidance, actuators, engine status,
    // catch status, catch config (the catch packets are also sent in Pad mode).
    std::vector<unsigned> apids;
    for (const auto& p : hal.packets) {
        apids.push_back((static_cast<unsigned>(p[0] & 0x07) << 8) | p[1]);
    }
    CHECK(apids.size() == 8);
    CHECK(apids[0] == 0x100 && apids[1] == 0x101 && apids[2] == 0x110 && apids[3] == 0x120 && apids[4] == 0x130);
    CHECK(apids[5] == 0x150);
    CHECK(apids[6] == 0x160 && apids[7] == 0x161);

    hal.packets.clear();
    hal.time = 0.02;
    hal.imu.stamp = hal.time;
    feedGnssAtTarget(hal, config);
    computer.tick();  // cycle 1: only the new GNSS solution
    CHECK(hal.packets.size() == 1);
    CHECK(hal.packets[0].size() == 68);
}
