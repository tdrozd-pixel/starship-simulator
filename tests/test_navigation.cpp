// Geodesy (sim and flight software implementations), the GNSS receiver model and
// the horizontal navigation filter.

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>

#include "TestFramework.hpp"
#include "fsw/Geodesy.hpp"
#include "fsw/HopFlightComputer.hpp"
#include "fsw/HorizontalEstimator.hpp"
#include "sim/core/Simulation.hpp"
#include "sim/env/Geodesy.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/Sensors.hpp"

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

const sim::env::Geodetic kPad{25.996603 * kDeg, -97.158251 * kDeg, -22.0};

}  // namespace

TEST_CASE(geodesy_reference_points) {
    const Eigen::Vector3d equator = sim::env::geodeticToEcef({0.0, 0.0, 0.0});
    CHECK_NEAR(equator.x(), sim::env::kWgs84A, 1e-6);
    CHECK_NEAR(equator.y(), 0.0, 1e-6);
    const Eigen::Vector3d pole = sim::env::geodeticToEcef({std::numbers::pi / 2.0, 0.0, 0.0});
    CHECK_NEAR(pole.z(), sim::env::kWgs84B, 1e-6);
    CHECK_NEAR(pole.x(), 0.0, 1e-6);
}

TEST_CASE(geodesy_round_trip_is_submillimeter) {
    const sim::env::LocalTangentFrame site(kPad);
    const Eigen::Vector3d points[] = {{0.0, 0.0, 0.0}, {-30.0, -20.0, 50.0}, {1200.0, -800.0, 3000.0},
                                      {50000.0, 20000.0, 100000.0}};
    for (const Eigen::Vector3d& enu : points) {
        const sim::env::Geodetic g = site.enuToGeodetic(enu);
        CHECK((site.geodeticToEnu(g) - enu).norm() < 1e-4);
    }
    // The origin maps to the site coordinates.
    const sim::env::Geodetic o = site.enuToGeodetic(Eigen::Vector3d::Zero());
    CHECK_NEAR(o.latitude, kPad.latitude, 1e-12);
    CHECK_NEAR(o.longitude, kPad.longitude, 1e-12);
    CHECK_NEAR(o.height, kPad.height, 1e-6);
    // 1 m north is about 1 / 110.8 km of latitude at 26 N.
    const sim::env::Geodetic n = site.enuToGeodetic(Eigen::Vector3d(0.0, 1000.0, 0.0));
    CHECK_NEAR((n.latitude - kPad.latitude) / kDeg, 1000.0 / 110800.0, 2e-5);
}

TEST_CASE(flight_software_geodesy_matches_sim) {
    // Two independent implementations must agree: the sim makes coordinates, the
    // flight software reads them.
    const sim::env::LocalTangentFrame site(kPad);
    const fsw::LocalEnuFrame frame({kPad.latitude, kPad.longitude, kPad.height});
    const Eigen::Vector3d points[] = {{-30.0, -20.0, 0.0}, {12.5, 3.25, 50.0}, {4000.0, -2500.0, 10000.0}};
    for (const Eigen::Vector3d& enu : points) {
        const sim::env::Geodetic g = site.enuToGeodetic(enu);
        double out[3];
        frame.toEnu({g.latitude, g.longitude, g.height}, out);
        CHECK((Eigen::Vector3d(out[0], out[1], out[2]) - enu).norm() < 1e-4);
    }
}

TEST_CASE(horizontal_estimator_tracks_constant_velocity) {
    fsw::HorizontalEstimator est({});
    est.initialize(0.0, 0.0, 1.0);
    const double dt = 0.01;
    for (int i = 1; i <= 3000; ++i) {
        const double t = i * dt;
        est.predict(0.0, 0.0, dt);
        if (i % 10 == 0) {
            est.correctPosition(2.0 * t, -1.0 * t, 0.02);
        }
    }
    CHECK_NEAR(est.velocityEast(), 2.0, 0.02);
    CHECK_NEAR(est.velocityNorth(), -1.0, 0.02);
    CHECK_NEAR(est.east(), 60.0, 0.05);
    CHECK(est.positionSigma(0) < 0.05);
}

TEST_CASE(gnss_model_reports_antenna_position_with_latency) {
    sim::Simulation sim({1000.0, 1});
    const sim::vehicle::HopperParams params;
    const sim::env::LocalTangentFrame site(kPad);
    auto& vehicle = sim.emplaceSystem<sim::vehicle::HopperDynamics>(
        params, sim::vehicle::HopperDynamics::stateOnPad(params, Eigen::Vector3d(-30.0, -20.0, 0.0),
                                                         Eigen::Quaterniond::Identity()));
    const Eigen::Vector3d antenna(0.0, 4.5, 46.0);
    sim::vehicle::GnssSpec spec;
    spec.horizontalBiasSigma = spec.verticalBiasSigma = 0.0;
    spec.horizontalNoiseSigma = spec.verticalNoiseSigma = spec.velocityNoiseSigma = 0.0;
    sim.scheduler().emplace<sim::vehicle::GnssModel>("gnss", 100.0, vehicle, site, antenna, spec, 7);
    sim.initialize();
    const auto& channel = sim.bus().channel<sim::vehicle::GnssMsg>(sim::vehicle::channels::kGnss);

    int outputs = 0;
    uint64_t lastSeq = 0;
    for (int i = 0; i < 1000; ++i) {
        sim.step();
        const auto& m = channel.latest();
        if (m.seq != lastSeq) {
            lastSeq = m.seq;
            ++outputs;
            CHECK_NEAR(m.stamp - m.data.fixTime, spec.latency, 1e-9);
            CHECK(m.data.valid);
            CHECK(m.data.fixType == 2);
        }
    }
    CHECK(outputs >= 9 && outputs <= 10);  // 10 Hz for 1 s, the first one delayed by latency

    const auto& m = channel.latest().data;
    const Eigen::Vector3d enu = site.geodeticToEnu({m.latitude, m.longitude, m.height});
    CHECK((enu - Eigen::Vector3d(-30.0, -15.5, 46.0)).norm() < 0.005);  // quantization only
}

// ---------------------------------------------------------------------------------------------
// Delayed GNSS fusion in HorizontalEstimator: scripted truth, exact IMU, noise-free fixes that
// arrive `latency` after their time of validity.
// ---------------------------------------------------------------------------------------------

namespace {

struct Track {
    std::function<double(double)> position;  // [m]
    std::function<double(double)> velocity;  // [m/s]
    std::function<double(double)> accel;     // [m/s^2]
};

struct TrackResult {
    double meanPositionError = 0.0;  // [m] estimate minus truth, mean over t > 10 s
    double maxPositionError = 0.0;   // [m]
    double meanVelocityError = 0.0;  // [m/s]
    double maxVelocityError = 0.0;   // [m/s]
    fsw::HorizontalEstimator::DelayedResult lastResult = fsw::HorizontalEstimator::DelayedResult::Applied;
};

// 100 Hz prediction, 10 Hz fixes. `legacy` fuses the way the flight software did before the
// delayed update existed: the position was moved to now with the current velocity estimate and
// the (stale) Doppler velocity was applied as if it were current.
TrackResult flyTrack(const Track& track, double latency, bool legacy, double duration = 30.0) {
    constexpr double kDt = 0.01;  // [s]
    fsw::HorizontalEstimator est({});
    est.initialize(track.position(0.0), 0.0, 0.02, 0.0);
    TrackResult r;
    int n = 0;
    const int steps = static_cast<int>(duration / kDt);
    for (int i = 1; i <= steps; ++i) {
        const double t = i * kDt;
        est.predict(track.accel(t), 0.0, kDt, t);
        const double tm = t - latency;  // time of validity of the fix arriving now
        if (i % 10 == 0 && tm >= 0.0) {
            if (legacy) {
                est.correctPosition(track.position(tm) + est.velocityEast() * latency, 0.0, 0.015);
                est.correctVelocity(track.velocity(tm), 0.0, 0.05);
            } else {
                r.lastResult = est.correctDelayed(t, tm, track.position(tm), 0.0, 0.015, track.velocity(tm), 0.0, 0.05);
            }
        }
        if (t > 10.0) {
            const double ep = est.east() - track.position(t);
            const double ev = est.velocityEast() - track.velocity(t);
            r.meanPositionError += ep;
            r.meanVelocityError += ev;
            r.maxPositionError = std::max(r.maxPositionError, std::abs(ep));
            r.maxVelocityError = std::max(r.maxVelocityError, std::abs(ev));
            ++n;
        }
    }
    r.meanPositionError /= n;
    r.meanVelocityError /= n;
    return r;
}

const Track kConstantVelocity{[](double t) { return 5.0 + 3.0 * t; }, [](double) { return 3.0; },
                              [](double) { return 0.0; }};
const Track kConstantAcceleration{[](double t) { return 5.0 + 0.5 * t * t; }, [](double t) { return t; },
                                  [](double) { return 1.0; }};

}  // namespace

// Constant velocity and constant acceleration with a 50 ms fix delay: no steady bias. Measured
// before the delayed update existed (the `legacy` branch, same scenario): with constant velocity the
// old position extrapolation was already right (error under 0.1 mm), but with 1 m/s^2 the stale
// Doppler velocity left a steady 7 mm/s velocity and 7 mm position error (44 mm/s at a 0.3 s
// delay). Applying the update at the state of its own time removes it.
TEST_CASE(delayed_fusion_has_no_bias_on_constant_velocity_and_acceleration) {
    for (const Track* track : {&kConstantVelocity, &kConstantAcceleration}) {
        const TrackResult r = flyTrack(*track, 0.05, false);
        CHECK(r.lastResult == fsw::HorizontalEstimator::DelayedResult::Applied);
        CHECK(std::abs(r.meanPositionError) < 1e-5);
        CHECK(r.maxPositionError < 1e-5);
        CHECK(r.maxVelocityError < 1e-5);
    }
    const TrackResult legacy = flyTrack(kConstantAcceleration, 0.05, true);
    CHECK(legacy.maxVelocityError > 5e-3);  // documents the old behaviour
    CHECK(legacy.maxPositionError > 5e-3);
}

// A fix time that is not a multiple of the 10 ms step (53 ms) is moved to the stored time with the
// measured velocity and the step's acceleration: still no bias.
TEST_CASE(delayed_fusion_handles_a_latency_between_steps) {
    for (const Track* track : {&kConstantVelocity, &kConstantAcceleration}) {
        const TrackResult r = flyTrack(*track, 0.053, false);
        CHECK(r.maxPositionError < 1e-5);
        CHECK(r.maxVelocityError < 1e-5);
    }
}

// A sudden velocity change (acceleration pulse of 20 m/s^2 for 0.1 s at t = 12 s): the fixes that
// describe the time before the pulse but arrive after it must not drag the estimate back.
// The filter uses the IMU through the pulse, so the error stays at the 1 cm level the 5 ms
// hold of the sampled acceleration allows.
TEST_CASE(delayed_fusion_survives_a_sudden_velocity_change) {
    const auto pulseAccel = [](double t) { return (t > 12.0 && t <= 12.1) ? 20.0 : 0.0; };
    const auto pulseVelocity = [](double t) { return std::clamp(t - 12.0, 0.0, 0.1) * 20.0; };
    const auto pulsePosition = [](double t) {
        const double u = std::clamp(t - 12.0, 0.0, 0.1);
        return 10.0 * u * u + 2.0 * std::max(0.0, t - 12.1);
    };
    const Track pulse{pulsePosition, pulseVelocity, pulseAccel};
    const TrackResult r = flyTrack(pulse, 0.05, false);
    CHECK(r.maxPositionError < 0.02);
    CHECK(r.maxVelocityError < 0.05);
    // Final state converged: no residual bias after the pulse.
    CHECK(std::abs(r.meanVelocityError) < 5e-3);
}

// Latency 0 reduces to the immediate update bit for bit.
TEST_CASE(delayed_fusion_with_zero_latency_equals_the_immediate_update) {
    fsw::HorizontalEstimator immediate({});
    fsw::HorizontalEstimator delayed({});
    immediate.initialize(1.0, -2.0, 0.5, 0.0);
    delayed.initialize(1.0, -2.0, 0.5, 0.0);
    for (int i = 1; i <= 500; ++i) {
        const double t = 0.01 * i;
        const double ax = 0.3 * std::sin(t);
        immediate.predict(ax, -ax, 0.01, t);
        delayed.predict(ax, -ax, 0.01, t);
        if (i % 10 == 0) {
            immediate.correctPosition(1.0 + 2.0 * t, -2.0 - t, 0.02);
            immediate.correctVelocity(2.0, -1.0, 0.05);
            delayed.correctDelayed(t, t, 1.0 + 2.0 * t, -2.0 - t, 0.02, 2.0, -1.0, 0.05);
        }
    }
    CHECK(immediate.east() == delayed.east());
    CHECK(immediate.north() == delayed.north());
    CHECK(immediate.velocityEast() == delayed.velocityEast());
    CHECK(immediate.velocityNorth() == delayed.velocityNorth());
    CHECK(immediate.accelBiasEast() == delayed.accelBiasEast());
    CHECK(immediate.positionSigma(0) == delayed.positionSigma(0));
    CHECK(immediate.positionInnovation(0) == delayed.positionInnovation(0));
}

// The innovation compares the fix with the prediction AT THE MEASUREMENT TIME: a constant
// acceleration track with exact fixes gives zero innovation even at 50 ms delay.
TEST_CASE(delayed_fusion_innovation_is_measured_at_the_measurement_time) {
    fsw::HorizontalEstimator est({});
    est.initialize(5.0, 0.0, 0.02, 0.0);
    double worst = 0.0;
    for (int i = 1; i <= 2000; ++i) {
        const double t = 0.01 * i;
        est.predict(1.0, 0.0, 0.01, t);
        const double tm = t - 0.05;
        if (i % 10 == 0 && tm >= 0.0) {
            est.correctDelayed(t, tm, 5.0 + 0.5 * tm * tm, 0.0, 0.015, tm, 0.0, 0.05);
            if (t > 5.0) {
                worst = std::max(worst, std::abs(est.positionInnovation(0)));
            }
        }
    }
    CHECK(worst < 1e-6);
}

// Out-of-order, repeated, future-stamped and too-old fixes are dropped and counted; the state
// is not touched by them.
TEST_CASE(delayed_fusion_drops_stale_and_out_of_order_stamps) {
    using Result = fsw::HorizontalEstimator::DelayedResult;
    fsw::HorizontalEstimator est({});
    est.initialize(0.0, 0.0, 0.02, 0.0);
    for (int i = 1; i <= 200; ++i) {
        est.predict(0.0, 0.0, 0.01, 0.01 * i);  // stationary, now = 2.0 s
    }
    CHECK(est.correctDelayed(2.0, 1.95, 0.0, 0.0, 0.015, 0.0, 0.0, 0.05) == Result::Applied);

    const double east = est.east();
    const double velocity = est.velocityEast();
    const double sigma = est.positionSigma(0);
    // Same stamp again, and an older one than the last applied: out of order.
    CHECK(est.correctDelayed(2.0, 1.95, 9.0, 9.0, 0.015, 9.0, 9.0, 0.05) == Result::OutOfOrder);
    CHECK(est.correctDelayed(2.0, 1.90, 9.0, 9.0, 0.015, 9.0, 9.0, 0.05) == Result::OutOfOrder);
    // Stamped in the future: out of order too.
    CHECK(est.correctDelayed(2.0, 2.5, 9.0, 9.0, 0.015, 9.0, 9.0, 0.05) == Result::OutOfOrder);
    CHECK(est.outOfOrderDrops() == 3);
    CHECK(est.staleDrops() == 0);

    // Older than the history (64 steps = 0.64 s): stale. Run on so the oldest entry is 1.37 s.
    fsw::HorizontalEstimator old({});
    old.initialize(0.0, 0.0, 0.02, 0.0);
    for (int i = 1; i <= 200; ++i) {
        old.predict(0.0, 0.0, 0.01, 0.01 * i);
    }
    CHECK(old.correctDelayed(2.0, 1.0, 9.0, 9.0, 0.015, 9.0, 9.0, 0.05) == Result::Stale);
    CHECK(old.staleDrops() == 1);
    CHECK(old.east() == 0.0);

    CHECK(est.east() == east);
    CHECK(est.velocityEast() == velocity);
    CHECK(est.positionSigma(0) == sigma);
}

// Stable with the longest delay the history holds, and every fix older than that is dropped (counted) without
// disturbing the filter.
TEST_CASE(delayed_fusion_is_stable_up_to_the_history_length) {
    const double longest = 0.01 * (fsw::HorizontalEstimator::kHistoryCapacity - 3);
    for (const Track* track : {&kConstantVelocity, &kConstantAcceleration}) {
        const TrackResult r = flyTrack(*track, longest, false);
        CHECK(r.lastResult == fsw::HorizontalEstimator::DelayedResult::Applied);
        CHECK(r.maxPositionError < 1e-4);
        CHECK(r.maxVelocityError < 1e-4);
    }
    const TrackResult beyond = flyTrack(kConstantAcceleration, 0.01 * fsw::HorizontalEstimator::kHistoryCapacity, false);
    CHECK(beyond.lastResult == fsw::HorizontalEstimator::DelayedResult::Stale);
    CHECK(std::isfinite(beyond.maxPositionError));
}

// A zero-velocity update recorded in the history is replayed: a delayed fix on the pad does not
// undo the knowledge that the vehicle is not moving.
TEST_CASE(delayed_fusion_replays_zero_velocity_updates) {
    fsw::HorizontalEstimator est({});
    est.initialize(0.0, 0.0, 0.02, 0.0);
    for (int i = 1; i <= 300; ++i) {
        const double t = 0.01 * i;
        est.predict(0.2, 0.0, 0.01, t);  // an apparent acceleration (e.g. tilt error) the filter would integrate
        est.zeroVelocityUpdate();
        if (i % 10 == 0) {
            est.correctDelayed(t, t - 0.05, 0.0, 0.0, 0.015, 0.0, 0.0, 0.05);
        }
    }
    CHECK(std::abs(est.velocityEast()) < 0.01);
    CHECK(std::abs(est.east()) < 0.05);
}

// removeBias moves the stored history too: a delayed update after a bias removal replays the
// same true acceleration, so the position stays continuous (no jump when the fix is applied).
TEST_CASE(delayed_fusion_keeps_history_consistent_after_bias_removal) {
    fsw::HorizontalEstimator est({});
    est.initialize(0.0, 0.0, 0.02, 0.0);
    double eastBefore = 0.0;
    for (int i = 1; i <= 100; ++i) {
        est.predict(0.0, 0.0, 0.01, 0.01 * i);
    }
    eastBefore = est.east();
    est.removeBias(0.05, 0.0);  // the closed-loop attitude correction
    // Predictions that follow carry accel 0.05 less and the bias 0.05 less: unchanged net motion.
    CHECK_NEAR(est.east(), eastBefore, 1e-12);
    est.correctDelayed(1.0, 0.95, 0.0, 0.0, 0.015, 0.0, 0.0, 0.05);
    CHECK(std::abs(est.east()) < 1e-3);
    CHECK(std::abs(est.velocityEast()) < 1e-3);
}

// Lever arm with rotation. The vehicle pivots about the IMU (it stays at a fixed point) while the
// antenna, 16 m above it, swings with it. Fixes are valid 50 ms before they arrive; the flight
// software must take the attitude and rate of the fix time. With the current attitude (the
// behaviour before this fix) the lever error was 16 m x 0.157 rad/s x 0.05 s = 0.13 m in position at the worst
// moment; with the attitude at the time of validity it stays at the 2 cm of the prelaunch
// attitude estimate (the gravity alignment leads the gyro attitude by one step).
TEST_CASE(gnss_lever_arm_uses_the_attitude_at_the_time_of_validity) {
    constexpr double kLever = 16.0;       // [m] antenna above the IMU
    constexpr double kAmplitude = 0.05;   // [rad] pivot angle amplitude
    constexpr double kOmega = 2.0 * std::numbers::pi * 0.5;  // [rad/s] 0.5 Hz
    constexpr double kLatency = 0.05;     // [s]
    constexpr double kG = 9.80665;        // [m/s^2]

    fsw::HopConfig config;
    config.target = {0.4537, -1.6957, -22.0};
    config.gnssAntennaPosition[2] = 43.0;
    config.imuPosition[2] = 27.0;
    config.gnssFixesRequired = 1000000;  // stay in Prelaunch: only navigation is under test
    config.gnssLatency = kLatency;
    const fsw::LocalEnuFrame frame(config.target);

    // Scripted Hal: pivot about body x, IMU at rest.
    struct PivotHal final : fsw::Hal {
        double time = 0.0;
        fsw::ImuSample imu;
        fsw::GnssSample gnss;
        double timeSec() const override { return time; }
        fsw::ImuSample readImu() override { return imu; }
        fsw::AltimeterSample readAltimeter() override { return {}; }
        fsw::GnssSample readGnss() override { return gnss; }
        fsw::GimbalFeedbackSample readGimbalFeedback() override { return {}; }
        fsw::EngineFeedbackSample readEngineFeedback() override { return {}; }
        void commandActuators(const fsw::ActuatorCommand&) override {}
        void publishTelemetry(const fsw::Telemetry&) override {}
        void downlink(const uint8_t*, std::size_t) override {}
    } hal;

    fsw::HopFlightComputer computer(hal, config);
    const auto angle = [&](double t) { return kAmplitude * std::sin(kOmega * t); };
    const auto rate = [&](double t) { return kAmplitude * kOmega * std::cos(kOmega * t); };
    double worst = 0.0;
    for (int i = 1; i <= 800; ++i) {
        const double t = 0.01 * i;
        hal.time = t;
        const double phi = angle(t);
        hal.imu.valid = true;
        hal.imu.stamp = t;
        // IMU at rest: specific force is +g up in the world; in the body frame that is R^T (0, 0, g).
        hal.imu.specificForce[0] = 0.0;
        hal.imu.specificForce[1] = kG * std::sin(phi);
        hal.imu.specificForce[2] = kG * std::cos(phi);
        hal.imu.angularRate[0] = rate(t);
        if (i % 10 == 0) {
            const double tm = t - kLatency;
            const double phiM = angle(tm);
            // Antenna world offset from the IMU (rotation about x, lever along body z): (0, -L sin, L cos).
            const double offset[3] = {0.0, -kLever * std::sin(phiM), kLever * std::cos(phiM)};
            // Geodetic position of that ENU point by Newton iteration on the flight software's own frame.
            fsw::GeodeticPosition g = config.target;
            const double target[3] = {offset[0], offset[1] + 0.0, offset[2] + 27.0};
            for (int k = 0; k < 4; ++k) {
                double enu[3];
                frame.toEnu(g, enu);
                g.latitude += (target[1] - enu[1]) / 6.378e6;
                g.longitude += (target[0] - enu[0]) / (6.378e6 * std::cos(g.latitude));
                g.height += target[2] - enu[2];
            }
            hal.gnss.valid = true;
            hal.gnss.stamp = t;
            hal.gnss.fixTime = tm;
            hal.gnss.fixType = fsw::GnssFixType::RtkFixed;
            hal.gnss.latitude = g.latitude;
            hal.gnss.longitude = g.longitude;
            hal.gnss.height = g.height;
            hal.gnss.velocityEnu[1] = -kLever * std::cos(phiM) * rate(tm);
            hal.gnss.velocityEnu[2] = -kLever * std::sin(phiM) * rate(tm);
            hal.gnss.horizontalSigma = 0.02;
        }
        computer.tick();
        if (t > 3.0 && i % 10 == 0) {
            worst = std::max({worst, std::abs(computer.horizontal().positionInnovation(0)),
                              std::abs(computer.horizontal().positionInnovation(1))});
        }
    }
    CHECK(computer.horizontal().initialized());
    CHECK(worst < 0.04);
    CHECK(computer.horizontal().staleDrops() == 0);
    CHECK(computer.horizontal().outOfOrderDrops() == 0);
}

// The flight software is configured for the receiver latency by value (it must not read the sensor
// model): this keeps the configured number and the receiver model's default in step.
TEST_CASE(configured_gnss_latency_matches_the_receiver_model) {
    CHECK(fsw::HopConfig{}.gnssLatency == sim::vehicle::GnssSpec{}.latency);
    const sim::vehicle::HopScenarioOptions options;
    CHECK(options.hop.gnssLatency == options.gnss.latency);
}
