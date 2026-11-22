// Geodesy (sim and flight software implementations), the GNSS receiver model and
// the horizontal navigation filter.

#include <cmath>
#include <numbers>

#include "TestFramework.hpp"
#include "fsw/Geodesy.hpp"
#include "fsw/HorizontalEstimator.hpp"
#include "sim/core/Simulation.hpp"
#include "sim/env/Geodesy.hpp"
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
