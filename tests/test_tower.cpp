// Catch tower tests: tower physics (Stage A adds the TowerModel tests; see the catch plan).
// Stage 0 only checks the header-only geometry helpers.
#include "TestFramework.hpp"
#include "sim/vehicle/TowerParams.hpp"

TEST_CASE(tower_params_seat_geometry) {
    sim::vehicle::HopperParams vehicle;
    sim::vehicle::TowerParams tower;
    CHECK(tower.validate() == nullptr);
    const double psi = tower.psiClosed(vehicle.catchPinOffset);
    const auto pad = tower.padCenterTower(psi);
    CHECK_NEAR(pad.lateral, vehicle.catchPinOffset, 1e-9);
    const auto seat = tower.seatPointEnu(vehicle);
    CHECK_NEAR(seat.north, 0.0, 1e-9);
    CHECK_NEAR(seat.east, 42.04, 0.02);
    CHECK_NEAR(seat.baseAltitude, 45.25, 1e-9);
    sim::vehicle::TowerParams skew;
    skew.approachSide[0] = -0.8;
    skew.approachSide[1] = -0.6;
    CHECK(skew.validate() != nullptr);
}

// ---- Stage A: TowerModel, hold and the truth snapshot fields ----------------------------------
#include <algorithm>
#include <cmath>
#include <vector>

#include "sim/core/Simulation.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/Tower.hpp"
#include "sim/vehicle/TruthSnapshotPublisher.hpp"

namespace {

using namespace sim::vehicle;

constexpr double kDt = 0.001;  // [s]

// A scripted vehicle (no integration) and the tower model driven directly.
struct Rig {
    HopperParams v;
    TowerParams t;
    TowerParams::SeatPoint seat;
    HopperDynamics vehicle;
    TowerModel model;
    uint64_t step = 0;

    Rig() : seat(t.seatPointEnu(v)), vehicle(v, stateAt(0.0, 0.0, 1.0, 0.0, -0.3)), model("tower", 1000.0, vehicle, t) {}

    // Base position relative to the seat point (east, north, height above seated altitude) and vertical speed.
    HopperState stateAt(double dEast, double dNorth, double dUp, double vx, double vz) const {
        const TowerParams::SeatPoint s = t.seatPointEnu(v);
        HopperState x = HopperDynamics::stateOnPad(
            v, Eigen::Vector3d(s.east + dEast, s.north + dNorth, s.baseAltitude + dUp), Eigen::Quaterniond::Identity());
        x.velocity = Eigen::Vector3d(vx, 0.0, vz);
        return x;
    }
    void place(double dEast, double dNorth, double dUp, double vz) { vehicle.setState(stateAt(dEast, dNorth, dUp, 0.0, vz)); }
    void tick() {
        model.update(sim::StepContext{static_cast<double>(step) * kDt, kDt, step});
        ++step;
    }
    // Hover 1 m above the seat until the arms are Closed.
    void closeArms() {
        place(0.0, 0.0, 1.0, -0.3);
        for (int i = 0; i < 12000 && model.state() != TowerState::Closed; ++i) {
            tick();
        }
    }
};

class VectorSink final : public TruthSnapshotSink {
public:
    void push(const TruthSnapshot& s) noexcept override { snapshots.push_back(s); }
    std::vector<TruthSnapshot> snapshots;
};

}  // namespace

TEST_CASE(tower_seated_pins_sit_on_the_cradles) {
    Rig rig;
    rig.place(0.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d p0 = rig.vehicle.pinWorld(0);
    const Eigen::Vector3d p1 = rig.vehicle.pinWorld(1);
    CHECK_NEAR(std::abs(p0.y() - rig.seat.north), rig.v.catchPinOffset, 1e-9);
    CHECK_NEAR(std::abs(p1.y() - rig.seat.north), rig.v.catchPinOffset, 1e-9);
    CHECK_NEAR(p0.x(), rig.seat.east, 1e-9);
    CHECK_NEAR(p0.z(), rig.t.railHeight + rig.v.catchPinRadius, 1e-9);
    // The pad centers of both closed arms are exactly under the pins.
    const auto pad = rig.t.padCenterTower(rig.model.closedAngle());
    CHECK_NEAR(pad.lateral, rig.v.catchPinOffset, 1e-9);
    CHECK_NEAR(rig.t.position[0] - pad.along, p0.x(), 1e-9);
}

TEST_CASE(tower_servo_is_rate_limited) {
    Rig rig;
    rig.place(0.0, 0.0, 1.0, -0.3);
    double previous = 0.0;
    double maxStep = 0.0;
    double closedAt = -1.0;
    for (int i = 0; i < 12000; ++i) {
        rig.tick();
        const double psi = rig.model.armAngle(0);
        maxStep = std::max(maxStep, std::abs(psi - previous));
        previous = psi;
        CHECK(psi == rig.model.armAngle(1));
        if (closedAt < 0.0 && rig.model.state() == TowerState::Closed) {
            closedAt = i * kDt;
        }
    }
    CHECK(maxStep <= rig.t.servoRateLimit * kDt * (1.0 + 1e-12));
    CHECK(maxStep > 0.99 * rig.t.servoRateLimit * kDt);  // it does saturate
    CHECK(closedAt > 5.0 && closedAt < 7.5);             // about 4.8 s of rate-limited travel plus the exponential tail
    CHECK_NEAR(rig.model.armAngle(0), rig.model.closedAngle(), rig.t.closedTolerance);
}

TEST_CASE(tower_catches_pins_descending_onto_closed_arms) {
    Rig rig;
    rig.closeArms();
    CHECK(rig.model.state() == TowerState::Closed);
    rig.place(0.0, 0.0, -0.05, -0.3);
    rig.tick();
    CHECK(rig.model.result() == CatchResult::Caught);
    CHECK(rig.model.state() == TowerState::Holding);
    CHECK(rig.vehicle.constraint() == Constraint::HeldByArms);
    CHECK(rig.vehicle.state().velocity.norm() == 0.0);
    // Seated: the pins sit on the cradle tops.
    CHECK_NEAR(rig.vehicle.pinWorld(0).z(), rig.t.railHeight + rig.v.catchPinRadius, 1e-9);
}

TEST_CASE(tower_arm_strike_when_offset_along_the_arm) {
    Rig rig;
    rig.closeArms();
    rig.place(2.0, 0.0, -0.05, -0.3);  // 2 m along: on the arm body, outside the 1.5 m cradle
    rig.tick();
    CHECK(rig.model.result() == CatchResult::ArmStrike);
    CHECK(rig.vehicle.constraint() == Constraint::Crashed);
}

TEST_CASE(tower_arm_strike_when_offset_across_the_arm) {
    Rig rig;
    rig.closeArms();
    rig.place(0.0, 1.0, -0.05, -0.3);  // 1 m lateral: on the arm body (half width 1.0 + pin), off the cradle (0.8)
    rig.tick();
    CHECK(rig.model.result() == CatchResult::ArmStrike);
}

TEST_CASE(tower_hull_strikes_closed_arms_only_when_off_by_more_than_the_gap) {
    // The hull (radius 4.5 m) passes between the closed arms with about 1.2 m to spare. Pins 3 m above the arm
    // top, so only the hull is at the arm height.
    Rig ok;
    ok.closeArms();
    ok.place(0.0, 0.9, 3.0, -0.3);
    for (int i = 0; i < 10; ++i) {
        ok.tick();
    }
    CHECK(ok.model.result() == CatchResult::None);
    CHECK(ok.vehicle.constraint() == Constraint::Free);

    Rig bad;
    bad.closeArms();
    bad.place(0.0, 1.6, 3.0, -0.3);
    bad.tick();
    CHECK(bad.model.result() == CatchResult::ArmStrike);
    CHECK(bad.vehicle.constraint() == Constraint::Crashed);
}

TEST_CASE(closed_arms_clear_the_hull_and_open_arms_clear_it_more) {
    const HopperParams v;
    const TowerParams t;
    const double closedGap = t.closedArmClearance(v.catchPinOffset, v.radius);
    CHECK(closedGap >= 0.5);  // requirement: the closed arms clear the hull by at least 0.5 m
    // The old 5 m pin offset put the arm tips inside the hull.
    CHECK(t.closedArmClearance(5.0, v.radius) < 0.0);
    // Open arms (psi = 0): every corner of the box is farther out than the closed arms.
    const double openPhi = t.phi(0.0);
    double nearest = 1e9;
    for (const double s : {0.0, t.armLength}) {
        for (const double side : {-1.0, 1.0}) {
            nearest = std::min(nearest, t.hingeHalfSpacing - s * std::sin(openPhi) + side * t.armHalfWidth * std::cos(openPhi));
        }
    }
    CHECK(nearest - v.radius > closedGap);
    CHECK(nearest - v.radius > 2.0);
    // Same numbers through the pure box test: a hull disc at the centerline at the arm tip row.
    const double psiClosed = t.psiClosed(v.catchPinOffset);
    const TowerParams::TowerPoint tip{t.hingeSetback + t.armLength * std::cos(t.phi(psiClosed)), 0.0};
    CHECK(!hullTouchesArm(t, v.radius, psiClosed, 1.0, tip.along, 0.0));
    CHECK(hullTouchesArm(t, v.radius, psiClosed, 1.0, tip.along, closedGap + 0.1));
    CHECK(!hullTouchesArm(t, v.radius, psiClosed, 1.0, tip.along, closedGap - 0.1));
}

TEST_CASE(close_trigger_is_height_or_time_to_seat_based) {
    const TowerParams t;
    // Over the seat and descending: closes below the trigger height ...
    CHECK(shouldCloseArms(t, 0.0, t.closeTriggerHeight - 1.0, 5.0));
    // ... and earlier when the predicted time to the seat (2 * height / sink) is within the lead time.
    const double height = 80.0;
    CHECK(!shouldCloseArms(t, 0.0, height, 10.0));                                  // 16 s away
    CHECK(shouldCloseArms(t, 0.0, height, 2.0 * height / t.closeTriggerLeadTime + 0.5));  // under the lead time
    // Not when far to the side, rising or hovering, above the maximum height, or already below the seat.
    CHECK(!shouldCloseArms(t, t.closeTriggerRadius + 1.0, 10.0, 5.0));
    CHECK(!shouldCloseArms(t, 0.0, 10.0, -1.0));
    CHECK(!shouldCloseArms(t, 0.0, 10.0, 0.0));
    CHECK(!shouldCloseArms(t, 0.0, t.closeTriggerMaxHeight + 1.0, 100.0));
    CHECK(!shouldCloseArms(t, 0.0, -0.1, 1.0));
    // The closing time fits in the time left at the trigger height on a 1.5 m/s^2 profile with a 0.5 m/s contact sink rate.
    const double timeLeft = (std::sqrt(0.25 + 2.0 * 1.5 * t.closeTriggerHeight) - 0.5) / 1.5;
    CHECK(timeLeft > 6.0);
}

TEST_CASE(tower_open_arms_do_not_touch_the_pins) {
    Rig rig;
    rig.place(0.0, 0.0, -0.05, -0.3);  // pins at rail height between the open arms, below the close trigger band
    for (int i = 0; i < 100; ++i) {
        rig.tick();
    }
    CHECK(rig.model.state() == TowerState::Open);
    CHECK(rig.model.result() == CatchResult::None);
    rig.place(4.0, 0.0, -0.05, -0.3);  // pins past the pad, toward the mast
    rig.tick();
    CHECK(rig.model.result() == CatchResult::None);
    CHECK(rig.vehicle.constraint() == Constraint::Free);
}

TEST_CASE(tower_unclosed_arms_strike) {
    Rig rig;
    rig.place(0.0, 0.0, 1.0, -0.3);
    while (rig.model.armAngle(0) < 0.95 * rig.model.closedAngle()) {
        rig.tick();
    }
    CHECK(rig.model.armAngle(0) < 0.98 * rig.model.closedAngle());
    rig.place(0.0, 0.0, -0.05, -0.3);
    rig.tick();
    CHECK(rig.model.result() == CatchResult::ArmStrike);
}

TEST_CASE(tower_contact_window_rejects_each_violation) {
    const HopperParams v;
    const TowerParams t;
    ContactInput ok;
    for (int i = 0; i < 2; ++i) {
        ok.along[i] = t.cradleDistance;
        ok.across[i] = 0.0;
        ok.z[i] = t.railHeight + v.catchPinRadius - 0.05;
    }
    ok.closure = 1.0;
    ok.sinkRate = 0.3;
    CHECK(evaluateContact(t, v.catchPinRadius, ok) == CatchResult::Caught);
    ContactInput bad = ok;
    bad.sinkRate = 2.0;
    CHECK(evaluateContact(t, v.catchPinRadius, bad) == CatchResult::ArmStrike);
    bad = ok;
    bad.tilt = 5.0 * kDegToRad;
    CHECK(evaluateContact(t, v.catchPinRadius, bad) == CatchResult::ArmStrike);
    bad = ok;
    bad.horizontalSpeed = 0.6;
    CHECK(evaluateContact(t, v.catchPinRadius, bad) == CatchResult::ArmStrike);
    bad = ok;
    bad.across[1] = 0.75;  // inside the cradle, outside the 0.7 m window
    CHECK(evaluateContact(t, v.catchPinRadius, bad) == CatchResult::ArmStrike);
    bad = ok;
    bad.z[0] = bad.z[1] = t.railHeight - 5.0;  // below the arms
    CHECK(evaluateContact(t, v.catchPinRadius, bad) == CatchResult::None);
}

TEST_CASE(tower_mast_strike) {
    Rig rig;
    rig.place(rig.t.position[0] - 10.0 - rig.seat.east, -rig.seat.north, 0.0, 0.0);  // base 10 m west of the tower axis
    rig.tick();
    CHECK(rig.model.result() == CatchResult::TowerStrike);
    CHECK(rig.vehicle.constraint() == Constraint::Crashed);
    Rig clear;
    clear.place(-20.0, 0.0, 0.0, 0.0);
    clear.tick();
    CHECK(clear.model.result() != CatchResult::TowerStrike);
}

TEST_CASE(held_vehicle_does_not_move_and_halts) {
    const HopperParams v;
    const TowerParams t;
    const auto seat = t.seatPointEnu(v);
    sim::Simulation sim(sim::SimConfig{1000.0, 1});
    HopperState x = HopperDynamics::stateOnPad(v, Eigen::Vector3d(seat.east, seat.north, seat.baseAltitude),
                                               Eigen::Quaterniond(0.999, 0.01, 0.02, 0.0).normalized());
    x.velocity = Eigen::Vector3d(0.1, -0.2, -0.3);
    x.angularRate = Eigen::Vector3d(0.01, 0.0, 0.02);
    HopperDynamics& vehicle = sim.emplaceSystem<HopperDynamics>(v, x);
    sim.scheduler().emplace<TowerModel>("tower", 1000.0, vehicle, t);
    sim.initialize();
    vehicle.hold(Constraint::HeldByArms, Eigen::Vector3d(0.0, 0.0, 0.1));
    const HopperState held = vehicle.state();
    CHECK(held.velocity.norm() == 0.0);
    CHECK(held.angularRate.norm() == 0.0);
    for (int i = 0; i < 1000; ++i) {
        sim.step();
        CHECK(vehicle.state().velocity.norm() == 0.0);
        CHECK(vehicle.state().angularRate.norm() == 0.0);
    }
    CHECK((vehicle.state().position - held.position).norm() == 0.0);
    CHECK(vehicle.state().attitude.coeffs() == held.attitude.coeffs());
    CHECK(!sim.halted());  // 1 s held, engines off: not yet
    for (int i = 0; i < 1200 && !sim.halted(); ++i) {
        sim.step();
    }
    CHECK(sim.halted());
    CHECK(sim.time() < 2.2);
    CHECK(vehicle.state().position == held.position);
}

TEST_CASE(tower_flight_is_deterministic) {
    HopScenarioOptions options;
    options.seed = 7;
    auto a = buildHopScenario(options);
    auto b = buildHopScenario(options);
    for (int i = 0; i < 2000; ++i) {
        a.sim->step();
        b.sim->step();
    }
    CHECK(a.vehicle->state().position == b.vehicle->state().position);
    CHECK(a.vehicle->state().velocity == b.vehicle->state().velocity);
    CHECK(a.vehicle->state().attitude.coeffs() == b.vehicle->state().attitude.coeffs());
    CHECK(a.tower != nullptr);
    CHECK(a.tower->armAngle(0) == b.tower->armAngle(0));
    CHECK(a.tower->state() == b.tower->state());
}

TEST_CASE(tower_scenario_fills_catch_config) {
    HopScenarioOptions options;
    options.landing = fsw::LandingMode::Catch;
    const TowerParams::SeatPoint seat = options.tower.seatPointEnu(options.vehicle);
    // The config is private to the flight computer; check the sim side of the wiring through the tower.
    auto scenario = buildHopScenario(options);
    CHECK_NEAR(scenario.tower->seat().east, seat.east, 1e-12);
    CHECK_NEAR(scenario.tower->seat().baseAltitude, 45.25, 1e-9);
    options.tower.approachSide[0] = -0.8;
    options.tower.approachSide[1] = -0.6;
    CHECK_THROWS(buildHopScenario(options));
}

TEST_CASE(snapshot_carries_tower_fields) {
    Rig rig;
    VectorSink sink;
    TruthSnapshotPublisher bare("truth_snapshot", 100.0, rig.vehicle, sink);
    bare.update(sim::StepContext{0.0, kDt, 0});
    CHECK(sink.snapshots.back().armAngle[0] == 0.0);
    CHECK(sink.snapshots.back().towerState == 0u);
    CHECK(sink.snapshots.back().catchResult == 0u);

    rig.closeArms();
    TruthSnapshotPublisher pub("truth_snapshot", 100.0, rig.vehicle, sink);
    pub.setTower(&rig.model);
    pub.update(sim::StepContext{1.0, kDt, 1000});
    TruthSnapshot s = sink.snapshots.back();
    CHECK(s.towerState == static_cast<uint32_t>(TowerState::Closed));
    CHECK_NEAR(s.armAngle[0], rig.model.armAngle(0), 0.0);
    CHECK_NEAR(s.armAngle[1], rig.model.armAngle(1), 0.0);
    CHECK((s.flags & (kSnapCaught | kSnapCrashed)) == 0u);

    rig.place(0.0, 0.0, -0.05, -0.3);
    rig.tick();
    pub.update(sim::StepContext{1.1, kDt, 1100});
    s = sink.snapshots.back();
    CHECK(s.towerState == static_cast<uint32_t>(TowerState::Holding));
    CHECK(s.catchResult == static_cast<uint32_t>(CatchResult::Caught));
    CHECK((s.flags & kSnapCaught) != 0u);
    CHECK((s.flags & kSnapCrashed) == 0u);

    Rig crash;
    TruthSnapshotPublisher pub2("truth_snapshot", 100.0, crash.vehicle, sink);
    pub2.setTower(&crash.model);
    crash.closeArms();
    crash.place(2.0, 0.0, -0.05, -0.3);
    crash.tick();
    pub2.update(sim::StepContext{1.1, kDt, 1100});
    s = sink.snapshots.back();
    CHECK(s.catchResult == static_cast<uint32_t>(CatchResult::ArmStrike));
    CHECK((s.flags & kSnapCrashed) != 0u);
    CHECK((s.flags & kSnapCaught) == 0u);
}
