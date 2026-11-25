#include <algorithm>
#include <cmath>
#include <numbers>

#include "TestFramework.hpp"
#include "fsw/EngineAllocator.hpp"
#include "sim/core/Bus.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace {

using namespace sim::vehicle;

// Hold a vehicle at a given base altitude (it is allowed to fall a few meters during the test)
// with the given engine commands and step the dynamics alone.
struct Rig {
    sim::Bus bus;
    HopperParams params;
    std::unique_ptr<HopperDynamics> vehicle;
    sim::Channel<ActuatorCommandMsg>* commands = nullptr;
    double t = 0.0;

    explicit Rig(double baseAltitude) {
        const HopperState initial = HopperDynamics::stateOnPad(params, Eigen::Vector3d(0.0, 0.0, baseAltitude),
                                                               Eigen::Quaterniond::Identity());
        vehicle = std::make_unique<HopperDynamics>(params, initial);
        vehicle->initialize(bus);
        commands = &bus.channel<ActuatorCommandMsg>(channels::kActuatorCmd);
    }

    void run(const ActuatorCommandMsg& command, double seconds) {
        commands->publish(command, t);
        const double dt = 0.001;
        const int steps = static_cast<int>(seconds / dt);
        for (int i = 0; i < steps; ++i) {
            vehicle->prepare(t);
            vehicle->integrate(t, dt);
            t += dt;
        }
    }
};

ActuatorCommandMsg rvacsOn(double throttle) {
    ActuatorCommandMsg c;
    for (int i = kFirstRvac; i < kEngineCount; ++i) {
        c.engines[i] = EngineCommandMsg{true, throttle};
    }
    return c;
}

}  // namespace

TEST_CASE(rvac_runs_at_vacuum_like_pressure_and_burns_propellant) {
    Rig rig(30000.0);  // ~1.2 kPa ambient, well below the flow separation limit
    const double propellant0 = rig.vehicle->state().propellantMass;
    rig.run(rvacsOn(1.0), 3.5);
    const HopperDynamics& v = *rig.vehicle;
    for (int i = 0; i < kFirstRvac; ++i) {
        CHECK(!v.engineRunning(i));
    }
    for (int i = kFirstRvac; i < kEngineCount; ++i) {
        CHECK(v.engineRunning(i));
        CHECK(v.engineState(i) == EngineState::Running);
        CHECK(!v.engineInhibited(i));
        CHECK_NEAR(v.state().throttle[i], 1.0, 0.01);
    }
    // Axial thrust: three engines at full flow minus the (small) ambient pressure term.
    const double expected = 3.0 * rig.params.rvacVacuumThrust / v.mass();  // [m/s^2], ambient term < 0.5%
    CHECK_NEAR(v.specificForceBody().z(), expected, 0.02 * expected);
    // Flow = F_vac / (Isp g0) per engine, for about 2.5 s at (ramping to) full throttle.
    const double flow = 3.0 * rig.params.rvacVacuumThrust / (rig.params.rvacVacuumIsp * 9.80665);  // [kg/s]
    const double burned = propellant0 - v.state().propellantMass;
    CHECK(burned > 1.5 * flow);
    CHECK(burned < 2.5 * flow);
}

TEST_CASE(rvac_refuses_to_run_at_sea_level) {
    Rig rig(100.0);  // sea-level air
    const double propellant0 = rig.vehicle->state().propellantMass;
    rig.run(rvacsOn(1.0), 3.0);
    const HopperDynamics& v = *rig.vehicle;
    for (int i = kFirstRvac; i < kEngineCount; ++i) {
        CHECK(!v.engineRunning(i));
        CHECK(v.engineInhibited(i));
        CHECK(v.engineState(i) == EngineState::Inhibited);
        CHECK(v.state().throttle[i] == 0.0);
    }
    CHECK(v.state().propellantMass == propellant0);
    // Free fall: an ideal accelerometer reads zero (drag at this speed is tiny).
    CHECK(std::abs(v.specificForceBody().z()) < 0.05);
}

TEST_CASE(rvac_limit_is_an_ambient_pressure_threshold) {
    Rig high(30000.0);
    high.run(rvacsOn(1.0), 2.5);
    CHECK(high.vehicle->engineRunning(kFirstRvac));
    Rig low(5000.0);  // ~54 kPa, above the limit
    low.run(rvacsOn(1.0), 2.5);
    CHECK(!low.vehicle->engineRunning(kFirstRvac));
    CHECK(low.vehicle->engineInhibited(kFirstRvac));
}

TEST_CASE(sea_level_engines_unaffected_by_rvac_commands_off) {
    // Two vehicles, identical but for RVac "off" commands being present: bit-identical state.
    ActuatorCommandMsg c;
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        c.engines[i] = EngineCommandMsg{true, 0.7};
        c.gimbals[i] = GimbalCommandMsg{0.01 * i, -0.02};
    }
    Rig a(50.0);
    Rig b(50.0);
    ActuatorCommandMsg withThrottleOnOffEngines = c;
    for (int i = kFirstRvac; i < kEngineCount; ++i) {
        withThrottleOnOffEngines.engines[i] = EngineCommandMsg{false, 1.0};  // off flag wins
    }
    a.run(c, 3.0);
    b.run(withThrottleOnOffEngines, 3.0);
    CHECK(a.vehicle->state().position == b.vehicle->state().position);
    CHECK(a.vehicle->state().propellantMass == b.vehicle->state().propellantMass);
    CHECK(a.vehicle->engineRunning(0));
}

TEST_CASE(rvac_ring_is_outside_the_sea_level_ring_and_offset_60_degrees) {
    const HopperParams p;
    CHECK(p.rvacClusterRadius > p.engineClusterRadius);
    const double offset = std::fmod(p.rvacAzimuth0 - p.engineAzimuth0 + 4.0 * std::numbers::pi, 2.0 * std::numbers::pi / 3.0);
    CHECK_NEAR(offset, std::numbers::pi / 3.0, 1e-9);
}

TEST_CASE(truth_snapshot_has_per_engine_actuals) {
    class Sink final : public TruthSnapshotSink {
    public:
        void push(const TruthSnapshot& s) noexcept override {
            if (s.simTime > 5.0 && !captured) {
                last = s;
                captured = (s.flags & kSnapEngineRunning) != 0;
            }
        }
        TruthSnapshot last;
        bool captured = false;
    } sink;
    HopScenarioOptions options;
    options.snapshotSink = &sink;
    HopScenario scenario = buildHopScenario(options);
    while (!sink.captured && scenario.sim->time() < 30.0) {
        scenario.sim->step();
    }
    CHECK(sink.captured);
    const TruthSnapshot& s = sink.last;
    double mean = 0.0;
    for (int i = 0; i < kSeaLevelEngineCount; ++i) {
        CHECK(s.engineRunning[i]);
        CHECK(s.engineThrottle[i] >= 0.39);
        mean += s.engineThrottle[i] / kSeaLevelEngineCount;
    }
    CHECK_NEAR(mean, s.throttle, 1e-12);  // the averaged field keeps its meaning
    for (int i = kFirstRvac; i < kEngineCount; ++i) {
        CHECK(!s.engineRunning[i]);
        CHECK(s.engineThrottle[i] == 0.0);
    }
}

namespace {

fsw::AllocatorConfig allocatorConfig() {
    fsw::AllocatorConfig c;
    c.leverArm = 9.5;
    c.ringRadius = 1.55;
    c.azimuth0 = 90.0 * std::numbers::pi / 180.0;
    c.gimbalLimit = 15.0 * std::numbers::pi / 180.0;
    c.rollGimbalBudget = 2.0 * std::numbers::pi / 180.0;
    c.vacuumThrust = 2.88e6;
    c.nozzleExitArea = 1.33;
    c.ambientPressure = 101325.0;
    c.minThrottle = 0.4;
    return c;
}

struct Achieved {
    double torque[3] = {};
    double totalThrust = 0.0;
    double axialThrust = 0.0;
};

// Exact torque about the CG from the commanded per-engine thrust and gimbal.
Achieved evaluate(const fsw::AllocatorConfig& c, const fsw::AllocatorResult& r) {
    Achieved a;
    for (int i = 0; i < fsw::kSeaLevelEngineCount; ++i) {
        const double az = c.azimuth0 + 2.0 * std::numbers::pi * i / fsw::kSeaLevelEngineCount;
        const double x = c.ringRadius * std::cos(az);
        const double y = c.ringRadius * std::sin(az);
        const double z = -c.leverArm;
        const double thrust = r.throttle[i] * c.vacuumThrust - c.ambientPressure * c.nozzleExitArea;
        const double dx = std::cos(r.pitch[i]) * std::sin(r.yaw[i]);
        const double dy = -std::sin(r.pitch[i]);
        const double dz = std::cos(r.pitch[i]) * std::cos(r.yaw[i]);
        const double fx = thrust * dx;
        const double fy = thrust * dy;
        const double fz = thrust * dz;
        a.torque[0] += y * fz - z * fy;
        a.torque[1] += z * fx - x * fz;
        a.torque[2] += x * fy - y * fx;
        a.totalThrust += thrust;
        a.axialThrust += fz;
    }
    return a;
}

double nominalTotalThrust(const fsw::AllocatorConfig& c, double throttle) {
    return 3.0 * (throttle * c.vacuumThrust - c.ambientPressure * c.nozzleExitArea);
}

}  // namespace

TEST_CASE(allocator_achieves_requested_torque_within_gimbal_range) {
    const fsw::AllocatorConfig c = allocatorConfig();
    fsw::AllocatorRequest req;
    req.throttle = 0.8;
    req.torque[0] = 2.0e6;
    req.torque[1] = -1.5e6;
    req.torque[2] = 1.0e5;
    const fsw::AllocatorResult r = fsw::allocateSeaLevelEngines(c, req);
    CHECK(!r.gimbalSaturated && !r.rollSaturated && !r.differentialThrottleUsed);
    const Achieved a = evaluate(c, r);
    for (int k = 0; k < 3; ++k) {
        CHECK_NEAR(a.torque[k], req.torque[k], 0.01 * std::abs(req.torque[k]));
        CHECK_NEAR(r.residualTorque[k], 0.0, 0.01 * std::abs(req.torque[k]));
    }
    for (int i = 0; i < 3; ++i) {
        CHECK(r.throttle[i] == 0.8);  // no differential throttle needed
        CHECK(std::abs(r.pitch[i]) <= c.gimbalLimit && std::abs(r.yaw[i]) <= c.gimbalLimit);
    }
}

TEST_CASE(allocator_uses_differential_throttle_when_gimbal_saturates) {
    const fsw::AllocatorConfig c = allocatorConfig();
    fsw::AllocatorRequest req;
    req.throttle = 0.7;
    req.torque[0] = 2.0e7;  // beyond what the 13 degrees of pitch/yaw gimbal can give
    req.torque[1] = 0.0;
    req.torque[2] = 2.0e5;
    const fsw::AllocatorResult r = fsw::allocateSeaLevelEngines(c, req);
    CHECK(r.gimbalSaturated);
    CHECK(r.differentialThrottleUsed);
    const Achieved a = evaluate(c, r);
    // Total thrust preserved, every engine between the throttle floor and full throttle.
    CHECK_NEAR(a.totalThrust, nominalTotalThrust(c, 0.7), 1e-6 * a.totalThrust);
    for (int i = 0; i < 3; ++i) {
        CHECK(r.throttle[i] >= c.minThrottle - 1e-12 && r.throttle[i] <= 1.0 + 1e-12);
        CHECK(std::abs(r.pitch[i]) <= c.gimbalLimit && std::abs(r.yaw[i]) <= c.gimbalLimit);
    }
    // Achieved plus reported residual equals the request.
    for (int k = 0; k < 3; ++k) {
        CHECK_NEAR(a.torque[k] + r.residualTorque[k], req.torque[k], 0.03 * std::abs(req.torque[0]));
    }
    // And differential throttle really adds torque beyond the gimbal-only result.
    CHECK(a.torque[0] > 0.0);
    CHECK(r.residualTorque[0] < req.torque[0]);
}

TEST_CASE(allocator_has_no_differential_throttle_at_the_floor) {
    const fsw::AllocatorConfig c = allocatorConfig();
    fsw::AllocatorRequest req;
    req.throttle = 0.4;  // minimum
    req.torque[1] = 2.0e7;
    const fsw::AllocatorResult r = fsw::allocateSeaLevelEngines(c, req);
    CHECK(r.gimbalSaturated);
    CHECK(!r.differentialThrottleUsed);
    for (int i = 0; i < 3; ++i) {
        CHECK(r.throttle[i] == 0.4);
    }
    CHECK(std::abs(r.residualTorque[1]) > 0.0);
}

TEST_CASE(allocator_roll_is_gimbal_only_and_keeps_headroom) {
    const fsw::AllocatorConfig c = allocatorConfig();
    fsw::AllocatorRequest req;
    req.throttle = 0.7;
    req.torque[2] = 5.0e6;      // far beyond the reserved roll budget
    req.torque[0] = 3.0e7;      // and saturating pitch at the same time
    const fsw::AllocatorResult r = fsw::allocateSeaLevelEngines(c, req);
    CHECK(r.rollSaturated);
    const Achieved a = evaluate(c, r);
    // Roll torque equals the budget's worth: 3 * R * T * budget; differential throttle adds none.
    const double thrust = nominalTotalThrust(c, 0.7) / 3.0;
    CHECK_NEAR(a.torque[2], 3.0 * c.ringRadius * thrust * c.rollGimbalBudget, 0.03 * a.torque[2]);
    CHECK_NEAR(a.torque[2] + r.residualTorque[2], req.torque[2], 0.03 * req.torque[2]);
    // Roll budget plus pitch/yaw gimbal never exceeds the per-engine limit.
    for (int i = 0; i < 3; ++i) {
        CHECK(std::abs(r.pitch[i]) <= c.gimbalLimit + 1e-12);
        CHECK(std::abs(r.yaw[i]) <= c.gimbalLimit + 1e-12);
    }
}
