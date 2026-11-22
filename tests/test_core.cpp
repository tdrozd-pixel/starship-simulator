#include <cmath>
#include <memory>
#include <numbers>

#include "TestFramework.hpp"
#include "sim/core/Bus.hpp"
#include "sim/core/Integrator.hpp"
#include "sim/core/Random.hpp"
#include "sim/core/Scheduler.hpp"
#include "sim/core/Simulation.hpp"

namespace {

// Harmonic oscillator x'' = -x, for checking RK4 against the exact solution
struct Osc {
    double x;
    double v;
};
struct OscDeriv {
    double dx;
    double dv;
};
OscDeriv operator+(const OscDeriv& a, const OscDeriv& b) { return {a.dx + b.dx, a.dv + b.dv}; }
OscDeriv operator*(double s, const OscDeriv& d) { return {s * d.dx, s * d.dv}; }
Osc addScaled(const Osc& s, const OscDeriv& d, double h) { return {s.x + h * d.dx, s.v + h * d.dv}; }

class CountingModel final : public sim::Model {
public:
    CountingModel(const char* name, double rate, int& counter) : sim::Model(name, rate), counter_(counter) {}
    void update(const sim::StepContext&) override { ++counter_; }

private:
    int& counter_;
};

}  // namespace

TEST_CASE(rk4_matches_harmonic_oscillator) {
    Osc s{1.0, 0.0};
    const double h = 1e-3;
    const int steps = 10000;  // 10 s
    for (int i = 0; i < steps; ++i) {
        s = sim::rk4Step(s, i * h, h, [](const Osc& st, double) { return OscDeriv{st.v, -st.x}; });
    }
    CHECK_NEAR(s.x, std::cos(10.0), 1e-10);
    CHECK_NEAR(s.v, -std::sin(10.0), 1e-10);
}

TEST_CASE(scheduler_runs_rate_groups) {
    sim::Scheduler scheduler(1000.0);
    int fast = 0;
    int mid = 0;
    int slow = 0;
    scheduler.emplace<CountingModel>("fast", 1000.0, fast);
    scheduler.emplace<CountingModel>("mid", 100.0, mid);
    scheduler.emplace<CountingModel>("slow", 10.0, slow);
    for (uint64_t step = 0; step < 1000; ++step) {
        scheduler.run({step * 1e-3, 1e-3, step});
    }
    CHECK(fast == 1000);
    CHECK(mid == 100);
    CHECK(slow == 10);
}

TEST_CASE(scheduler_rejects_rates_that_do_not_divide_base) {
    sim::Scheduler scheduler(1000.0);
    int n = 0;
    CHECK_THROWS(scheduler.add(std::make_unique<CountingModel>("bad", 300.0, n)));
    CHECK_THROWS(scheduler.add(std::make_unique<CountingModel>("too_fast", 2000.0, n)));
}

TEST_CASE(bus_channels_are_typed_and_sequenced) {
    sim::Bus bus;
    auto& ch = bus.channel<double>("x");
    CHECK(!ch.latest().valid());
    ch.publish(3.0, 0.5);
    ch.publish(4.0, 0.6);
    CHECK(ch.latest().seq == 2);
    CHECK_NEAR(ch.latest().data, 4.0, 0.0);
    CHECK(&bus.channel<double>("x") == &ch);
    CHECK_THROWS(bus.channel<int>("x"));
}

TEST_CASE(rng_is_deterministic_and_streams_are_independent) {
    sim::Rng a(sim::deriveSeed(7, "imu"));
    sim::Rng b(sim::deriveSeed(7, "imu"));
    sim::Rng c(sim::deriveSeed(7, "altimeter"));
    bool allEqual = true;
    bool anyDifferent = false;
    for (int i = 0; i < 100; ++i) {
        const double va = a.normal();
        allEqual = allEqual && (va == b.normal());
        anyDifferent = anyDifferent || (va != c.normal());
    }
    CHECK(allEqual);
    CHECK(anyDifferent);

    sim::Rng r(123);
    double sum = 0.0;
    double sumSq = 0.0;
    const int n = 200000;
    for (int i = 0; i < n; ++i) {
        const double v = r.normal();
        sum += v;
        sumSq += v * v;
    }
    const double mean = sum / n;
    CHECK_NEAR(mean, 0.0, 0.01);
    CHECK_NEAR(std::sqrt(sumSq / n - mean * mean), 1.0, 0.01);
}

TEST_CASE(simulation_time_is_exact_step_count) {
    sim::Simulation simulation(sim::SimConfig{1000.0, 1});
    simulation.initialize();
    for (int i = 0; i < 12345; ++i) {
        simulation.step();
    }
    CHECK(simulation.stepCount() == 12345);
    CHECK_NEAR(simulation.time(), 12.345, 1e-12);
}
