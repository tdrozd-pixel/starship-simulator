#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "sim/core/Bus.hpp"
#include "sim/core/ContinuousSystem.hpp"
#include "sim/core/Scheduler.hpp"

namespace sim {

struct SimConfig {
    double baseRateHz = 1000.0;
    uint64_t seed = 1;
};

// The sim executive. Owns the bus, the continuous systems and the scheduler.
// It never looks at a clock: time is step * dt, so runs are deterministic and
// can be paced at any speed (see Pacer).
class Simulation {
public:
    explicit Simulation(SimConfig config = {});

    template <typename T, typename... Args>
    T& emplaceSystem(Args&&... args) {
        auto system = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *system;
        systems_.push_back(std::move(system));
        return ref;
    }

    Scheduler& scheduler() { return scheduler_; }
    Bus& bus() { return bus_; }

    void initialize();
    void step();

    double time() const { return static_cast<double>(step_) * dt_; }
    double dt() const { return dt_; }
    uint64_t stepCount() const { return step_; }
    uint64_t seed() const { return config_.seed; }
    bool halted() const;

private:
    SimConfig config_;
    double dt_;
    uint64_t step_ = 0;
    bool initialized_ = false;
    Bus bus_;
    Scheduler scheduler_;
    std::vector<std::unique_ptr<ContinuousSystem>> systems_;
};

}  // namespace sim
