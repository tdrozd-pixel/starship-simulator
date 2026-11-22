#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "sim/core/Bus.hpp"

namespace sim {

struct StepContext {
    double t = 0.0;     // sim time at the start of this step [s]
    double dt = 0.0;    // base step [s]
    uint64_t step = 0;  // base step index
};

// A discrete model: sensors, flight software, thermal, comms, logging...
// The scheduler calls update() every (baseRate / rateHz) base steps.
// Continuous dynamics do NOT go here; see ContinuousSystem.
class Model {
public:
    Model(std::string name, double rateHz) : name_(std::move(name)), rateHz_(rateHz) {}
    virtual ~Model() = default;

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    // Resolve bus channels and allocate everything here, never in update().
    virtual void initialize(Bus& bus) { (void)bus; }
    virtual void update(const StepContext& ctx) = 0;

    const std::string& name() const { return name_; }
    double rateHz() const { return rateHz_; }

private:
    std::string name_;
    double rateHz_;
};

}  // namespace sim
