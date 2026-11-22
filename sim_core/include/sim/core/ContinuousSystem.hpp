#pragma once

#include "sim/core/Bus.hpp"

namespace sim {

// Something with continuous state that is integrated every base step
// (a vehicle's rigid body, slosh, bending modes, actuator positions...).
//
// Per base step the Simulation calls, in order:
//   1. prepare(t)          on every system: handle discrete events, compute derived
//                          truth (accelerations etc.) that sensors read this step
//   2. the scheduler runs all discrete models that are due
//   3. integrate(t, dt)    on every system: read latest commands, advance the state
class ContinuousSystem {
public:
    virtual ~ContinuousSystem() = default;

    virtual void initialize(Bus& bus) { (void)bus; }
    virtual void prepare(double t) { (void)t; }
    virtual void integrate(double t, double dt) = 0;

    // True once the scenario this system belongs to is over.
    virtual bool halted() const { return false; }
};

}  // namespace sim
