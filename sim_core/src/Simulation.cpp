#include "sim/core/Simulation.hpp"

#include <stdexcept>

#include "sim/core/Profiling.hpp"

namespace sim {

Simulation::Simulation(SimConfig config)
    : config_(config), dt_(1.0 / config.baseRateHz), scheduler_(config.baseRateHz) {}

void Simulation::initialize() {
    if (initialized_) {
        throw std::logic_error("Simulation::initialize called twice");
    }
    for (auto& system : systems_) {
        system->initialize(bus_);
    }
    scheduler_.initialize(bus_);
    initialized_ = true;
}

void Simulation::step() {
    SIM_ZONE("Simulation::step");
    if (!initialized_) {
        throw std::logic_error("Simulation::step called before initialize");
    }

    const double t = time();
    const StepContext ctx{t, dt_, step_};

    {
        SIM_ZONE("prepare");
        for (auto& system : systems_) {
            system->prepare(t);
        }
    }

    scheduler_.run(ctx);

    {
        SIM_ZONE("integrate");
        for (auto& system : systems_) {
            system->integrate(t, dt_);
        }
    }

    ++step_;
    SIM_FRAME_MARK("sim_step");
}

bool Simulation::halted() const {
    for (const auto& system : systems_) {
        if (system->halted()) {
            return true;
        }
    }
    return false;
}

}  // namespace sim
