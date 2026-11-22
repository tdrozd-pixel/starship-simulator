#include "sim/core/Scheduler.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "sim/core/Profiling.hpp"

namespace sim {

Scheduler::Scheduler(double baseRateHz) : baseRateHz_(baseRateHz) {
    if (!(baseRateHz > 0.0)) {
        throw std::invalid_argument("Scheduler base rate must be positive");
    }
}

Model& Scheduler::add(std::unique_ptr<Model> model) {
    if (!model) {
        throw std::invalid_argument("Scheduler::add given a null model");
    }
    const double rate = model->rateHz();
    if (!(rate > 0.0) || rate > baseRateHz_) {
        throw std::invalid_argument("Model '" + model->name() + "' rate must be in (0, base rate]");
    }
    const double ratio = baseRateHz_ / rate;
    const double rounded = std::round(ratio);
    if (std::abs(ratio - rounded) > 1e-9) {
        throw std::invalid_argument("Model '" + model->name() + "' rate " + std::to_string(rate) +
                                    " Hz does not divide the base rate " + std::to_string(baseRateHz_) + " Hz");
    }
    entries_.push_back(Entry{std::move(model), static_cast<uint64_t>(rounded)});
    return *entries_.back().model;
}

void Scheduler::initialize(Bus& bus) {
    for (auto& entry : entries_) {
        entry.model->initialize(bus);
    }
}

void Scheduler::run(const StepContext& ctx) {
    for (auto& entry : entries_) {
        if (ctx.step % entry.divisor == 0) {
            SIM_ZONE_DYNAMIC(entry.model->name());
            entry.model->update(ctx);
        }
    }
}

}  // namespace sim
