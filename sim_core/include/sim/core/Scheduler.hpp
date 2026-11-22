#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "sim/core/Model.hpp"

namespace sim {

// Runs discrete models in rate groups. Every model's rate must divide the base
// rate exactly (1000 Hz base: 1000, 500, 250, 200, 100, 50, 10, 1 Hz are fine;
// 300 Hz is rejected). Models run in the order they were added, so add them in
// dependency order: sensors, then flight software, then loggers.
class Scheduler {
public:
    explicit Scheduler(double baseRateHz);

    Model& add(std::unique_ptr<Model> model);

    template <typename T, typename... Args>
    T& emplace(Args&&... args) {
        auto model = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *model;
        add(std::move(model));
        return ref;
    }

    void initialize(Bus& bus);
    void run(const StepContext& ctx);

    double baseRateHz() const { return baseRateHz_; }
    std::size_t size() const { return entries_.size(); }

private:
    struct Entry {
        std::unique_ptr<Model> model;
        uint64_t divisor;
    };

    double baseRateHz_;
    std::vector<Entry> entries_;
};

}  // namespace sim
