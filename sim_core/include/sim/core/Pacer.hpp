#pragma once

#include <cstdint>
#include <functional>

#include "sim/core/Simulation.hpp"

namespace sim {

struct PacerOptions {
    bool realtime = true;     // false: run as fast as possible (Monte Carlo, tests)
    double timeScale = 1.0;   // 1 = real time, 0.1 = slow motion, 10 = 10x
    double maxSimTime = 1e9;  // stop after this much sim time [s]
    double lagWarnThreshold = 0.05;  // count a lag event when this far behind [s]
};

struct PacerStats {
    uint64_t steps = 0;
    double simTime = 0.0;
    double wallTime = 0.0;
    double maxLag = 0.0;     // worst amount the sim fell behind the wall clock [s]
    uint64_t lagEvents = 0;  // times the lag exceeded lagWarnThreshold

    double speedFactor() const { return wallTime > 0.0 ? simTime / wallTime : 0.0; }
};

// Runs a Simulation against the wall clock (or not). The sim itself has no
// notion of real time; this is the only place that does.
//
// In real-time mode steps are run in batches and the thread sleeps only when it
// is more than ~2 ms ahead, so OS sleep granularity does not limit the step rate.
// `shouldStop` is checked between steps (for UI quit, Ctrl-C, scenario end).
PacerStats runPaced(Simulation& sim, const PacerOptions& options,
                    const std::function<bool()>& shouldStop = {});

}  // namespace sim
