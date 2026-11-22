#include "sim/core/Pacer.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <timeapi.h>
#endif

namespace sim {
namespace {

using Clock = std::chrono::steady_clock;

// Windows sleeps in ~15.6 ms ticks by default; ask for 1 ms while pacing.
struct TimerResolutionGuard {
#ifdef _WIN32
    TimerResolutionGuard() { timeBeginPeriod(1); }
    ~TimerResolutionGuard() { timeEndPeriod(1); }
#endif
};

double secondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

PacerStats runPaced(Simulation& sim, const PacerOptions& options, const std::function<bool()>& shouldStop) {
    if (options.realtime && !(options.timeScale > 0.0)) {
        throw std::invalid_argument("timeScale must be positive");
    }

    PacerStats stats;
    const double simStart = sim.time();
    const Clock::time_point wallStart = Clock::now();
    const TimerResolutionGuard timerGuard;
    (void)timerGuard;

    constexpr double kSleepWhenAheadBy = 0.002;  // [s]

    while (!sim.halted() && sim.time() - simStart < options.maxSimTime) {
        if (shouldStop && shouldStop()) {
            break;
        }

        sim.step();
        ++stats.steps;

        if (!options.realtime) {
            continue;
        }

        const double wallTarget = (sim.time() - simStart) / options.timeScale;
        const double wallNow = secondsSince(wallStart);
        const double ahead = wallTarget - wallNow;

        if (ahead > kSleepWhenAheadBy) {
            std::this_thread::sleep_until(wallStart + std::chrono::duration_cast<Clock::duration>(
                                                          std::chrono::duration<double>(wallTarget)));
        } else if (ahead < 0.0) {
            const double lag = -ahead;
            stats.maxLag = std::max(stats.maxLag, lag);
            if (lag > options.lagWarnThreshold) {
                ++stats.lagEvents;
            }
        }
    }

    stats.simTime = sim.time() - simStart;
    stats.wallTime = secondsSince(wallStart);
    return stats;
}

}  // namespace sim
