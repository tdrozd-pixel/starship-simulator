#include "sim/server/SimRunner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>

#include "sim/core/Pacer.hpp"

namespace sim::server {

SimRunner::SimRunner(const SimRunnerConfig& config, SnapshotRing& ring, TelemetryRing& telemetryRing)
    : config_(config), ring_(ring), telemetryRing_(telemetryRing) {}

SimRunner::~SimRunner() { stop(); }

void SimRunner::start() {
    thread_ = std::thread([this] { threadMain(); });
}

void SimRunner::requestRestart() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        restart_ = true;
    }
    cv_.notify_all();
}

void SimRunner::stop() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void SimRunner::push(const vehicle::TruthSnapshot& snapshot) noexcept {
    vehicle::TruthSnapshot s = snapshot;
    s.runId = runId_.load(std::memory_order_relaxed);
    if (ring_.tryPush(s)) {
        return;
    }
    if ((s.flags & vehicle::kSnapHalted) != 0) {
        // The one-shot final snapshot must not be lost: wait for room.
        while (!stop_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (ring_.tryPush(s)) {
                return;
            }
        }
    }
    dropped_.fetch_add(1, std::memory_order_relaxed);
}

void SimRunner::deliver(const uint8_t* packet, std::size_t length, double groundTime) noexcept {
    if (packet == nullptr || length > vehicle::kMaxTelemetryPacket) {
        telemetryDropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    TelemetryItem item;
    item.runId = runId_.load(std::memory_order_relaxed);
    item.groundTime = groundTime;
    item.length = static_cast<uint16_t>(length);
    std::copy(packet, packet + length, item.bytes);
    if (!telemetryRing_.tryPush(item)) {
        telemetryDropped_.fetch_add(1, std::memory_order_relaxed);
    }
}

void SimRunner::threadMain() {
    try {
        while (!stop_) {
            if (!config_.autostart && config_.clientPresent) {
                if (!config_.clientPresent() && !warnedWaiting_ && config_.onWaitingForClient) {
                    warnedWaiting_ = true;
                    config_.onWaitingForClient();
                }
                std::unique_lock<std::mutex> lock(mutex_);
                while (!stop_ && !restart_ && !config_.clientPresent()) {
                    cv_.wait_for(lock, std::chrono::milliseconds(20));
                }
            }
            if (stop_) {
                break;
            }
            restart_ = false;
            runId_.fetch_add(1);

            vehicle::HopScenarioOptions options = config_.hop;
            options.snapshotSink = this;
            options.telemetrySink = this;
            // Allocation is fine here: this is outside the step loop.
            vehicle::HopScenario scenario = vehicle::buildHopScenario(options);

            PacerOptions pacer;
            pacer.realtime = true;
            pacer.timeScale = config_.timeScale;
            pacer.maxSimTime = config_.maxSimTime;
            runPaced(*scenario.sim, pacer, [this] { return stop_.load() || restart_.load(); });

            if (stop_) {
                break;
            }
            if (!restart_) {
                // stepCount() is one past the last executed step, so it is strictly
                // greater than the step of every periodic snapshot.
                scenario.snapshotPublisher->publishFinal(scenario.sim->stepCount(), scenario.sim->time());
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_.load() || restart_.load(); });
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "sim thread failed: %s\n", e.what());
        failed_ = true;
    }
}

}  // namespace sim::server
