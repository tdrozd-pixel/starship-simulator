#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "sim/server/SnapshotRing.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace sim::server {

struct SimRunnerConfig {
    vehicle::HopScenarioOptions hop;
    double timeScale = 1.0;
    double maxSimTime = 120.0;  // [s]

    // When false, a run does not start until clientPresent() returns true (a
    // viewer should see the run from T+0). A null predicate counts as "present".
    bool autostart = true;
    std::function<bool()> clientPresent;
    std::function<void()> onWaitingForClient;  // called once, the first time a run has to wait
};

// Owns the sim thread. Runs the hop paced, publishes snapshots into the ring
// (it is the snapshot sink), and rebuilds the scenario on restart.
class SimRunner final : public vehicle::TruthSnapshotSink {
public:
    SimRunner(const SimRunnerConfig& config, SnapshotRing& ring);
    ~SimRunner() override;

    SimRunner(const SimRunner&) = delete;
    SimRunner& operator=(const SimRunner&) = delete;

    void start();
    void requestRestart();
    void stop();  // joins the sim thread

    uint32_t runId() const { return runId_.load(); }
    bool failed() const { return failed_.load(); }
    uint64_t droppedSnapshots() const { return dropped_.load(); }

    // Sim thread only. Stamps the run id and enqueues; counts a drop when full.
    void push(const vehicle::TruthSnapshot& snapshot) noexcept override;

private:
    void threadMain();

    SimRunnerConfig config_;
    SnapshotRing& ring_;
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> restart_{false};
    std::atomic<bool> failed_{false};
    bool warnedWaiting_ = false;  // sim thread only
    std::atomic<uint32_t> runId_{0};
    std::atomic<uint64_t> dropped_{0};
};

}  // namespace sim::server
