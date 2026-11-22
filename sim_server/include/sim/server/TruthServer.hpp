#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "sim/server/JsonMessages.hpp"
#include "sim/server/SnapshotRing.hpp"
#include "sim/server/TelemetryRing.hpp"
#include "sim/server/WireFormat.hpp"
#include "sim/server/WsEndpoint.hpp"

struct mg_context;

namespace sim::server {

struct TruthServerConfig {
    std::string host = "0.0.0.0";
    int port = 8080;  // 0 picks a free port (see boundPort())
    std::string webRoot;
    double timeScale = 1.0;
};

// HTTP static files plus the /ws/truth and /ws/telemetry WebSockets on one port.
//
// Structure: one civetweb context and two WsEndpoints (per-client queues, sender
// threads, close and join; see WsEndpoint.hpp), each drained by its own
// broadcaster thread from its own lock-free ring. A broadcaster only enqueues
// into per-client bounded queues (short critical section, never any socket I/O);
// each client has its own sender thread doing the blocking writes, so a slow or
// stalled client cannot delay a broadcaster, the sim or other clients, on either
// endpoint. Overflow drops that client's oldest queued droppable items, never
// JSON control messages and never the final halted truth snapshot. A client
// whose write fails or times out (request_timeout_ms) is dropped; civetweb
// closes it on its next keepalive ping, and its sender thread is joined in the
// close handler.
//
// Client cap: civetweb pins one worker thread per WebSocket (num_threads is 32),
// so at most kMaxClients WebSockets are accepted across both endpoints; extras
// are refused in the connect handler and plain HTTP stays reachable.
//
// Truth: hello, latest snapshot, run_started, heartbeat of the halted snapshot,
// restart requests (docs/protocol/truth_stream.md). Telemetry: hello, current
// value table (latest frame per APID), run_started, ground frames; client
// messages are ignored (docs/protocol/telemetry_stream.md).
class TruthServer {
public:
    static constexpr std::size_t kMaxClients = 24;               // WebSockets over both endpoints
    static constexpr std::size_t kQueueCapacity = 300;           // droppable snapshots per truth client (~3 s)
    static constexpr std::size_t kTelemetryQueueCapacity = 600;  // droppable frames per telemetry client (~3 s)

    TruthServer(const TruthServerConfig& config, SnapshotRing& ring, TelemetryRing& telemetryRing,
                std::function<uint32_t()> currentRunId, std::function<void()> onRestart,
                const HelloInfo& helloTemplate);
    ~TruthServer();

    TruthServer(const TruthServer&) = delete;
    TruthServer& operator=(const TruthServer&) = delete;

    // Throws std::runtime_error on bind failure or a missing web root.
    void start();
    void stop();

    int boundPort() const { return boundPort_; }
    std::size_t clientCount() const;              // WebSocket clients on either endpoint
    uint64_t droppedForClients() const;           // truth snapshots dropped from slow client queues
    uint64_t telemetryDroppedForClients() const;  // telemetry frames dropped from slow client queues

private:
    struct CvtEntry {
        bool valid = false;
        uint16_t apid = 0;
        WsEndpoint::Item item;
    };
    static constexpr std::size_t kMaxApids = 16;

    void truthBroadcasterMain();
    void telemetryBroadcasterMain();
    void enqueueSnapshot(const WireSnapshot& wire, bool essential);  // caller holds truth_.mutex()
    void truthJoin(std::vector<WsEndpoint::Item>& initial);          // truth_.mutex() held
    void telemetryJoin(std::vector<WsEndpoint::Item>& initial);      // telemetry_.mutex() held
    bool acceptRestart();

    TruthServerConfig config_;
    SnapshotRing& ring_;
    TelemetryRing& telemetryRing_;
    std::function<uint32_t()> currentRunId_;
    std::function<void()> onRestart_;
    HelloInfo helloTemplate_;

    mg_context* ctx_ = nullptr;
    int boundPort_ = 0;
    std::thread truthBroadcaster_;
    std::thread telemetryBroadcaster_;
    std::atomic<bool> stopping_{false};
    std::atomic<std::size_t> connections_{0};  // accepted WebSockets, for the cap

    WsEndpoint truth_;
    WsEndpoint telemetry_;

    // Truth state, guarded by truth_.mutex().
    WireSnapshot latest_{};
    bool haveLatest_ = false;
    bool latestHalted_ = false;
    uint32_t latestRunId_ = 0;
    uint32_t lastRunId_ = 0;

    // Telemetry state, guarded by telemetry_.mutex(). The current value table
    // holds the latest ground frame per APID of the latest run.
    std::array<CvtEntry, kMaxApids> cvt_{};
    uint32_t telemetryRunId_ = 0;

    std::mutex restartMutex_;
    bool haveRestart_ = false;
    std::chrono::steady_clock::time_point lastRestart_;
    std::chrono::steady_clock::time_point lastRestartLog_;
};

}  // namespace sim::server
