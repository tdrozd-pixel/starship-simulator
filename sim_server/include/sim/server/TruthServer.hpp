#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "sim/server/JsonMessages.hpp"
#include "sim/server/SnapshotRing.hpp"
#include "sim/server/WireFormat.hpp"

struct mg_context;
struct mg_connection;

namespace sim::server {

struct TruthServerConfig {
    std::string host = "0.0.0.0";
    int port = 8080;  // 0 picks a free port (see boundPort())
    std::string webRoot;
    double timeScale = 1.0;
};

// HTTP static files plus the /ws/truth WebSocket on one port.
//
// Threads: the broadcaster drains the snapshot ring and only enqueues into
// per-client bounded queues (short critical section, never any socket I/O).
// Each client has its own sender thread doing the blocking writes, so a slow or
// stalled client cannot delay the broadcaster, the sim or other clients.
// Overflow drops that client's oldest queued snapshots, never JSON control
// messages and never the final halted snapshot. A client whose write fails or
// times out (request_timeout_ms) is dropped; civetweb closes it on its next
// keepalive ping, and its sender thread is joined in the close handler so it
// never touches the connection afterwards.
//
// Client cap: civetweb pins one worker thread per WebSocket (num_threads is 32),
// so at most kMaxClients WebSockets are accepted; extras are refused in the
// connect handler and plain HTTP stays reachable.
class TruthServer {
public:
    static constexpr std::size_t kMaxClients = 24;
    static constexpr std::size_t kQueueCapacity = 300;  // droppable snapshots per client (~3 s)

    TruthServer(const TruthServerConfig& config, SnapshotRing& ring, std::function<uint32_t()> currentRunId,
                std::function<void()> onRestart, const HelloInfo& helloTemplate);
    ~TruthServer();

    TruthServer(const TruthServer&) = delete;
    TruthServer& operator=(const TruthServer&) = delete;

    // Throws std::runtime_error on bind failure or a missing web root.
    void start();
    void stop();

    int boundPort() const { return boundPort_; }
    std::size_t clientCount() const;
    uint64_t droppedForClients() const;  // snapshots dropped from slow client queues

    // civetweb callbacks (public so the C trampolines can reach them).
    bool onConnect();  // false: refuse
    void onReady(mg_connection* conn);
    void onData(int bits, const char* data, std::size_t len);
    void onClose(const mg_connection* conn);

private:
    struct Client;
    struct QueueItem;

    void broadcasterMain();
    void senderMain(Client* client);
    void enqueueText(const std::string& text);      // caller holds mutex_
    void enqueueSnapshot(const WireSnapshot& wire, bool essential);  // caller holds mutex_
    void enqueue(Client& client, QueueItem&& item);              // caller holds mutex_
    bool acceptRestart();

    TruthServerConfig config_;
    SnapshotRing& ring_;
    std::function<uint32_t()> currentRunId_;
    std::function<void()> onRestart_;
    HelloInfo helloTemplate_;

    mg_context* ctx_ = nullptr;
    int boundPort_ = 0;
    std::thread broadcaster_;
    std::atomic<bool> stopping_{false};
    std::atomic<std::size_t> connections_{0};  // accepted WebSockets, for the cap

    mutable std::mutex mutex_;  // guards everything below
    std::vector<std::unique_ptr<Client>> clients_;
    WireSnapshot latest_{};
    bool haveLatest_ = false;
    bool latestHalted_ = false;
    uint32_t latestRunId_ = 0;
    uint32_t lastRunId_ = 0;
    uint64_t droppedForClients_ = 0;

    std::mutex restartMutex_;
    bool haveRestart_ = false;
    std::chrono::steady_clock::time_point lastRestart_;
    std::chrono::steady_clock::time_point lastRestartLog_;
};

}  // namespace sim::server
