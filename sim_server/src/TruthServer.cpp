#include "sim/server/TruthServer.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "civetweb.h"
#include "sim/server/TelemetryFrame.hpp"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace sim::server {
namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kRestartMinInterval = std::chrono::seconds(1);

// civetweb's own directory redirect uses the configured authentication domain
// as the host, which is wrong on a LAN, so /gnc gets a relative redirect instead.
int gncRedirectHandler(mg_connection* conn, void*) {
    mg_printf(conn,
              "HTTP/1.1 302 Found\r\nLocation: /gnc/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    return 1;
}

}  // namespace

TruthServer::TruthServer(const TruthServerConfig& config, SnapshotRing& ring, TelemetryRing& telemetryRing,
                         std::function<uint32_t()> currentRunId, std::function<void()> onRestart,
                         const HelloInfo& helloTemplate)
    : config_(config),
      ring_(ring),
      telemetryRing_(telemetryRing),
      currentRunId_(std::move(currentRunId)),
      onRestart_(std::move(onRestart)),
      helloTemplate_(helloTemplate),
      truth_(kQueueCapacity, connections_, kMaxClients),
      telemetry_(kTelemetryQueueCapacity, connections_, kMaxClients) {
    helloTemplate_.timeScale = config.timeScale;
    truth_.setJoinHook([this](std::vector<WsEndpoint::Item>& initial) { truthJoin(initial); });
    truth_.setTextHandler([this](std::string_view text) {
        if (parseClientMessage(text) == ClientMsg::Restart && onRestart_ && acceptRestart()) {
            onRestart_();
        }
    });
    telemetry_.setJoinHook([this](std::vector<WsEndpoint::Item>& initial) { telemetryJoin(initial); });
    // No text handler on telemetry: a console cannot command anything.
}

TruthServer::~TruthServer() { stop(); }

std::size_t TruthServer::clientCount() const { return truth_.clientCount() + telemetry_.clientCount(); }

uint64_t TruthServer::droppedForClients() const { return truth_.droppedForClients(); }

uint64_t TruthServer::telemetryDroppedForClients() const { return telemetry_.droppedForClients(); }

void TruthServer::start() {
    std::error_code ec;
    if (!std::filesystem::is_directory(config_.webRoot, ec)) {
        throw std::runtime_error("web root does not exist or is not a directory: " + config_.webRoot);
    }

    mg_init_library(0);

    const std::string listening = config_.host + ":" + std::to_string(config_.port);
    const char* options[] = {"listening_ports",
                             listening.c_str(),
                             "document_root",
                             config_.webRoot.c_str(),
                             "enable_directory_listing",
                             "no",
                             "enable_websocket_ping_pong",
                             "yes",
                             // The WebSocket read loop wakes this often to notice a connection
                             // marked for closing and to send keepalive pings.
                             "websocket_timeout_ms",
                             "1000",
                             // Also bounds how long one blocked socket write can last.
                             "request_timeout_ms",
                             "5000",
                             "tcp_nodelay",
                             "1",
                             "num_threads",
                             "32",
                             "static_file_max_age",
                             "0",  // dev server: always revalidate
                             "extra_mime_types",
                             ".js=text/javascript,.mjs=text/javascript,.glb=model/gltf-binary,"
                             ".gltf=model/gltf+json,.wasm=application/wasm",
                             nullptr};

    mg_callbacks callbacks{};
    ctx_ = mg_start(&callbacks, this, options);
    if (ctx_ == nullptr) {
        throw std::runtime_error("cannot listen on " + listening + " (port in use or address not available)");
    }
    mg_set_request_handler(ctx_, "/gnc$", gncRedirectHandler, nullptr);
    truth_.attach(ctx_, "/ws/truth");
    telemetry_.attach(ctx_, "/ws/telemetry");

    mg_server_port ports[4] = {};
    const int n = mg_get_server_ports(ctx_, 4, ports);
    boundPort_ = n > 0 ? ports[0].port : config_.port;

    stopping_ = false;
    truthBroadcaster_ = std::thread([this] { truthBroadcasterMain(); });
    telemetryBroadcaster_ = std::thread([this] { telemetryBroadcasterMain(); });
}

void TruthServer::stop() {
    stopping_ = true;
    if (truthBroadcaster_.joinable()) {
        truthBroadcaster_.join();
    }
    if (telemetryBroadcaster_.joinable()) {
        telemetryBroadcaster_.join();
    }
    if (ctx_ != nullptr) {
        // Wake every sender; civetweb's close handler then joins each one.
        truth_.markAllDead();
        telemetry_.markAllDead();
        mg_stop(ctx_);
        ctx_ = nullptr;
    }
}

void TruthServer::truthJoin(std::vector<WsEndpoint::Item>& initial) {
    HelloInfo hello = helloTemplate_;
    hello.runId = haveLatest_ ? latestRunId_ : currentRunId_();  // match the snapshot that follows
    WsEndpoint::Item helloItem;
    helloItem.text = makeHello(hello);
    initial.push_back(std::move(helloItem));
    if (haveLatest_) {
        WsEndpoint::Item snap;
        snap.binary = true;
        snap.size = latest_.size();
        std::copy(latest_.begin(), latest_.end(), snap.data.begin());
        initial.push_back(std::move(snap));
    }
}

void TruthServer::telemetryJoin(std::vector<WsEndpoint::Item>& initial) {
    WsEndpoint::Item helloItem;
    helloItem.text = makeTelemetryHello(telemetryRunId_ != 0 ? telemetryRunId_ : currentRunId_(), config_.timeScale);
    initial.push_back(std::move(helloItem));
    for (const CvtEntry& e : cvt_) {
        if (e.valid) {
            initial.push_back(e.item);
        }
    }
}

bool TruthServer::acceptRestart() {
    const std::lock_guard<std::mutex> lock(restartMutex_);
    const Clock::time_point now = Clock::now();
    if (haveRestart_ && now - lastRestart_ < kRestartMinInterval) {
        if (now - lastRestartLog_ >= kRestartMinInterval) {
            lastRestartLog_ = now;
            std::fprintf(stderr, "restart request ignored (rate limit: 1 per second)\n");
        }
        return false;
    }
    haveRestart_ = true;
    lastRestart_ = now;
    return true;
}

void TruthServer::enqueueSnapshot(const WireSnapshot& wire, bool essential) {
    WsEndpoint::Item item;
    item.binary = true;
    item.essential = essential;
    item.size = wire.size();
    std::copy(wire.begin(), wire.end(), item.data.begin());
    truth_.enqueueAllLocked(item);
}

void TruthServer::truthBroadcasterMain() {
    constexpr auto kHeartbeat = std::chrono::seconds(1);
    constexpr auto kIdleSleep = std::chrono::milliseconds(2);

    vehicle::TruthSnapshot s;
    WireSnapshot wire{};
    Clock::time_point lastHeartbeat = Clock::now();

    while (!stopping_) {
        bool drained = false;
        while (!stopping_ && ring_.tryPop(s)) {
            drained = true;
            encodeSnapshot(s, wire);
            const bool halted = (s.flags & vehicle::kSnapHalted) != 0;
            // run_started, the latest-snapshot update and the snapshot fan-out are one
            // critical section, so a joining client sees a consistent picture.
            const std::lock_guard<std::mutex> lock(truth_.mutex());
            if (s.runId != lastRunId_) {
                lastRunId_ = s.runId;
                WsEndpoint::Item started;
                started.essential = true;
                started.text = makeRunStarted(s.runId);
                truth_.enqueueAllLocked(started);
            }
            latest_ = wire;
            haveLatest_ = true;
            latestHalted_ = halted;
            latestRunId_ = s.runId;
            enqueueSnapshot(wire, halted);
            if (halted) {
                lastHeartbeat = Clock::now();
            }
        }
        if (!drained) {
            std::this_thread::sleep_for(kIdleSleep);
        }

        const Clock::time_point now = Clock::now();
        if (now - lastHeartbeat >= kHeartbeat) {
            const std::lock_guard<std::mutex> lock(truth_.mutex());
            if (latestHalted_) {
                enqueueSnapshot(latest_, false);  // heartbeat: droppable, same step
                lastHeartbeat = now;
            }
        }
    }
}

void TruthServer::telemetryBroadcasterMain() {
    constexpr auto kIdleSleep = std::chrono::milliseconds(2);

    TelemetryItem t;
    uint32_t lastRunId = 0;
    uint8_t frame[kMaxGroundFrame];

    while (!stopping_) {
        bool drained = false;
        while (!stopping_ && telemetryRing_.tryPop(t)) {
            drained = true;
            if (t.length < kMinCcsdsPacket) {
                continue;
            }
            const uint16_t apid = packetApid(t.bytes);
            WsEndpoint::Item item;
            item.binary = true;
            item.size = encodeGroundFrame(t, frame);
            std::copy(frame, frame + item.size, item.data.begin());

            // run_started, the table update and the fan-out are one critical
            // section, so a joining client sees a consistent picture.
            const std::lock_guard<std::mutex> lock(telemetry_.mutex());
            if (t.runId != lastRunId) {
                lastRunId = t.runId;
                telemetryRunId_ = t.runId;
                for (CvtEntry& e : cvt_) {
                    e.valid = false;  // current run only
                }
                WsEndpoint::Item started;
                started.essential = true;
                started.text = makeRunStarted(t.runId);
                telemetry_.enqueueAllLocked(started);
            }
            CvtEntry* slot = nullptr;
            for (CvtEntry& e : cvt_) {
                if (e.valid && e.apid == apid) {
                    slot = &e;
                    break;
                }
                if (!e.valid && slot == nullptr) {
                    slot = &e;
                }
            }
            if (slot != nullptr) {
                slot->valid = true;
                slot->apid = apid;
                slot->item = item;
            }
            telemetry_.enqueueAllLocked(item);
        }
        if (!drained) {
            std::this_thread::sleep_for(kIdleSleep);
        }
    }
}

}  // namespace sim::server
