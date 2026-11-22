#include "sim/server/TruthServer.hpp"

#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string_view>

#include "civetweb.h"
#include "sim/vehicle/TruthSnapshot.hpp"

namespace sim::server {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kMaxClientMessage = 4096;  // [bytes], larger text frames are ignored
constexpr auto kRestartMinInterval = std::chrono::seconds(1);

int connectHandler(const mg_connection*, void* user) { return static_cast<TruthServer*>(user)->onConnect() ? 0 : 1; }

void readyHandler(mg_connection* conn, void* user) { static_cast<TruthServer*>(user)->onReady(conn); }

int dataHandler(mg_connection*, int bits, char* data, size_t len, void* user) {
    static_cast<TruthServer*>(user)->onData(bits, data, len);
    return 1;  // keep the connection open
}

void closeHandler(const mg_connection* conn, void* user) { static_cast<TruthServer*>(user)->onClose(conn); }

}  // namespace

struct TruthServer::QueueItem {
    bool binary = false;
    bool essential = false;  // never dropped on overflow
    WireSnapshot wire{};
    std::string text;
};

struct TruthServer::Client {
    mg_connection* conn = nullptr;
    std::deque<QueueItem> queue;
    std::size_t droppable = 0;  // queued items with essential == false
    bool dead = false;
    std::condition_variable cv;
    std::thread sender;
};

TruthServer::TruthServer(const TruthServerConfig& config, SnapshotRing& ring, std::function<uint32_t()> currentRunId,
                         std::function<void()> onRestart, const HelloInfo& helloTemplate)
    : config_(config),
      ring_(ring),
      currentRunId_(std::move(currentRunId)),
      onRestart_(std::move(onRestart)),
      helloTemplate_(helloTemplate) {
    helloTemplate_.timeScale = config.timeScale;
}

TruthServer::~TruthServer() { stop(); }

std::size_t TruthServer::clientCount() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return clients_.size();
}

uint64_t TruthServer::droppedForClients() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return droppedForClients_;
}

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
    mg_set_websocket_handler(ctx_, "/ws/truth", connectHandler, readyHandler, dataHandler, closeHandler, this);

    mg_server_port ports[4] = {};
    const int n = mg_get_server_ports(ctx_, 4, ports);
    boundPort_ = n > 0 ? ports[0].port : config_.port;

    stopping_ = false;
    broadcaster_ = std::thread([this] { broadcasterMain(); });
}

void TruthServer::stop() {
    stopping_ = true;
    if (broadcaster_.joinable()) {
        broadcaster_.join();
    }
    if (ctx_ != nullptr) {
        // Wake every sender; civetweb's close handler then joins each one.
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            for (auto& c : clients_) {
                c->dead = true;
                c->cv.notify_all();
            }
        }
        mg_stop(ctx_);
        ctx_ = nullptr;
    }
}

bool TruthServer::onConnect() {
    if (connections_.fetch_add(1) >= kMaxClients) {
        connections_.fetch_sub(1);
        return false;
    }
    return true;
}

void TruthServer::enqueue(Client& client, QueueItem&& item) {
    if (client.dead) {
        return;
    }
    if (!item.essential) {
        if (client.droppable >= kQueueCapacity) {
            // Drop this client's oldest droppable entry.
            const auto it = std::find_if(client.queue.begin(), client.queue.end(),
                                         [](const QueueItem& q) { return !q.essential; });
            if (it != client.queue.end()) {
                client.queue.erase(it);
                --client.droppable;
                ++droppedForClients_;
            }
        }
        ++client.droppable;
    }
    client.queue.push_back(std::move(item));
    client.cv.notify_one();
}

void TruthServer::enqueueText(const std::string& text) {
    for (auto& c : clients_) {
        QueueItem item;
        item.text = text;
        item.essential = true;
        enqueue(*c, std::move(item));
    }
}

void TruthServer::enqueueSnapshot(const WireSnapshot& wire, bool essential) {
    for (auto& c : clients_) {
        QueueItem item;
        item.binary = true;
        item.essential = essential;
        item.wire = wire;
        enqueue(*c, std::move(item));
    }
}

void TruthServer::onReady(mg_connection* conn) {
    auto client = std::make_unique<Client>();
    client->conn = conn;
    Client* raw = client.get();

    // Hello, latest snapshot and registration happen in one critical section, so
    // the broadcaster can neither interleave nor skip anything for this client.
    const std::lock_guard<std::mutex> lock(mutex_);
    HelloInfo hello = helloTemplate_;
    hello.runId = haveLatest_ ? latestRunId_ : currentRunId_();  // match the snapshot that follows
    QueueItem helloItem;
    helloItem.text = makeHello(hello);
    helloItem.essential = true;
    enqueue(*client, std::move(helloItem));
    if (haveLatest_) {
        QueueItem snap;
        snap.binary = true;
        snap.essential = true;
        snap.wire = latest_;
        enqueue(*client, std::move(snap));
    }
    clients_.push_back(std::move(client));
    raw->sender = std::thread([this, raw] { senderMain(raw); });
}

void TruthServer::senderMain(Client* c) {
    while (true) {
        QueueItem item;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            c->cv.wait(lock, [c] { return c->dead || !c->queue.empty(); });
            if (c->dead) {
                return;
            }
            item = std::move(c->queue.front());
            c->queue.pop_front();
            if (!item.essential) {
                --c->droppable;
            }
        }
        // Blocking write, outside the lock. The connection stays valid until
        // onClose, which joins this thread first.
        const int rc = item.binary ? mg_websocket_write(c->conn, MG_WEBSOCKET_OPCODE_BINARY,
                                                        reinterpret_cast<const char*>(item.wire.data()), item.wire.size())
                                   : mg_websocket_write(c->conn, MG_WEBSOCKET_OPCODE_TEXT, item.text.data(),
                                                        item.text.size());
        if (rc <= 0) {
            // Write failed or timed out: stop sending. civetweb's next keepalive
            // ping fails the same way and closes the connection.
            const std::lock_guard<std::mutex> lock(mutex_);
            c->dead = true;
            c->queue.clear();
            return;
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

void TruthServer::onData(int bits, const char* data, std::size_t len) {
    const bool fin = (bits & 0x80) != 0;
    const bool text = (bits & 0xF) == MG_WEBSOCKET_OPCODE_TEXT;
    if (!fin || !text || data == nullptr || len > kMaxClientMessage) {
        return;  // fragments, binary, ping/pong, oversized frames are ignored
    }
    if (parseClientMessage(std::string_view(data, len)) == ClientMsg::Restart && onRestart_ && acceptRestart()) {
        onRestart_();
    }
}

void TruthServer::onClose(const mg_connection* conn) {
    std::unique_ptr<Client> client;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto it = std::find_if(clients_.begin(), clients_.end(),
                                     [conn](const std::unique_ptr<Client>& c) { return c->conn == conn; });
        if (it != clients_.end()) {
            client = std::move(*it);
            clients_.erase(it);
            client->dead = true;
            client->cv.notify_all();
        }
    }
    // Join outside the lock: the sender may be finishing a blocking write.
    if (client && client->sender.joinable()) {
        client->sender.join();
    }
    connections_.fetch_sub(1);
}

void TruthServer::broadcasterMain() {
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
            const std::lock_guard<std::mutex> lock(mutex_);
            if (s.runId != lastRunId_) {
                lastRunId_ = s.runId;
                enqueueText(makeRunStarted(s.runId));
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
            const std::lock_guard<std::mutex> lock(mutex_);
            if (latestHalted_) {
                enqueueSnapshot(latest_, false);  // heartbeat: droppable, same step
                lastHeartbeat = now;
            }
        }
    }
}

}  // namespace sim::server
