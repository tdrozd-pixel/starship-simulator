#include "sim/server/WsEndpoint.hpp"

#include <algorithm>

#include "civetweb.h"

namespace sim::server {
namespace {

constexpr std::size_t kMaxClientMessage = 4096;  // [bytes], larger text frames are ignored

int connectHandler(const mg_connection*, void* user) { return static_cast<WsEndpoint*>(user)->onConnect() ? 0 : 1; }

void readyHandler(mg_connection* conn, void* user) { static_cast<WsEndpoint*>(user)->onReady(conn); }

int dataHandler(mg_connection*, int bits, char* data, size_t len, void* user) {
    static_cast<WsEndpoint*>(user)->onData(bits, data, len);
    return 1;  // keep the connection open
}

void closeHandler(const mg_connection* conn, void* user) { static_cast<WsEndpoint*>(user)->onClose(conn); }

}  // namespace

struct WsEndpoint::Client {
    mg_connection* conn = nullptr;
    std::deque<Item> queue;
    std::size_t droppable = 0;  // queued items with essential == false
    bool dead = false;
    std::condition_variable cv;
    std::thread sender;
};

WsEndpoint::WsEndpoint(std::size_t queueCapacity, std::atomic<std::size_t>& connections, std::size_t maxClients)
    : queueCapacity_(queueCapacity), connections_(connections), maxClients_(maxClients) {}

WsEndpoint::~WsEndpoint() = default;

std::size_t WsEndpoint::clientCount() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return clients_.size();
}

uint64_t WsEndpoint::droppedForClients() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return droppedForClients_;
}

void WsEndpoint::attach(mg_context* ctx, const char* path) {
    mg_set_websocket_handler(ctx, path, connectHandler, readyHandler, dataHandler, closeHandler, this);
}

void WsEndpoint::markAllDead() {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (auto& c : clients_) {
        c->dead = true;
        c->cv.notify_all();
    }
}

bool WsEndpoint::onConnect() {
    if (connections_.fetch_add(1) >= maxClients_) {
        connections_.fetch_sub(1);
        return false;
    }
    return true;
}

void WsEndpoint::enqueue(Client& client, const Item& item) {
    if (client.dead) {
        return;
    }
    if (!item.essential) {
        if (client.droppable >= queueCapacity_) {
            // Drop this client's oldest droppable entry.
            const auto it =
                std::find_if(client.queue.begin(), client.queue.end(), [](const Item& q) { return !q.essential; });
            if (it != client.queue.end()) {
                client.queue.erase(it);
                --client.droppable;
                ++droppedForClients_;
            }
        }
        ++client.droppable;
    }
    client.queue.push_back(item);
    client.cv.notify_one();
}

void WsEndpoint::enqueueAllLocked(const Item& item) {
    for (auto& c : clients_) {
        enqueue(*c, item);
    }
}

void WsEndpoint::onReady(mg_connection* conn) {
    auto client = std::make_unique<Client>();
    client->conn = conn;
    Client* raw = client.get();

    // Join items and registration happen in one critical section, so the
    // broadcaster can neither interleave nor skip anything for this client.
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Item> initial;
    if (joinHook_) {
        joinHook_(initial);
    }
    for (Item& item : initial) {
        item.essential = true;
        enqueue(*client, item);
    }
    clients_.push_back(std::move(client));
    raw->sender = std::thread([this, raw] { senderMain(raw); });
}

void WsEndpoint::senderMain(Client* c) {
    while (true) {
        Item item;
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
                                                        reinterpret_cast<const char*>(item.data.data()), item.size)
                                   : mg_websocket_write(c->conn, MG_WEBSOCKET_OPCODE_TEXT, item.text.data(),
                                                        item.text.size());
        if (rc <= 0) {
            // Write failed or timed out: stop sending. The next keepalive ping
            // from civetweb fails the same way and closes the connection.
            const std::lock_guard<std::mutex> lock(mutex_);
            c->dead = true;
            c->queue.clear();
            return;
        }
    }
}

void WsEndpoint::onData(int bits, const char* data, std::size_t len) {
    const bool fin = (bits & 0x80) != 0;
    const bool text = (bits & 0xF) == MG_WEBSOCKET_OPCODE_TEXT;
    if (!fin || !text || data == nullptr || len > kMaxClientMessage) {
        return;  // fragments, binary, ping/pong, oversized frames are ignored
    }
    if (textHandler_) {
        textHandler_(std::string_view(data, len));
    }
}

void WsEndpoint::onClose(const mg_connection* conn) {
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

}  // namespace sim::server
