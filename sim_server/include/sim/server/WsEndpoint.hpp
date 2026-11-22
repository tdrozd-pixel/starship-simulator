#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

struct mg_context;
struct mg_connection;

namespace sim::server {

// Per-client machinery for one WebSocket path: the client list, bounded queues
// with essential and droppable items, one sender thread per client, the shared
// client cap, and a clean close and join. The owner (TruthServer) registers it
// on a civetweb context and runs its own broadcaster that fans items out.
//
// The endpoint mutex guards the client list and queues. Owners also use it to
// guard their own per-stream state (latest snapshot, current value table) so a
// joining client sees a state consistent with the fan-out. Sender threads do
// all blocking socket writes outside the mutex.
class WsEndpoint {
public:
    static constexpr std::size_t kMaxBinary = 280;  // [bytes] 24-byte ground frame + 256-byte packet

    struct Item {
        bool binary = false;
        bool essential = false;  // never dropped on overflow
        std::size_t size = 0;    // valid bytes of `data` when binary
        std::array<uint8_t, kMaxBinary> data{};
        std::string text;
    };

    // `connections` is shared between endpoints and counts accepted WebSockets
    // against `maxClients` (civetweb pins one worker thread per WebSocket).
    WsEndpoint(std::size_t queueCapacity, std::atomic<std::size_t>& connections, std::size_t maxClients);
    ~WsEndpoint();

    WsEndpoint(const WsEndpoint&) = delete;
    WsEndpoint& operator=(const WsEndpoint&) = delete;

    // Called with the mutex held when a client becomes ready; appends the items
    // the new client receives first (all treated as essential).
    void setJoinHook(std::function<void(std::vector<Item>&)> hook) { joinHook_ = std::move(hook); }
    // Called (no lock held) for each complete client text message.
    void setTextHandler(std::function<void(std::string_view)> handler) { textHandler_ = std::move(handler); }

    void attach(mg_context* ctx, const char* path);
    void markAllDead();  // wake senders so mg_stop can join them

    std::mutex& mutex() const { return mutex_; }
    void enqueueAllLocked(const Item& item);  // caller holds mutex()

    std::size_t clientCount() const;
    uint64_t droppedForClients() const;

    // civetweb callbacks (public so the C trampolines can reach them).
    bool onConnect();  // false: refuse
    void onReady(mg_connection* conn);
    void onData(int bits, const char* data, std::size_t len);
    void onClose(const mg_connection* conn);

private:
    struct Client;

    void senderMain(Client* client);
    void enqueue(Client& client, const Item& item);  // caller holds mutex_

    std::size_t queueCapacity_;
    std::atomic<std::size_t>& connections_;
    std::size_t maxClients_;
    std::function<void(std::vector<Item>&)> joinHook_;
    std::function<void(std::string_view)> textHandler_;

    mutable std::mutex mutex_;
    std::vector<std::unique_ptr<Client>> clients_;
    uint64_t droppedForClients_ = 0;
};

}  // namespace sim::server
