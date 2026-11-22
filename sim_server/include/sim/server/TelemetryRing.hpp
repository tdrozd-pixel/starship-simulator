#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/vehicle/TelemetrySink.hpp"

namespace sim::server {

// One downlinked CCSDS packet with the run it belongs to.
struct TelemetryItem {
    uint32_t runId = 0;
    double groundTime = 0.0;  // [s] sim time when the ground received the packet
    uint16_t length = 0;      // [bytes] valid bytes in `bytes`
    uint8_t bytes[vehicle::kMaxTelemetryPacket] = {};
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4324)  // structure was padded due to alignment specifier
#endif

// Single-producer single-consumer lock-free ring (sim thread to telemetry
// broadcaster), same scheme as SnapshotRing. Never blocks; when full, tryPush
// returns false and the caller counts a drop. Storage is allocated once.
class TelemetryRing {
public:
    static constexpr std::size_t kCapacity = 4096;  // power of two
    static_assert((kCapacity & (kCapacity - 1)) == 0, "capacity must be a power of two");

    TelemetryRing() : buffer_(kCapacity) {}

    // Producer thread only.
    bool tryPush(const TelemetryItem& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= kCapacity) {
            return false;
        }
        buffer_[head & (kCapacity - 1)] = item;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer thread only.
    bool tryPop(TelemetryItem& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        if (head == tail) {
            return false;
        }
        out = buffer_[tail & (kCapacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

private:
    std::vector<TelemetryItem> buffer_;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

}  // namespace sim::server
