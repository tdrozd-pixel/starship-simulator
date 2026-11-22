#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include "sim/vehicle/TruthSnapshot.hpp"

namespace sim::server {

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4324)  // structure was padded due to alignment specifier
#endif

// Single-producer single-consumer lock-free ring. The producer (sim thread)
// never blocks and never overwrites: when full, tryPush returns false and the
// caller counts a drop. Storage is allocated once in the constructor.
class SnapshotRing {
public:
    static constexpr std::size_t kCapacity = 2048;  // power of two
    static_assert((kCapacity & (kCapacity - 1)) == 0, "capacity must be a power of two");

    SnapshotRing() : buffer_(kCapacity) {}

    // Producer thread only.
    bool tryPush(const vehicle::TruthSnapshot& s) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= kCapacity) {
            return false;
        }
        buffer_[head & (kCapacity - 1)] = s;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer thread only.
    bool tryPop(vehicle::TruthSnapshot& out) noexcept {
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
    std::vector<vehicle::TruthSnapshot> buffer_;
    // Producer and consumer indices each get their own cache line, away from the
    // buffer pointer (alignas pads the class, hence the C4324 suppression).
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

}  // namespace sim::server
