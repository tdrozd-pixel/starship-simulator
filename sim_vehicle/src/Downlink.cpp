#include "sim/vehicle/Downlink.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace sim::vehicle {

DownlinkQueue::DownlinkQueue() : slots_(kCapacity) {}

bool DownlinkQueue::push(const uint8_t* packet, std::size_t length, double time) {
    if (count_ >= kCapacity || length > kMaxTelemetryPacket) {
        ++overflowed_;
        return false;
    }
    Packet& slot = slots_[(head_ + count_) % kCapacity];
    std::memcpy(slot.data, packet, length);
    slot.length = length;
    slot.enqueueTime = time;
    ++count_;
    return true;
}

void DownlinkQueue::pop() {
    if (count_ > 0) {
        head_ = (head_ + 1) % kCapacity;
        --count_;
    }
}

DownlinkModel::DownlinkModel(std::string name, double rateHz, DownlinkQueue& queue, TelemetrySink& sink,
                             DownlinkSpec spec, uint64_t seed)
    : Model(std::move(name), rateHz), queue_(queue), sink_(sink), spec_(spec), rng_(seed), ring_(kInFlightCapacity) {}

void DownlinkModel::update(const StepContext& ctx) {
    const double now = ctx.t;

    // Transmit as much as the bit budget allows; leftover budget carries over.
    budgetBits_ += spec_.bitRate / rateHz();
    while (!queue_.empty() && count_ < kInFlightCapacity) {
        const DownlinkQueue::Packet& front = queue_.front();
        const double bits = 8.0 * static_cast<double>(front.length);
        if (bits > budgetBits_) {
            break;
        }
        budgetBits_ -= bits;
        ++sent_;
        const bool dropped = rng_.uniform() < spec_.lossProbability;
        const double jitter = (2.0 * rng_.uniform() - 1.0) * spec_.jitter;
        if (dropped) {
            ++lost_;
        } else {
            InFlight& slot = ring_[(head_ + count_) % kInFlightCapacity];
            slot.packet = front;
            // Never earlier than the previous packet: the link keeps order.
            slot.deliveryTime = std::max(lastDelivery_, now + spec_.latency + jitter);
            lastDelivery_ = slot.deliveryTime;
            ++count_;
        }
        queue_.pop();
    }
    if (queue_.empty()) {
        budgetBits_ = 0.0;  // an idle link does not bank capacity
    }

    while (count_ > 0 && ring_[head_].deliveryTime <= now) {
        const InFlight& slot = ring_[head_];
        sink_.deliver(slot.packet.data, slot.packet.length, now);
        ++delivered_;
        head_ = (head_ + 1) % kInFlightCapacity;
        --count_;
    }
}

}  // namespace sim::vehicle
