#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sim/core/Model.hpp"
#include "sim/core/Random.hpp"
#include "sim/vehicle/TelemetrySink.hpp"

namespace sim::vehicle {

// Onboard side of the telemetry link: a fixed ring the flight computer pushes packets into
// (through SimHal::downlink). All storage is allocated in the constructor.
class DownlinkQueue {
public:
    static constexpr std::size_t kCapacity = 512;  // [packets]

    struct Packet {
        uint8_t data[kMaxTelemetryPacket] = {};
        std::size_t length = 0;    // [bytes]
        double enqueueTime = 0.0;  // onboard time the packet was queued [s]
    };

    DownlinkQueue();

    // Returns false (and counts an overflow) when the queue is full or the packet is too long.
    bool push(const uint8_t* packet, std::size_t length, double time);

    bool empty() const { return count_ == 0; }
    std::size_t size() const { return count_; }
    const Packet& front() const { return slots_[head_]; }
    void pop();
    uint64_t overflowed() const { return overflowed_; }

private:
    std::vector<Packet> slots_;
    std::size_t head_ = 0;
    std::size_t count_ = 0;
    uint64_t overflowed_ = 0;
};

// Link parameters. All values are placeholder estimates, not a model of a real link.
// TODO(phase 5, item 29): Starlink links, ground stations; plasma blackout is item 26.
struct DownlinkSpec {
    double bitRate = 2.0e6;          // [bit/s] placeholder
    double latency = 0.060;          // [s] mean one-way delay, placeholder
    double jitter = 0.010;           // [s] uniform +/- around the latency, placeholder
    double lossProbability = 0.001;  // per packet, placeholder
};

// Moves packets from the DownlinkQueue to the TelemetrySink with a bit rate limit, delay,
// jitter (never reordering) and random loss. Runs at 100 Hz.
class DownlinkModel final : public Model {
public:
    static constexpr std::size_t kInFlightCapacity = 512;  // [packets]

    DownlinkModel(std::string name, double rateHz, DownlinkQueue& queue, TelemetrySink& sink, DownlinkSpec spec,
                  uint64_t seed);

    void update(const StepContext& ctx) override;

    uint64_t sent() const { return sent_; }                      // packets put on the link
    uint64_t delivered() const { return delivered_; }            // packets handed to the sink
    uint64_t lost() const { return lost_; }                      // dropped by the link
    uint64_t overflowed() const { return queue_.overflowed(); }  // dropped onboard, queue full

private:
    struct InFlight {
        DownlinkQueue::Packet packet;
        double deliveryTime = 0.0;  // [s]
    };

    DownlinkQueue& queue_;
    TelemetrySink& sink_;
    DownlinkSpec spec_;
    Rng rng_;
    std::vector<InFlight> ring_;
    std::size_t head_ = 0;
    std::size_t count_ = 0;
    double budgetBits_ = 0.0;
    double lastDelivery_ = 0.0;  // [s]
    uint64_t sent_ = 0;
    uint64_t delivered_ = 0;
    uint64_t lost_ = 0;
};

}  // namespace sim::vehicle
