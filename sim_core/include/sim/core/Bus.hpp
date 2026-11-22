#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace sim {

// A message as last published on a channel.
template <typename T>
struct Message {
    T data{};
    double stamp = 0.0;  // sim time of publication [s]
    uint64_t seq = 0;    // 0 means never published

    bool valid() const { return seq > 0; }
};

class ChannelBase {
public:
    virtual ~ChannelBase() = default;
};

// Latest-value channel. Publishing overwrites; readers compare seq to detect new data.
// No allocation happens on publish, so channels are safe to use inside the step loop.
template <typename T>
class Channel final : public ChannelBase {
public:
    void publish(const T& data, double stamp) {
        msg_.data = data;
        msg_.stamp = stamp;
        ++msg_.seq;
    }

    const Message<T>& latest() const { return msg_; }

private:
    Message<T> msg_;
};

// Models talk to each other only through named, typed channels on the bus.
// Resolve channels once in initialize() and keep the reference: lookups use a
// hash map and must not happen inside the step loop.
class Bus {
public:
    template <typename T>
    Channel<T>& channel(const std::string& name) {
        auto it = channels_.find(name);
        if (it == channels_.end()) {
            auto created = std::make_unique<Channel<T>>();
            Channel<T>* raw = created.get();
            channels_.emplace(name, std::move(created));
            return *raw;
        }
        auto* typed = dynamic_cast<Channel<T>*>(it->second.get());
        if (typed == nullptr) {
            throw std::logic_error("Bus channel '" + name + "' requested with a different message type");
        }
        return *typed;
    }

    bool has(const std::string& name) const { return channels_.count(name) > 0; }

private:
    std::unordered_map<std::string, std::unique_ptr<ChannelBase>> channels_;
};

}  // namespace sim
