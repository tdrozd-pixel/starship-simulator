#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace sim::server {

struct HelloInfo {
    uint32_t runId = 0;
    double timeScale = 1.0;
    std::string vehicleName = "test hopper (placeholder)";
    double vehicleLength = 20.0;  // [m]
    double vehicleRadius = 4.5;   // [m]
};

std::string makeHello(const HelloInfo& info);
std::string makeRunStarted(uint32_t runId);

enum class ClientMsg { Unknown, Restart };

// Minimal scan for the "type" string member; no JSON library.
ClientMsg parseClientMessage(std::string_view text);

}  // namespace sim::server
