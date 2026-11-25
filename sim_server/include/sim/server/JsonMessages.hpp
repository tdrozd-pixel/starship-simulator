#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace sim::server {

// Catch tower geometry for the hello message (tower frame conventions in docs/protocol/truth_stream.md).
// Filled from sim::vehicle::TowerParams and HopperParams by the caller (sim_server/main.cpp).
struct TowerInfo {
    double position[2] = {60.0, 0.0};      // [m] tower center, world east, north
    double approachDir[2] = {-1.0, 0.0};   // [] unit vector from the tower toward the approach side, east, north
    double mastHalfWidth = 6.0;            // [m]
    double mastHeight = 145.0;             // [m]
    double hingeHalfSpacing = 8.0;         // [m] lateral, either side of the approach axis
    double hingeSetback = 6.0;             // [m] along approachDir from the tower center
    double armLength = 16.0;               // [m]
    double armHalfWidth = 1.0;             // [m]
    double armThickness = 1.5;             // [m]
    double railHeight = 75.0;              // [m] arm top surface above the origin plane
    double armOpenAngle = 0.349066;        // [rad] open arm direction, outward from approachDir (positive outward)
    double armClosedAngle = 0.253073;      // [rad] closed arm direction, inward from approachDir (positive inward)
    double pinHeight = 30.0;               // [m] Ship catch pin above the base
    double pinOffset = 5.0;                // [m] Ship catch pin lateral offset from the axis
};

struct HelloInfo {
    uint32_t runId = 0;
    double timeScale = 1.0;
    std::string vehicleName = "Starship V3 Ship, hop configuration";
    double vehicleLength = 20.0;  // [m]
    double vehicleRadius = 4.5;   // [m]
    TowerInfo tower;
};

std::string makeHello(const HelloInfo& info);
std::string makeRunStarted(uint32_t runId);

// Hello of the /ws/telemetry stream (docs/protocol/telemetry_stream.md).
std::string makeTelemetryHello(uint32_t runId, double timeScale);

enum class ClientMsg { Unknown, Restart };

// Minimal scan for the "type" string member; no JSON library.
ClientMsg parseClientMessage(std::string_view text);

}  // namespace sim::server
