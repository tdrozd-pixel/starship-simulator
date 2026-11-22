#pragma once

// Builds the GNC telemetry packets (docs/telemetry/gnc_packets.md) and hands them to the HAL.
// Only flight software knowledge goes in here, never truth.

#include <cstddef>
#include <cstdint>

#include "fsw/Hal.hpp"

namespace fsw {

namespace apid {
constexpr uint16_t kFswStatus = 0x100;
constexpr uint16_t kMissionConfig = 0x101;
constexpr uint16_t kNavigation = 0x110;
constexpr uint16_t kGuidanceControl = 0x120;
constexpr uint16_t kActuatorFeedback = 0x130;
constexpr uint16_t kGnss = 0x140;
}  // namespace apid

// User data lengths [bytes] per the dictionary.
constexpr std::size_t kFswStatusLength = 20;
constexpr std::size_t kMissionConfigLength = 36;
constexpr std::size_t kNavigationLength = 80;
constexpr std::size_t kGuidanceControlLength = 84;
constexpr std::size_t kActuatorFeedbackLength = 36;
constexpr std::size_t kGnssLength = 56;

// Everything the packets carry, filled by the flight computer each cycle.
struct GncTelemetryInput {
    // Status
    int mode = 0;  // HopMode value
    bool enginesOn = false;
    bool horizontalNavValid = false;
    bool translating = false;
    bool alignedOverTarget = false;
    bool gnssReady = false;
    int gnssFixCount = 0;
    double timeInMode = 0.0;          // [s]
    double propellantEstimate = 0.0;  // [kg]
    double massEstimate = 0.0;        // [kg]

    // Mission config
    double targetLatitude = 0.0;         // [rad]
    double targetLongitude = 0.0;        // [rad]
    double targetHeight = 0.0;           // [m]
    double targetAltitude = 0.0;         // [m]
    double landingAlignTolerance = 0.0;  // [m]
    double translateAltitude = 0.0;      // [m]

    // Navigation
    double attitude[4] = {1.0, 0.0, 0.0, 0.0};  // w, x, y, z
    ImuSample imu;
    double altitude = 0.0;          // [m]
    double verticalVelocity = 0.0;  // [m/s]
    AltimeterSample altimeter;
    double east = 0.0;   // [m]
    double north = 0.0;  // [m]
    double velocityEast = 0.0;   // [m/s]
    double velocityNorth = 0.0;  // [m/s]
    double accelBiasEast = 0.0;   // [m/s^2]
    double accelBiasNorth = 0.0;  // [m/s^2]

    // Guidance and control
    double altitudeSetpoint = 0.0;  // [m]
    double velocitySetpoint = 0.0;  // [m/s]
    double eastSetpoint = 0.0;      // [m]
    double northSetpoint = 0.0;     // [m]
    double eastAccelCommand = 0.0;   // [m/s^2]
    double northAccelCommand = 0.0;  // [m/s^2]
    double attitudeError[3] = {};    // body [rad]
    ActuatorCommand command;

    // Actuator feedback
    GimbalFeedbackSample gimbal;

    // GNSS: latest solution received, and whether it arrived this cycle
    GnssSample gnss;
    bool gnssNew = false;
};

class GncTelemetry {
public:
    // Sends the packets due on `cycle` (flight computer cycles at 100 Hz, from 0).
    // `time` is flight computer time [s] for the CUC stamp. No allocation.
    void emit(Hal& hal, const GncTelemetryInput& in, uint32_t cycle, double time);

private:
    // Index of each APID's sequence counter.
    enum Slot : std::size_t { kStatus, kConfig, kNav, kGuidance, kActuators, kGnssSlot, kSlotCount };

    uint8_t* begin(Slot slot, uint16_t apidValue, std::size_t userLength, double time);
    void send(Hal& hal, std::size_t userLength);

    uint8_t buffer_[256] = {};
    uint16_t sequence_[kSlotCount] = {};
};

}  // namespace fsw
