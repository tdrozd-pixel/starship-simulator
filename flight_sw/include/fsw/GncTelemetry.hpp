#pragma once

// Builds the GNC telemetry packets (docs/telemetry/gnc_packets.md, generated from
// telemetry/dictionary.json into fsw/TelemetryPackets.hpp) and hands them to the HAL.
// Only flight software knowledge goes in here, never truth.

#include <cstddef>
#include <cstdint>

#include "fsw/Hal.hpp"
#include "fsw/TelemetryPackets.hpp"

namespace fsw {

// APIDs and user data lengths [bytes] come from the generated packet layouts
// (telemetry/dictionary.json); these names are kept for existing users.
namespace apid {
constexpr uint16_t kFswStatus = telemetry::kFswStatusApid;
constexpr uint16_t kMissionConfig = telemetry::kMissionConfigApid;
constexpr uint16_t kNavigation = telemetry::kNavigationApid;
constexpr uint16_t kGuidanceControl = telemetry::kGuidanceControlApid;
constexpr uint16_t kActuatorFeedback = telemetry::kActuatorFeedbackApid;
constexpr uint16_t kEngineStatus = telemetry::kEngineStatusApid;
constexpr uint16_t kGnss = telemetry::kGnssApid;
constexpr uint16_t kCatchStatus = telemetry::kCatchStatusApid;
constexpr uint16_t kCatchConfig = telemetry::kCatchConfigApid;
}  // namespace apid

constexpr std::size_t kFswStatusLength = telemetry::kFswStatusUserDataLength;
constexpr std::size_t kMissionConfigLength = telemetry::kMissionConfigUserDataLength;
constexpr std::size_t kNavigationLength = telemetry::kNavigationUserDataLength;
constexpr std::size_t kGuidanceControlLength = telemetry::kGuidanceControlUserDataLength;
constexpr std::size_t kActuatorFeedbackLength = telemetry::kActuatorFeedbackUserDataLength;
constexpr std::size_t kEngineStatusLength = telemetry::kEngineStatusUserDataLength;
constexpr std::size_t kGnssLength = telemetry::kGnssUserDataLength;
constexpr std::size_t kCatchStatusLength = telemetry::kCatchStatusUserDataLength;
constexpr std::size_t kCatchConfigLength = telemetry::kCatchConfigUserDataLength;

// Downlinked enum values (telemetry/dictionary.json: catchPhase, catchAbort, armsExpected).
enum class CatchPhase : int { Inactive = 0, Staging = 1, Approach = 2, Hold = 3, Final = 4, Caught = 5, Abort = 6 };
enum class CatchAbortReason : int { None = 0, RtkLost = 1, Corridor = 2, Window = 3, Timeout = 4 };
enum class ArmsExpected : int { Open = 0, Closing = 1, Closed = 2 };

// Catch downlink inputs (catch_status and catch_config). In Pad mode only landingMode is set:
// the packets are still sent so the console can show the mode.
struct CatchTelemetryInput {
    int landingMode = 0;  // LandingMode of the mission data
    int phase = 0;        // CatchPhase
    int abortReason = 0;  // CatchAbortReason
    int armsExpected = 0;  // ArmsExpected, from the sequence timer, not measured
    bool go = false;
    bool positionOk = false;
    bool speedOk = false;
    bool tiltOk = false;  // tilt and heading within the window
    bool rtkOk = false;
    bool corridorOk = false;
    bool aborted = false;
    double alongRemaining = 0.0;      // [m] positive = short of the seat
    double lateralError = 0.0;        // [m] positive toward +b = (d_y, -d_x), d = approach direction
    double pinHeightAboveSeat = 0.0;  // [m]
    double alongSpeed = 0.0;          // [m/s]
    double lateralSpeed = 0.0;        // [m/s]
    double sinkRate = 0.0;            // [m/s] positive down
    double tilt = 0.0;                // [rad]
    double lateralLimit = 0.0;        // [m] funnel half-width
    double timeToSeat = -1.0;         // [s] negative: not closing on the seat

    // catch_config (mission data, target frame)
    double seatPoint[3] = {};     // [m]
    double approachDir[2] = {};   // []
    double stagingDistance = 0.0;       // [m]
    double towerPosition[2] = {};       // [m]
    double mastHalfWidth = 0.0;         // [m]
    double mastHeight = 0.0;            // [m]
    double hingeHalfSpacing = 0.0;      // [m]
    double hingeSetback = 0.0;          // [m]
    double armLength = 0.0;             // [m]
    double armOpenAngle = 0.0;          // [rad]
    double armClosedAngle = 0.0;        // [rad]
    double windowLateral = 0.0;         // [m]
    double windowAlong = 0.0;           // [m]
    double windowSink = 0.0;            // [m/s]
    double windowTilt = 0.0;            // [rad]
};

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
    double positionSigmaEast = 0.0;      // [m]
    double positionSigmaNorth = 0.0;     // [m]
    double velocitySigmaEast = 0.0;      // [m/s]
    double velocitySigmaNorth = 0.0;     // [m/s]
    double gnssInnovationEast = 0.0;     // [m]
    double gnssInnovationNorth = 0.0;    // [m]
    double altimeterInnovation = 0.0;    // [m]
    double gnssUpdateAge = -1.0;         // [s] negative: none accepted yet
    double altimeterUpdateAge = -1.0;    // [s] negative: none accepted yet

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
    EngineFeedbackSample engines;  // measured throttle and state, all six engines

    // GNSS: latest solution received, and whether it arrived this cycle
    GnssSample gnss;
    bool gnssNew = false;

    CatchTelemetryInput catchInfo;
};

class GncTelemetry {
public:
    // Sends the packets due on `cycle` (flight computer cycles at 100 Hz, from 0).
    // `time` is flight computer time [s] for the CUC stamp. No allocation.
    void emit(Hal& hal, const GncTelemetryInput& in, uint32_t cycle, double time);

private:
    // Index of each APID's sequence counter.
    enum Slot : std::size_t { kStatus, kConfig, kNav, kGuidance, kActuators, kEngineSlot, kGnssSlot, kCatchStatusSlot,
                kCatchConfigSlot, kSlotCount };

    uint8_t* begin(Slot slot, uint16_t apidValue, std::size_t userLength, double time);
    void send(Hal& hal, std::size_t userLength);

    uint8_t buffer_[256] = {};
    uint16_t sequence_[kSlotCount] = {};
};

}  // namespace fsw
