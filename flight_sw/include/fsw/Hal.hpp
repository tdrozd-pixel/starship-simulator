#pragma once

// Hardware abstraction layer: the ONLY way flight software touches the world.
// In simulation it is implemented by sim::vehicle::SimHal; on hardware it would
// wrap real drivers. Flight software never reads simulator truth.

#include <cstddef>
#include <cstdint>

namespace fsw {

// Engines this flight computer controls. Index 0..2 are the three sea-level Raptors
// (inner ring, gimbaled); index 3..5 are the three Raptor Vacuums (outer ring, fixed).
constexpr int kSeaLevelEngineCount = 3;  // gimbaled
constexpr int kRvacCount = 3;            // not gimbaled
constexpr int kEngineCount = kSeaLevelEngineCount + kRvacCount;
constexpr int kFirstRvac = kSeaLevelEngineCount;  // index of the first RVac

struct ImuSample {
    double stamp = 0.0;  // [s]
    bool valid = false;
    double specificForce[3] = {0.0, 0.0, 0.0};  // body frame [m/s^2]
    double angularRate[3] = {0.0, 0.0, 0.0};    // body frame [rad/s]
};

struct AltimeterSample {
    double stamp = 0.0;  // [s]
    bool valid = false;
    double range = 0.0;  // slant range along the body -z axis to the ground [m]
};

enum class GnssFixType : int {
    None = 0,
    Standalone = 1,  // single-point GNSS, meter level
    RtkFixed = 2,    // carrier-phase differential, centimeter level
};

// One GNSS navigation solution at the antenna phase center.
struct GnssSample {
    double stamp = 0.0;    // when the solution reached the flight computer [s]
    double fixTime = 0.0;  // time of validity of the solution [s] (receivers report it)
    bool valid = false;
    GnssFixType fixType = GnssFixType::None;
    double latitude = 0.0;   // WGS-84 geodetic [rad]
    double longitude = 0.0;  // WGS-84 [rad], east positive
    double height = 0.0;     // above the WGS-84 ellipsoid [m]
    double velocityEnu[3] = {0.0, 0.0, 0.0};  // local east, north, up at the antenna [m/s]
    double horizontalSigma = 0.0;  // receiver's own 1-sigma accuracy estimate [m]
    double verticalSigma = 0.0;    // [m]
    int satellitesVisible = 0;
    int satellitesUsed = 0;
};

// Measured gimbal positions from the gimbal position sensors (sea-level engines only;
// the RVacs have no gimbal).
struct GimbalFeedbackSample {
    double stamp = 0.0;  // [s]
    bool valid[kSeaLevelEngineCount] = {};
    double pitch[kSeaLevelEngineCount] = {};  // about body x [rad]
    double yaw[kSeaLevelEngineCount] = {};    // about body y [rad]
};

// Engine controller state as reported by the engine.
enum class EngineState : int {
    Off = 0,
    Starting = 1,   // commanded on, start sequence in progress
    Running = 2,
    Inhibited = 3,  // commanded on but refused (flow separation or start inhibit)
};

// Measured engine throttle and state, all six engines.
struct EngineFeedbackSample {
    double stamp = 0.0;  // [s]
    bool valid[kEngineCount] = {};
    double throttle[kEngineCount] = {};  // measured actual, fraction of rated flow
    EngineState state[kEngineCount] = {};
};

// Per engine (all six): on/off and the throttle (power setting) command.
struct EngineCommand {
    bool on = false;
    double throttle = 0.0;  // fraction of rated mass flow
};

// Per sea-level engine only (the RVacs are fixed).
struct GimbalCommand {
    double pitch = 0.0;  // [rad], about body x
    double yaw = 0.0;    // [rad], about body y
};

struct ActuatorCommand {
    EngineCommand engines[kEngineCount];
    GimbalCommand gimbals[kSeaLevelEngineCount];
};

struct Telemetry {
    int mode = 0;
    double altitudeEstimate = 0.0;
    double verticalVelocityEstimate = 0.0;
    double altitudeSetpoint = 0.0;
    double velocitySetpoint = 0.0;
    double throttleCommand = 0.0;
    double massEstimate = 0.0;
    // Horizontal navigation in the target's local ENU frame (target at the origin).
    bool horizontalNavValid = false;
    double eastEstimate = 0.0;   // [m]
    double northEstimate = 0.0;  // [m]
    double eastVelocityEstimate = 0.0;   // [m/s]
    double northVelocityEstimate = 0.0;  // [m/s]
    double eastSetpoint = 0.0;   // [m] current horizontal position target
    double northSetpoint = 0.0;  // [m]
    double eastAccelCommand = 0.0;   // [m/s^2] horizontal guidance output
    double northAccelCommand = 0.0;  // [m/s^2]
    int gnssFixType = 0;
};

class Hal {
public:
    virtual ~Hal() = default;

    virtual double timeSec() const = 0;
    virtual ImuSample readImu() = 0;
    virtual AltimeterSample readAltimeter() = 0;
    virtual GnssSample readGnss() = 0;
    virtual GimbalFeedbackSample readGimbalFeedback() = 0;
    virtual EngineFeedbackSample readEngineFeedback() = 0;
    virtual void commandActuators(const ActuatorCommand& command) = 0;
    virtual void publishTelemetry(const Telemetry& telemetry) = 0;
    // Send one complete CCSDS packet down the telemetry link (fire and forget).
    virtual void downlink(const uint8_t* packet, std::size_t length) = 0;
};

}  // namespace fsw
