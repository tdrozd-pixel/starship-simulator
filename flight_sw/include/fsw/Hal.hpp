#pragma once

// Hardware abstraction layer: the ONLY way flight software touches the world.
// In simulation it is implemented by sim::vehicle::SimHal; on hardware it would
// wrap real drivers. Flight software never reads simulator truth.

namespace fsw {

struct ImuSample {
    double stamp = 0.0;  // [s]
    bool valid = false;
    double specificForce[3] = {0.0, 0.0, 0.0};  // body frame [m/s^2]
    double angularRate[3] = {0.0, 0.0, 0.0};    // body frame [rad/s]
};

struct AltimeterSample {
    double stamp = 0.0;  // [s]
    bool valid = false;
    double range = 0.0;  // distance to ground [m]
};

struct ActuatorCommand {
    bool engineOn = false;
    double throttle = 0.0;     // fraction of max thrust
    double gimbalPitch = 0.0;  // [rad], about body x
    double gimbalYaw = 0.0;    // [rad], about body y
};

struct Telemetry {
    int mode = 0;
    double altitudeEstimate = 0.0;
    double verticalVelocityEstimate = 0.0;
    double altitudeSetpoint = 0.0;
    double velocitySetpoint = 0.0;
    double throttleCommand = 0.0;
    double massEstimate = 0.0;
};

class Hal {
public:
    virtual ~Hal() = default;

    virtual double timeSec() const = 0;
    virtual ImuSample readImu() = 0;
    virtual AltimeterSample readAltimeter() = 0;
    virtual void commandActuators(const ActuatorCommand& command) = 0;
    virtual void publishTelemetry(const Telemetry& telemetry) = 0;
};

}  // namespace fsw
