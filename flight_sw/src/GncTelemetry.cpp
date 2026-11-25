#include "fsw/GncTelemetry.hpp"

#include <algorithm>

#include "fsw/Ccsds.hpp"
#include "fsw/TelemetryPackets.hpp"

namespace fsw {
namespace {

using namespace ccsds;

bool due(uint32_t cycle, uint32_t period) { return (cycle % period) == 0; }

float toF32(double value) { return static_cast<float>(value); }

}  // namespace

uint8_t* GncTelemetry::begin(Slot slot, uint16_t apidValue, std::size_t userLength, double time) {
    const auto dataLength = static_cast<uint16_t>(kSecondaryHeaderSize + userLength);
    writePrimaryHeader(buffer_, apidValue, sequence_[slot], dataLength);
    sequence_[slot] = static_cast<uint16_t>((sequence_[slot] + 1U) % (kMaxSequenceCount + 1U));
    writeCucTime(buffer_ + kPrimaryHeaderSize, time);
    return buffer_ + kHeaderSize;
}

void GncTelemetry::send(Hal& hal, std::size_t userLength) { hal.downlink(buffer_, kHeaderSize + userLength); }

void GncTelemetry::emit(Hal& hal, const GncTelemetryInput& in, uint32_t cycle, double time) {
    namespace tm = telemetry;

    if (due(cycle, tm::kFswStatusPeriodCycles)) {
        tm::FswStatus s;
        s.mode = static_cast<uint8_t>(in.mode);
        s.enginesOn = in.enginesOn;
        s.navValid = in.horizontalNavValid;
        s.translating = in.translating;
        s.aligned = in.alignedOverTarget;
        s.gnssReady = in.gnssReady;
        s.gnssFixCount = static_cast<uint16_t>(std::clamp(in.gnssFixCount, 0, 65535));
        s.cycleCount = cycle;
        s.timeInMode = toF32(in.timeInMode);
        s.propellant = toF32(in.propellantEstimate);
        s.mass = toF32(in.massEstimate);
        tm::encode(s, begin(kStatus, apid::kFswStatus, kFswStatusLength, time));
        send(hal, kFswStatusLength);
    }

    if (due(cycle, tm::kMissionConfigPeriodCycles)) {
        tm::MissionConfig c;
        c.targetLatitude = in.targetLatitude;
        c.targetLongitude = in.targetLongitude;
        c.targetHeight = in.targetHeight;
        c.targetAltitude = toF32(in.targetAltitude);
        c.alignTolerance = toF32(in.landingAlignTolerance);
        c.translateAltitude = toF32(in.translateAltitude);
        tm::encode(c, begin(kConfig, apid::kMissionConfig, kMissionConfigLength, time));
        send(hal, kMissionConfigLength);
    }

    if (due(cycle, tm::kNavigationPeriodCycles)) {
        tm::Navigation n;
        for (std::size_t i = 0; i < n.attitude.size(); ++i) {
            n.attitude[i] = toF32(in.attitude[i]);
        }
        for (std::size_t i = 0; i < 3; ++i) {
            n.angularRate[i] = toF32(in.imu.angularRate[i]);
            n.specificForce[i] = toF32(in.imu.specificForce[i]);
        }
        n.altitude = toF32(in.altitude);
        n.verticalVelocity = toF32(in.verticalVelocity);
        n.altimeterRange = toF32(in.altimeter.range);
        n.altimeterValid = in.altimeter.valid;
        n.east = toF32(in.east);
        n.north = toF32(in.north);
        n.velocityEast = toF32(in.velocityEast);
        n.velocityNorth = toF32(in.velocityNorth);
        n.accelBiasEast = toF32(in.accelBiasEast);
        n.accelBiasNorth = toF32(in.accelBiasNorth);
        n.positionSigmaEast = toF32(in.positionSigmaEast);
        n.positionSigmaNorth = toF32(in.positionSigmaNorth);
        n.velocitySigmaEast = toF32(in.velocitySigmaEast);
        n.velocitySigmaNorth = toF32(in.velocitySigmaNorth);
        n.gnssInnovationEast = toF32(in.gnssInnovationEast);
        n.gnssInnovationNorth = toF32(in.gnssInnovationNorth);
        n.altimeterInnovation = toF32(in.altimeterInnovation);
        n.gnssUpdateAge = toF32(in.gnssUpdateAge);
        n.altimeterUpdateAge = toF32(in.altimeterUpdateAge);
        tm::encode(n, begin(kNav, apid::kNavigation, kNavigationLength, time));
        send(hal, kNavigationLength);
    }

    if (due(cycle, tm::kGuidanceControlPeriodCycles)) {
        tm::GuidanceControl g;
        g.altitudeSetpoint = toF32(in.altitudeSetpoint);
        g.velocitySetpoint = toF32(in.velocitySetpoint);
        g.eastSetpoint = toF32(in.eastSetpoint);
        g.northSetpoint = toF32(in.northSetpoint);
        g.eastAccelCommand = toF32(in.eastAccelCommand);
        g.northAccelCommand = toF32(in.northAccelCommand);
        for (std::size_t i = 0; i < 3; ++i) {
            g.attitudeError[i] = toF32(in.attitudeError[i]);
        }
        for (std::size_t i = 0; i < g.engines.size(); ++i) {  // sea-level engines only
            const EngineCommand& e = in.command.engines[i];
            g.engines[i].on = e.on;
            g.engines[i].throttle = toF32(e.throttle);
            g.engines[i].pitch = toF32(in.command.gimbals[i].pitch);
            g.engines[i].yaw = toF32(in.command.gimbals[i].yaw);
        }
        tm::encode(g, begin(kGuidance, apid::kGuidanceControl, kGuidanceControlLength, time));
        send(hal, kGuidanceControlLength);
    }

    if (due(cycle, tm::kActuatorFeedbackPeriodCycles)) {
        tm::ActuatorFeedback a;
        for (std::size_t i = 0; i < a.engines.size(); ++i) {
            a.engines[i].valid = in.gimbal.valid[i];
            a.engines[i].pitch = toF32(in.gimbal.pitch[i]);
            a.engines[i].yaw = toF32(in.gimbal.yaw[i]);
        }
        tm::encode(a, begin(kActuators, apid::kActuatorFeedback, kActuatorFeedbackLength, time));
        send(hal, kActuatorFeedbackLength);
    }

    if (due(cycle, tm::kEngineStatusPeriodCycles)) {
        tm::EngineStatus e;
        for (std::size_t i = 0; i < e.engines.size(); ++i) {
            const EngineCommand& c = in.command.engines[i];
            e.engines[i].on = c.on;
            e.engines[i].state = static_cast<uint8_t>(in.engines.state[i]);
            e.engines[i].valid = in.engines.valid[i];
            e.engines[i].command = toF32(c.on ? c.throttle : 0.0);
            e.engines[i].throttle = toF32(in.engines.throttle[i]);
        }
        tm::encode(e, begin(kEngineSlot, apid::kEngineStatus, kEngineStatusLength, time));
        send(hal, kEngineStatusLength);
    }

    if (tm::kGnssOnNewFix && in.gnssNew) {
        tm::Gnss g;
        g.fixType = static_cast<uint8_t>(in.gnss.fixType);
        g.satellitesUsed = static_cast<uint8_t>(std::clamp(in.gnss.satellitesUsed, 0, 255));
        g.satellitesVisible = static_cast<uint8_t>(std::clamp(in.gnss.satellitesVisible, 0, 255));
        g.fixTime = in.gnss.fixTime;
        g.latitude = in.gnss.latitude;
        g.longitude = in.gnss.longitude;
        g.height = in.gnss.height;
        for (std::size_t i = 0; i < 3; ++i) {
            g.velocity[i] = toF32(in.gnss.velocityEnu[i]);
        }
        g.horizontalSigma = toF32(in.gnss.horizontalSigma);
        g.verticalSigma = toF32(in.gnss.verticalSigma);
        tm::encode(g, begin(kGnssSlot, apid::kGnss, kGnssLength, time));
        send(hal, kGnssLength);
    }

    const CatchTelemetryInput& c = in.catchInfo;
    if (due(cycle, tm::kCatchStatusPeriodCycles)) {
        tm::CatchStatus s;
        s.phase = static_cast<uint8_t>(c.phase);
        s.abortReason = static_cast<uint8_t>(c.abortReason);
        s.armsExpected = static_cast<uint8_t>(c.armsExpected);
        s.go = c.go;
        s.positionOk = c.positionOk;
        s.speedOk = c.speedOk;
        s.tiltOk = c.tiltOk;
        s.rtkOk = c.rtkOk;
        s.corridorOk = c.corridorOk;
        s.aborted = c.aborted;
        s.alongRemaining = toF32(c.alongRemaining);
        s.lateralError = toF32(c.lateralError);
        s.pinHeightAboveSeat = toF32(c.pinHeightAboveSeat);
        s.alongSpeed = toF32(c.alongSpeed);
        s.lateralSpeed = toF32(c.lateralSpeed);
        s.sinkRate = toF32(c.sinkRate);
        s.tilt = toF32(c.tilt);
        s.lateralLimit = toF32(c.lateralLimit);
        s.timeToSeat = toF32(c.timeToSeat);
        tm::encode(s, begin(kCatchStatusSlot, apid::kCatchStatus, kCatchStatusLength, time));
        send(hal, kCatchStatusLength);
    }

    if (due(cycle, tm::kCatchConfigPeriodCycles)) {
        tm::CatchConfig k;
        k.landingMode = static_cast<uint8_t>(c.landingMode);
        for (std::size_t i = 0; i < k.seatPoint.size(); ++i) {
            k.seatPoint[i] = toF32(c.seatPoint[i]);
        }
        for (std::size_t i = 0; i < k.approachDir.size(); ++i) {
            k.approachDir[i] = toF32(c.approachDir[i]);
            k.towerPosition[i] = toF32(c.towerPosition[i]);
        }
        k.stagingDistance = toF32(c.stagingDistance);
        k.mastHalfWidth = toF32(c.mastHalfWidth);
        k.mastHeight = toF32(c.mastHeight);
        k.hingeHalfSpacing = toF32(c.hingeHalfSpacing);
        k.hingeSetback = toF32(c.hingeSetback);
        k.armLength = toF32(c.armLength);
        k.armOpenAngle = toF32(c.armOpenAngle);
        k.armClosedAngle = toF32(c.armClosedAngle);
        k.windowLateral = toF32(c.windowLateral);
        k.windowAlong = toF32(c.windowAlong);
        k.windowSink = toF32(c.windowSink);
        k.windowTilt = toF32(c.windowTilt);
        tm::encode(k, begin(kCatchConfigSlot, apid::kCatchConfig, kCatchConfigLength, time));
        send(hal, kCatchConfigLength);
    }
}

}  // namespace fsw
