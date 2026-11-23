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
        for (std::size_t i = 0; i < g.engines.size(); ++i) {
            const EngineCommand& e = in.command.engines[i];
            g.engines[i].on = e.on;
            g.engines[i].throttle = toF32(e.throttle);
            g.engines[i].pitch = toF32(e.gimbalPitch);
            g.engines[i].yaw = toF32(e.gimbalYaw);
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
}

}  // namespace fsw
