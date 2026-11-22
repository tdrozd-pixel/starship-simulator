#include "fsw/GncTelemetry.hpp"

#include <algorithm>

#include "fsw/Ccsds.hpp"

namespace fsw {
namespace {

using namespace ccsds;

uint8_t* putF32(uint8_t* p, double value) { return p + writeF32(p, static_cast<float>(value)); }

uint8_t* putF64(uint8_t* p, double value) { return p + writeF64(p, value); }

uint8_t* putU8(uint8_t* p, uint8_t value) { return p + writeU8(p, value); }

uint8_t* putPad(uint8_t* p, int count) {
    for (int i = 0; i < count; ++i) {
        *p++ = 0;
    }
    return p;
}

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
    const bool every50Hz = (cycle % 2U) == 0;
    const bool every10Hz = (cycle % 10U) == 0;
    const bool every1Hz = (cycle % 100U) == 0;

    if (every10Hz) {
        uint8_t* p = begin(kStatus, apid::kFswStatus, kFswStatusLength, time);
        const uint8_t flags = static_cast<uint8_t>((in.enginesOn ? 1U : 0U) | (in.horizontalNavValid ? 2U : 0U) |
                                                   (in.translating ? 4U : 0U) | (in.alignedOverTarget ? 8U : 0U) |
                                                   (in.gnssReady ? 16U : 0U));
        p = putU8(p, static_cast<uint8_t>(in.mode));
        p = putU8(p, flags);
        p += writeU16(p, static_cast<uint16_t>(std::clamp(in.gnssFixCount, 0, 65535)));
        p += writeU32(p, cycle);
        p = putF32(p, in.timeInMode);
        p = putF32(p, in.propellantEstimate);
        putF32(p, in.massEstimate);
        send(hal, kFswStatusLength);
    }

    if (every1Hz) {
        uint8_t* p = begin(kConfig, apid::kMissionConfig, kMissionConfigLength, time);
        p = putF64(p, in.targetLatitude);
        p = putF64(p, in.targetLongitude);
        p = putF64(p, in.targetHeight);
        p = putF32(p, in.targetAltitude);
        p = putF32(p, in.landingAlignTolerance);
        putF32(p, in.translateAltitude);
        send(hal, kMissionConfigLength);
    }

    if (every50Hz) {
        uint8_t* p = begin(kNav, apid::kNavigation, kNavigationLength, time);
        for (double q : in.attitude) {
            p = putF32(p, q);
        }
        for (double w : in.imu.angularRate) {
            p = putF32(p, w);
        }
        for (double f : in.imu.specificForce) {
            p = putF32(p, f);
        }
        p = putF32(p, in.altitude);
        p = putF32(p, in.verticalVelocity);
        p = putF32(p, in.altimeter.range);
        p = putU8(p, in.altimeter.valid ? 1 : 0);
        p = putPad(p, 3);
        p = putF32(p, in.east);
        p = putF32(p, in.north);
        p = putF32(p, in.velocityEast);
        p = putF32(p, in.velocityNorth);
        p = putF32(p, in.accelBiasEast);
        putF32(p, in.accelBiasNorth);
        send(hal, kNavigationLength);

        p = begin(kGuidance, apid::kGuidanceControl, kGuidanceControlLength, time);
        p = putF32(p, in.altitudeSetpoint);
        p = putF32(p, in.velocitySetpoint);
        p = putF32(p, in.eastSetpoint);
        p = putF32(p, in.northSetpoint);
        p = putF32(p, in.eastAccelCommand);
        p = putF32(p, in.northAccelCommand);
        for (double e : in.attitudeError) {
            p = putF32(p, e);
        }
        for (const EngineCommand& e : in.command.engines) {
            p = putU8(p, e.on ? 1 : 0);
            p = putPad(p, 3);
            p = putF32(p, e.throttle);
            p = putF32(p, e.gimbalPitch);
            p = putF32(p, e.gimbalYaw);
        }
        send(hal, kGuidanceControlLength);

        p = begin(kActuators, apid::kActuatorFeedback, kActuatorFeedbackLength, time);
        for (int i = 0; i < kEngineCount; ++i) {
            p = putU8(p, in.gimbal.valid[i] ? 1 : 0);
            p = putPad(p, 3);
            p = putF32(p, in.gimbal.pitch[i]);
            p = putF32(p, in.gimbal.yaw[i]);
        }
        send(hal, kActuatorFeedbackLength);
    }

    if (in.gnssNew) {
        uint8_t* p = begin(kGnssSlot, apid::kGnss, kGnssLength, time);
        p = putU8(p, static_cast<uint8_t>(in.gnss.fixType));
        p = putU8(p, static_cast<uint8_t>(std::clamp(in.gnss.satellitesUsed, 0, 255)));
        p = putU8(p, static_cast<uint8_t>(std::clamp(in.gnss.satellitesVisible, 0, 255)));
        p = putPad(p, 1);
        p = putF64(p, in.gnss.fixTime);
        p = putF64(p, in.gnss.latitude);
        p = putF64(p, in.gnss.longitude);
        p = putF64(p, in.gnss.height);
        for (double v : in.gnss.velocityEnu) {
            p = putF32(p, v);
        }
        p = putF32(p, in.gnss.horizontalSigma);
        putF32(p, in.gnss.verticalSigma);
        send(hal, kGnssLength);
    }
}

}  // namespace fsw
