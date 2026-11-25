// Catch tower tests: catch flight software against a scripted fake Hal. The rig plays the part of
// the sensors only (altitude, position and fix type are set by the test); these tests cover the
// mode logic and the downlinked packets, not flight performance.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "TestFramework.hpp"
#include "fsw/Ccsds.hpp"
#include "fsw/GncTelemetry.hpp"
#include "fsw/HopFlightComputer.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;                    // [m/s^2]
constexpr double kTickDt = 0.01;                        // [s] flight software cycle
constexpr double kEquatorialRadius = 6378137.0;         // [m] WGS-84
constexpr double kEccentricitySquared = 0.00669437999;  // WGS-84
constexpr int kGnssEveryTicks = 10;                     // 10 Hz GNSS
constexpr double kTestApexAltitude = 60.0;              // [m] Ascent target for the rig

class RigHal final : public fsw::Hal {
public:
    double timeSec() const override { return time; }
    fsw::ImuSample readImu() override { return imu; }
    fsw::AltimeterSample readAltimeter() override { return altimeter; }
    fsw::GnssSample readGnss() override { return gnss; }
    void commandActuators(const fsw::ActuatorCommand& value) override { command = value; }
    fsw::GimbalFeedbackSample readGimbalFeedback() override { return gimbal; }
    fsw::EngineFeedbackSample readEngineFeedback() override { return engineFeedback; }
    void publishTelemetry(const fsw::Telemetry&) override {}
    void downlink(const uint8_t* packet, std::size_t length) override { packets.emplace_back(packet, packet + length); }

    double time = 0.0;
    fsw::ImuSample imu;
    fsw::AltimeterSample altimeter;
    fsw::GnssSample gnss;
    fsw::ActuatorCommand command;
    fsw::GimbalFeedbackSample gimbal;
    fsw::EngineFeedbackSample engineFeedback;
    std::vector<std::vector<uint8_t>> packets;
};

fsw::HopConfig rigConfig(fsw::LandingMode landing) {
    fsw::HopConfig config;
    config.ignitionTime = 0.0;
    config.gnssFixesRequired = 1;
    config.target = {0.4537, -1.6957, -22.0};
    config.targetAltitude = kTestApexAltitude;
    config.dryMass = 150000.0;
    config.initialPropellant = 450000.0;
    config.vacuumThrust = 2.88e6;
    config.nozzleExitArea = 1.33;
    config.vacuumIsp = 350.0;
    config.minThrottle = 0.4;
    config.engineLeverArm = 9.5;
    config.engineClusterRadius = 1.55;
    config.engineAzimuth0 = kPi / 2.0;
    config.gimbalLimit = 15.0 * kPi / 180.0;
    config.transverseInertiaDry = 4.7e7;
    config.transverseInertiaPropellant = 1.2e7;
    config.axialInertiaDry = 3.0e6;
    config.axialInertiaPropellant = 4.6e6;
    config.landingMode = landing;
    return config;
}

// Scripted sensors: the altimeter reads `altitude`, GNSS reads (east, north) with `fix`.
// `altitudeTarget` and `altitudeRate` move the altitude a little each tick.
struct Rig {
    explicit Rig(fsw::LandingMode landing) : config(rigConfig(landing)), computer(hal, config) {
        hal.imu.valid = true;
        hal.imu.specificForce[2] = kGravity;
        hal.altimeter.valid = true;
    }

    void tick() {
        ++ticks;
        hal.time = kTickDt * ticks;
        hal.imu.stamp = hal.time;
        const double step = altitudeRate * kTickDt;
        altitude += std::clamp(altitudeTarget - altitude, -step, step);
        hal.altimeter.stamp = hal.time;
        hal.altimeter.range = altitude;
        if (ticks % kGnssEveryTicks == 0) {
            const double latitude = config.target.latitude;
            const double s = std::sin(latitude);
            const double denominator = 1.0 - kEccentricitySquared * s * s;
            const double primeVertical = kEquatorialRadius / std::sqrt(denominator);
            const double meridional = kEquatorialRadius * (1.0 - kEccentricitySquared) / (denominator * std::sqrt(denominator));
            hal.gnss.valid = true;
            hal.gnss.stamp = hal.time;
            hal.gnss.fixTime = hal.time;
            hal.gnss.fixType = fix;
            hal.gnss.latitude = latitude + north / meridional;
            hal.gnss.longitude = config.target.longitude + east / (primeVertical * std::cos(latitude));
            hal.gnss.height = config.target.height;
            hal.gnss.horizontalSigma = 0.02;
        }
        computer.tick();
        modesSeen |= 1U << static_cast<unsigned>(computer.mode());
    }

    void run(double seconds) {
        const int count = static_cast<int>(std::lround(seconds / kTickDt));
        for (int i = 0; i < count; ++i) {
            tick();
        }
    }

    // Runs until `mode` is reached; false if it takes longer than `limit` seconds.
    bool runUntil(fsw::HopMode mode, double limit) {
        const int count = static_cast<int>(std::lround(limit / kTickDt));
        for (int i = 0; i < count && computer.mode() != mode; ++i) {
            tick();
        }
        return computer.mode() == mode;
    }

    void place(double e, double n, double alt) {
        east = e;
        north = n;
        altitude = alt;
        altitudeTarget = alt;
    }

    // Lift off, climb, and arrive in Descent toward the seat point.
    bool flyToDescent() {
        place(0.0, 0.0, 0.0);
        run(0.1);
        altitude = 1.0;  // liftoff
        altitudeTarget = kTestApexAltitude;
        altitudeRate = 100.0;
        return runUntil(fsw::HopMode::Descent, 10.0);
    }

    // Height of the base above the seat altitude [m].
    double heightAboveSeat() const { return altitude - config.catchConfig.seat[2]; }

    // Descent over the seat point down to the entry height (what the catch needs to go on).
    bool flyToApproach() {
        if (!flyToDescent()) {
            return false;
        }
        const fsw::CatchConfig& c = config.catchConfig;
        place(c.seat[0], c.seat[1], c.seat[2] + c.entryHeight + 0.5 * c.entryAltitudeBand);
        return runUntil(fsw::HopMode::CatchApproach, 30.0);
    }

    fsw::HopConfig config;
    RigHal hal;
    fsw::HopFlightComputer computer;
    int ticks = 0;
    double altitude = 0.0;        // [m]
    double altitudeTarget = 0.0;  // [m]
    double altitudeRate = 0.0;    // [m/s]
    double east = 0.0;            // [m]
    double north = 0.0;           // [m]
    fsw::GnssFixType fix = fsw::GnssFixType::RtkFixed;
    unsigned modesSeen = 0;
};

constexpr unsigned kCatchModeMask = (1U << 7) | (1U << 8) | (1U << 9) | (1U << 10);

// Last packet with `apid`, or null.
const std::vector<uint8_t>* lastPacket(const RigHal& hal, uint16_t apid) {
    const std::vector<uint8_t>* found = nullptr;
    for (const auto& p : hal.packets) {
        if (fsw::ccsds::readPrimaryHeader(p.data()).apid == apid) {
            found = &p;
        }
    }
    return found;
}

float readF32(const uint8_t* p) {
    const uint32_t bits = (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                          (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

const uint8_t* userData(const std::vector<uint8_t>& packet) { return packet.data() + fsw::ccsds::kHeaderSize; }

}  // namespace

TEST_CASE(catch_modes_have_names) {
    CHECK(std::strcmp(fsw::toString(fsw::HopMode::CatchApproach), "CatchApproach") == 0);
    CHECK(std::strcmp(fsw::toString(fsw::HopMode::CatchHold), "CatchHold") == 0);
    CHECK(std::strcmp(fsw::toString(fsw::HopMode::CatchFinal), "CatchFinal") == 0);
    CHECK(std::strcmp(fsw::toString(fsw::HopMode::Caught), "Caught") == 0);
    CHECK(static_cast<int>(fsw::HopMode::CatchApproach) == 7);
    CHECK(static_cast<int>(fsw::HopMode::Caught) == 10);
}

TEST_CASE(catch_flight_stages_then_approaches_the_seat) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToDescent());
    // Ascent hands over to a Descent aimed at the seat point (no staging point beside the tower), not the pad.
    CHECK_NEAR(rig.computer.horizontal().east(), 0.0, 0.5);
    CHECK(rig.computer.landingMode() == fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    CHECK(!rig.computer.catchAborted());
}

TEST_CASE(standalone_fix_in_a_catch_phase_aborts_to_the_pad_fallback) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    rig.run(3.0);
    CHECK(rig.computer.mode() == fsw::HopMode::CatchApproach);  // RTK fixed: no abort

    rig.fix = fsw::GnssFixType::Standalone;
    rig.run(0.5 * rig.config.catchConfig.fixLossTime);
    CHECK(rig.computer.mode() == fsw::HopMode::CatchApproach);  // inside the allowance
    rig.run(0.5 * rig.config.catchConfig.fixLossTime + 0.5);
    CHECK(rig.computer.catchAborted());
    CHECK(rig.computer.landingMode() == fsw::LandingMode::Pad);
    CHECK(rig.computer.mode() == fsw::HopMode::Descent);

    // The downlink says so.
    rig.run(0.2);
    const std::vector<uint8_t>* status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        const uint8_t* u = userData(*status);
        CHECK(u[0] == static_cast<uint8_t>(fsw::CatchPhase::Abort));
        CHECK(u[1] == static_cast<uint8_t>(fsw::CatchAbortReason::RtkLost));
        CHECK((u[3] & 0x40) != 0);  // aborted flag
        CHECK((u[3] & 0x10) == 0);  // rtkOk clear
    }
}

TEST_CASE(pad_config_never_enters_the_catch_modes) {
    Rig rig(fsw::LandingMode::Pad);
    CHECK(rig.flyToDescent());
    // The same inputs that bring a catch flight into CatchApproach, Hold and Final.
    const fsw::CatchConfig& c = rig.config.catchConfig;
    rig.place(c.seat[0], c.seat[1], c.seat[2] + c.entryHeight + 0.5 * c.entryAltitudeBand);
    rig.run(20.0);
    rig.place(c.seat[0], c.seat[1], c.seat[2] + 0.5 * c.armTriggerHeight);
    rig.run(20.0);
    CHECK((rig.modesSeen & kCatchModeMask) == 0);
    CHECK(rig.computer.landingMode() == fsw::LandingMode::Pad);
    CHECK(!rig.computer.catchAborted());
}

TEST_CASE(catch_sequence_approach_hold_final_caught) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    const fsw::CatchConfig& c = rig.config.catchConfig;

    // Arms are not expected to be closing above the trigger height.
    rig.run(0.5);
    const std::vector<uint8_t>* status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        CHECK(userData(*status)[2] == static_cast<uint8_t>(fsw::ArmsExpected::Open));
    }

    // Down to the arms-closing height: Hold, arms expected closing, then closed after the closing time.
    rig.altitudeTarget = c.seat[2] + c.armTriggerHeight - 1.0;
    rig.altitudeRate = 8.0;
    CHECK(rig.runUntil(fsw::HopMode::CatchHold, 15.0));
    rig.run(1.0);
    status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        CHECK(userData(*status)[0] == static_cast<uint8_t>(fsw::CatchPhase::Hold));
        CHECK(userData(*status)[2] == static_cast<uint8_t>(fsw::ArmsExpected::Closing));
    }
    rig.run(c.armsClosingTime);
    status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(rig.computer.mode() == fsw::HopMode::CatchHold);
    if (status != nullptr) {
        CHECK(userData(*status)[2] == static_cast<uint8_t>(fsw::ArmsExpected::Closed));
    }

    // Final below the final height, then contact stops it.
    rig.altitudeTarget = c.seat[2] + 0.5 * c.finalHeight;
    rig.altitudeRate = 3.0;  // inside the sink gate (profile plus margin) in the last metres
    CHECK(rig.runUntil(fsw::HopMode::CatchFinal, 25.0));
    rig.altitudeTarget = c.seat[2];
    rig.altitudeRate = c.contactSinkRate;
    CHECK(rig.runUntil(fsw::HopMode::Caught, 30.0));
    rig.run(0.5);
    for (int i = 0; i < fsw::kEngineCount; ++i) {
        CHECK(!rig.hal.command.engines[i].on);
    }
    status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        CHECK(userData(*status)[0] == static_cast<uint8_t>(fsw::CatchPhase::Caught));
        CHECK(userData(*status)[2] == static_cast<uint8_t>(fsw::ArmsExpected::Closed));
        CHECK((userData(*status)[3] & 0x40) == 0);  // not aborted
    }
}

TEST_CASE(catch_gates_are_looser_than_the_window_but_not_unbounded) {
    const fsw::CatchConfig c;
    // At the seat the flight software gates are about 2x the tower catch window (and no looser than 3x).
    CHECK(c.funnelBase >= 1.5 * c.windowLateral && c.funnelBase <= 3.0 * c.windowLateral);
    CHECK(c.speedGateBase >= 1.5 * c.windowHorizontalSpeed && c.speedGateBase <= 3.0 * c.windowHorizontalSpeed);
    CHECK(c.tiltGateBase >= 1.5 * c.windowTilt && c.tiltGateBase <= 3.0 * c.windowTilt);
    CHECK(c.headingGate >= 1.5 * c.windowHeading && c.headingGate <= 3.0 * c.windowHeading);
    CHECK(c.contactSinkRate < c.windowSink);
    // They widen with height above the seat.
    CHECK(c.funnelSlope > 0.0 && c.speedGateSlope > 0.0 && c.tiltGateSlope > 0.0);
    // The flight software is told no staging point: the translation target is the seat.
    CHECK(c.stagingDistance == 0.0);
}

TEST_CASE(catch_profile_gives_the_arms_time_to_close_before_contact) {
    const fsw::CatchConfig c;
    // Time from the arms-closing height to the seat on the nominal profile exceeds the closing time with margin,
    // and the profile is monotonic.
    CHECK(c.nominalTimeToSeat(c.armTriggerHeight) >= c.armsClosingTime + 1.0);
    CHECK(c.nominalTimeToSeat(0.0) == 0.0);
    CHECK(c.nominalTimeToSeat(10.0) < c.nominalTimeToSeat(20.0));
    // The whole sequence from 200 ft (60.96 m) above the seat fits well inside 15 s on the nominal profile.
    CHECK(c.nominalTimeToSeat(60.96) < 12.0);
}

TEST_CASE(catch_final_phase_misaligned_aborts_with_a_corridor_reason) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    const fsw::CatchConfig& c = rig.config.catchConfig;
    rig.altitudeTarget = c.seat[2] + 0.5 * c.finalHeight;
    rig.altitudeRate = 3.0;
    CHECK(rig.runUntil(fsw::HopMode::CatchFinal, 40.0));
    // Well outside the funnel (about 1.5 m wide at this height).
    rig.north = 6.0;
    rig.run(5.0);
    CHECK(rig.computer.catchAborted());
    CHECK(rig.computer.landingMode() == fsw::LandingMode::Pad);
    const std::vector<uint8_t>* status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        CHECK(userData(*status)[1] == static_cast<uint8_t>(fsw::CatchAbortReason::Corridor));
    }
}

TEST_CASE(catch_abort_high_above_the_seat_falls_back_to_the_pad) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    rig.fix = fsw::GnssFixType::Standalone;
    rig.run(rig.config.catchConfig.fixLossTime + 1.0);
    CHECK(rig.computer.catchAborted());
    CHECK(rig.computer.mode() == fsw::HopMode::Descent);
    // The pad fallback targets the pad center, not the seat.
    rig.run(0.2);
    CHECK(rig.computer.landingMode() == fsw::LandingMode::Pad);
}

TEST_CASE(catch_telemetry_packets_decode_in_catch_mode) {
    Rig rig(fsw::LandingMode::Catch);
    CHECK(rig.flyToApproach());
    rig.run(1.2);
    const fsw::CatchConfig& c = rig.config.catchConfig;

    const std::vector<uint8_t>* config = lastPacket(rig.hal, fsw::apid::kCatchConfig);
    CHECK(config != nullptr);
    if (config != nullptr) {
        CHECK(config->size() == fsw::ccsds::kHeaderSize + fsw::kCatchConfigLength);
        const uint8_t* u = userData(*config);
        CHECK(u[0] == static_cast<uint8_t>(fsw::LandingMode::Catch));
        CHECK_NEAR(readF32(u + 4), c.seat[0], 1e-4);
        CHECK_NEAR(readF32(u + 12), c.seat[2], 1e-4);
        CHECK_NEAR(readF32(u + 24), c.stagingDistance, 1e-4);
        CHECK_NEAR(readF32(u + 64), c.windowLateral, 1e-4);
    }

    const std::vector<uint8_t>* status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(status != nullptr);
    if (status != nullptr) {
        CHECK(status->size() == fsw::ccsds::kHeaderSize + fsw::kCatchStatusLength);
        const uint8_t* u = userData(*status);
        CHECK(u[0] == static_cast<uint8_t>(fsw::CatchPhase::Approach));
        CHECK(u[1] == static_cast<uint8_t>(fsw::CatchAbortReason::None));
        CHECK(u[2] == static_cast<uint8_t>(fsw::ArmsExpected::Open));
        CHECK((u[3] & 0x10) != 0);  // rtkOk
        CHECK((u[3] & 0x40) == 0);  // not aborted
        // Along remaining: over the seat. The rig teleports the vehicle, which tilts the levelled attitude estimate a
        // little, and the pins (30 m up) report that tilt as an offset, so the tolerance is loose.
        CHECK_NEAR(readF32(u + 4), 0.0, 4.0);
        const double funnel = c.funnelBase + c.funnelSlope * rig.heightAboveSeat();
        CHECK_NEAR(readF32(u + 32), funnel, 0.2);
    }
}

TEST_CASE(catch_packets_are_sent_in_pad_mode_as_inactive) {
    Rig rig(fsw::LandingMode::Pad);
    rig.run(1.2);
    const std::vector<uint8_t>* config = lastPacket(rig.hal, fsw::apid::kCatchConfig);
    const std::vector<uint8_t>* status = lastPacket(rig.hal, fsw::apid::kCatchStatus);
    CHECK(config != nullptr);
    CHECK(status != nullptr);
    if (config != nullptr) {
        CHECK(userData(*config)[0] == static_cast<uint8_t>(fsw::LandingMode::Pad));
    }
    if (status != nullptr) {
        const uint8_t* u = userData(*status);
        CHECK(u[0] == static_cast<uint8_t>(fsw::CatchPhase::Inactive));
        CHECK(u[1] == 0);
        CHECK(u[3] == 0);  // no flags
    }
}

TEST_CASE(pin_midpoint_offset_follows_tilt_and_lever) {
    const double imu[3] = {0.0, 0.0, 13.0};  // [m] body frame, from the base (test value)
    const double pinHeight = 30.0;           // [m]
    double offset[2];

    const double upright[4] = {1.0, 0.0, 0.0, 0.0};
    fsw::pinMidpointOffset(upright, imu, pinHeight, offset);
    CHECK(std::abs(offset[0]) < 1e-12 && std::abs(offset[1]) < 1e-12);

    // Tilt of 3 deg about world +x: body +z leans toward -north (right-hand rotation), so the pin midpoint sits
    // (pinHeight - imuZ) * sin(tilt) north-negative of the IMU point and does not move east.
    const double tilt = 3.0 * kPi / 180.0;  // [rad]
    const double aboutX[4] = {std::cos(tilt / 2.0), std::sin(tilt / 2.0), 0.0, 0.0};
    fsw::pinMidpointOffset(aboutX, imu, pinHeight, offset);
    CHECK_NEAR(offset[0], 0.0, 1e-9);
    CHECK_NEAR(offset[1], -17.0 * std::sin(tilt), 1e-9);

    // Tilt about world +y leans body +z toward +east.
    const double aboutY[4] = {std::cos(tilt / 2.0), 0.0, std::sin(tilt / 2.0), 0.0};
    fsw::pinMidpointOffset(aboutY, imu, pinHeight, offset);
    CHECK_NEAR(offset[0], 17.0 * std::sin(tilt), 1e-9);
    CHECK_NEAR(offset[1], 0.0, 1e-9);
}
