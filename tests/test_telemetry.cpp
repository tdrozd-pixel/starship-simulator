#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

#include "TestFramework.hpp"
#include "fsw/Ccsds.hpp"
#include "fsw/GncTelemetry.hpp"
#include "fsw/TelemetryPackets.hpp"
#include "sim/vehicle/HopScenario.hpp"
#include "sim/vehicle/Messages.hpp"

namespace {

namespace ccsds = fsw::ccsds;
using sim::vehicle::HopScenario;
using sim::vehicle::HopScenarioOptions;

struct Captured {
    std::vector<uint8_t> bytes;
    double groundTime = 0.0;
};

class CapturingSink final : public sim::vehicle::TelemetrySink {
public:
    void deliver(const uint8_t* packet, std::size_t length, double groundTime) noexcept override {
        Captured c;
        c.bytes.assign(packet, packet + length);
        c.groundTime = groundTime;
        packets.push_back(std::move(c));
    }
    std::vector<Captured> packets;
};

struct Run {
    std::vector<Captured> packets;
    uint64_t sent = 0;
    uint64_t lost = 0;
    uint64_t overflowed = 0;
    double touchdownTime = 0.0;
    double touchdownEast = 0.0;
    double touchdownNorth = 0.0;
    double maxAltitude = 0.0;
    double endTime = 0.0;
};

Run runHop(HopScenarioOptions options, bool withSink) {
    CapturingSink sink;
    if (withSink) {
        options.telemetrySink = &sink;
    }
    HopScenario scenario = sim::vehicle::buildHopScenario(options);
    while (!scenario.sim->halted() && scenario.sim->time() < 150.0) {
        scenario.sim->step();
    }
    Run run;
    run.packets = std::move(sink.packets);
    if (scenario.downlink != nullptr) {
        run.sent = scenario.downlink->sent();
        run.lost = scenario.downlink->lost();
        run.overflowed = scenario.downlink->overflowed();
    }
    const auto& td = scenario.vehicle->touchdown();
    run.touchdownTime = td.time;
    run.touchdownEast = td.east;
    run.touchdownNorth = td.north;
    run.maxAltitude = scenario.vehicle->maxAltitude();
    run.endTime = scenario.sim->time();
    return run;
}

std::size_t expectedLength(uint16_t apid) {
    switch (apid) {
        case fsw::apid::kFswStatus: return ccsds::kHeaderSize + fsw::kFswStatusLength;
        case fsw::apid::kMissionConfig: return ccsds::kHeaderSize + fsw::kMissionConfigLength;
        case fsw::apid::kNavigation: return ccsds::kHeaderSize + fsw::kNavigationLength;
        case fsw::apid::kGuidanceControl: return ccsds::kHeaderSize + fsw::kGuidanceControlLength;
        case fsw::apid::kActuatorFeedback: return ccsds::kHeaderSize + fsw::kActuatorFeedbackLength;
        case fsw::apid::kEngineStatus: return ccsds::kHeaderSize + fsw::kEngineStatusLength;
        case fsw::apid::kGnss: return ccsds::kHeaderSize + fsw::kGnssLength;
        case fsw::apid::kCatchStatus: return ccsds::kHeaderSize + fsw::kCatchStatusLength;
        case fsw::apid::kCatchConfig: return ccsds::kHeaderSize + fsw::kCatchConfigLength;
        default: return 0;
    }
}

}  // namespace

TEST_CASE(ccsds_primary_header_matches_hand_computed_bytes) {
    uint8_t buf[6] = {};
    ccsds::writePrimaryHeader(buf, 0x110, 5, 86);
    // 0x0800 | 0x110 = 0x0910; 0xC000 | 5 = 0xC005; 86 - 1 = 85 = 0x0055.
    const uint8_t expected[6] = {0x09, 0x10, 0xC0, 0x05, 0x00, 0x55};
    for (int i = 0; i < 6; ++i) {
        CHECK(buf[i] == expected[i]);
    }
    const ccsds::PrimaryHeader h = ccsds::readPrimaryHeader(buf);
    CHECK(h.version == 0);
    CHECK(h.type == 0);
    CHECK(h.secondaryHeader);
    CHECK(h.apid == 0x110);
    CHECK(h.sequenceFlags == 3);
    CHECK(h.sequenceCount == 5);
    CHECK(h.dataLength == 85);
}

TEST_CASE(ccsds_scalar_round_trips_are_big_endian) {
    uint8_t buf[8] = {};
    ccsds::writeU32(buf, 0x01020304U);
    CHECK(buf[0] == 1 && buf[3] == 4);
    CHECK(ccsds::readU32(buf) == 0x01020304U);
    ccsds::writeF32(buf, 1.5F);  // 0x3FC00000
    CHECK(buf[0] == 0x3F && buf[1] == 0xC0 && buf[2] == 0 && buf[3] == 0);
    CHECK(ccsds::readF32(buf) == 1.5F);
    ccsds::writeF64(buf, -2.25);
    CHECK(ccsds::readF64(buf) == -2.25);
    ccsds::writeU16(buf, 0xABCD);
    CHECK(buf[0] == 0xAB && buf[1] == 0xCD);
}

TEST_CASE(ccsds_cuc_time_round_trips_and_clamps_negative) {
    uint8_t buf[6] = {};
    ccsds::writeCucTime(buf, 12.5);
    CHECK(ccsds::readU32(buf) == 12U);
    CHECK(ccsds::readU16(buf + 4) == 32768U);
    CHECK_NEAR(ccsds::readCucTime(buf), 12.5, 1e-9);
    ccsds::writeCucTime(buf, 98.123456);
    CHECK_NEAR(ccsds::readCucTime(buf), 98.123456, 1.0 / 65536.0);
    ccsds::writeCucTime(buf, -3.0);
    CHECK(ccsds::readCucTime(buf) == 0.0);
}

namespace {

class CountingHal final : public fsw::Hal {
public:
    double timeSec() const override { return 0.0; }
    fsw::ImuSample readImu() override { return {}; }
    fsw::AltimeterSample readAltimeter() override { return {}; }
    fsw::GnssSample readGnss() override { return {}; }
    fsw::GimbalFeedbackSample readGimbalFeedback() override { return {}; }
    fsw::EngineFeedbackSample readEngineFeedback() override { return {}; }
    void commandActuators(const fsw::ActuatorCommand&) override {}
    void publishTelemetry(const fsw::Telemetry&) override {}
    void downlink(const uint8_t* packet, std::size_t length) override {
        packets.emplace_back(packet, packet + length);
    }
    std::vector<std::vector<uint8_t>> packets;
};

}  // namespace

TEST_CASE(gnc_telemetry_sequence_count_wraps_at_16383) {
    fsw::GncTelemetry tm;
    CountingHal hal;
    const fsw::GncTelemetryInput input;
    // The status packet goes out every 10th cycle.
    for (uint32_t i = 0; i < 16385; ++i) {
        tm.emit(hal, input, i * 10U, i * 0.1);
    }
    std::vector<uint16_t> status;
    for (const auto& p : hal.packets) {
        if (ccsds::readPrimaryHeader(p.data()).apid == fsw::apid::kFswStatus) {
            status.push_back(ccsds::readPrimaryHeader(p.data()).sequenceCount);
        }
    }
    CHECK(status.size() == 16385);
    CHECK(status[16383] == 16383);
    CHECK(status[16384] == 0);
}

namespace {

// Decodes and cross-checks a whole flight's downlink; returns the highest flight mode seen.
int checkTelemetryStream(const HopScenarioOptions& options) {
    const Run run = runHop(options, true);
    CHECK(run.packets.size() > 1000);
    CHECK(run.overflowed == 0);

    std::map<uint16_t, uint16_t> lastSeq;
    std::map<uint16_t, double> lastTime;
    std::map<uint16_t, uint64_t> gaps;
    std::map<uint16_t, int> counts;
    int maxMode = 0;
    double minLatency = 1e9;
    double maxLatency = -1e9;
    bool satsOk = true;
    bool packetsOk = true;
    for (const Captured& c : run.packets) {
        const ccsds::PrimaryHeader h = ccsds::readPrimaryHeader(c.bytes.data());
        const std::size_t expected = expectedLength(h.apid);
        if (expected == 0 || c.bytes.size() != expected ||
            static_cast<std::size_t>(h.dataLength) + 1 + ccsds::kPrimaryHeaderSize != c.bytes.size() ||
            h.version != 0 || h.type != 0 || !h.secondaryHeader || h.sequenceFlags != 3) {
            packetsOk = false;
            continue;
        }
        ++counts[h.apid];
        const double onboard = ccsds::readCucTime(c.bytes.data() + ccsds::kPrimaryHeaderSize);
        minLatency = std::min(minLatency, c.groundTime - onboard);
        maxLatency = std::max(maxLatency, c.groundTime - onboard);
        if (counts[h.apid] > 1) {
            const uint16_t diff = static_cast<uint16_t>((h.sequenceCount - lastSeq[h.apid]) & 0x3FFF);
            if (diff == 0) {
                packetsOk = false;
            } else {
                gaps[h.apid] += diff - 1U;
            }
            if (!(onboard > lastTime[h.apid])) {
                packetsOk = false;
            }
        }
        lastSeq[h.apid] = h.sequenceCount;
        lastTime[h.apid] = onboard;

        const uint8_t* user = c.bytes.data() + ccsds::kHeaderSize;
        if (h.apid == fsw::apid::kFswStatus) {
            maxMode = std::max<int>(maxMode, user[0]);
        } else if (h.apid == fsw::apid::kGnss) {
            const int used = user[1];
            const int visible = user[2];
            if (visible < 16 || visible > 32 || used > visible) {
                satsOk = false;
            }
        }
    }
    CHECK(packetsOk);
    CHECK(satsOk);
    CHECK(counts.size() == 9);
    CHECK(minLatency >= 0.015);
    CHECK(maxLatency <= 0.035);
    uint64_t totalGaps = 0;
    for (const auto& g : gaps) {
        totalGaps += g.second;
    }
    // Every gap is a lost packet (losses after the last delivered packet of an APID go unseen).
    CHECK(totalGaps <= run.lost);
    return maxMode;
}

}  // namespace

// The pad flight ends in Landed (mode 6).
TEST_CASE(telemetry_stream_decodes_and_is_consistent) {
    HopScenarioOptions options;
    options.landing = fsw::LandingMode::Pad;
    CHECK(checkTelemetryStream(options) == 6);
}

// The default (catch) flight carries the catch modes 7 to 10 on the wire; whether it ends
// caught is flight performance and is not asserted here.
TEST_CASE(telemetry_stream_decodes_in_catch_mode) {
    HopScenarioOptions options;
    options.weather.gusts.clear();
    const int maxMode = checkTelemetryStream(options);
    CHECK(maxMode >= 7);
    CHECK(maxMode <= 10);
}

namespace {

// 64-bit FNV-1a over a byte range, continuing from `hash`.
uint64_t fnv1a(uint64_t hash, const uint8_t* data, std::size_t length) {
    for (std::size_t i = 0; i < length; ++i) {
        hash ^= data[i];
        hash *= 0x100000001B3ULL;
    }
    return hash;
}

}  // namespace

// Pins the downlinked wire bytes of the first 0.9 s of the standard hop (seed 1, default
// options): the vehicle is on the pad, so the bytes depend on the packet layouts, the sensor
// models and navigation but not on guidance or control tuning (apart from the catch funnel limit below). The layouts are generated from
// telemetry/dictionary.json; this proves the generator and refactors never change a byte on
// the wire. The default landing mode is Catch: the stream includes the catch_status (it carries the
// approach funnel limit, so it depends on CatchConfig funnelBase/funnelSlope) and catch_config packets.
// Re-recorded (hash was 0x3E50B2D4D7711264, count unchanged at 179) when the GNSS fusion became delayed-measurement: the on-pad navigation values and innovations changed.
// The cut-off is ground receive time, so a change to the placeholder link latency also moves it. Update the constants only when the dictionary, those models or the link change on purpose.
TEST_CASE(telemetry_stream_matches_golden) {
    constexpr uint64_t kGoldenHash = 0x48A28B892886F245ULL;
    constexpr std::size_t kGoldenCount = 179;
    HopScenarioOptions options;
    const Run run = runHop(options, true);
    uint64_t hash = 0xCBF29CE484222325ULL;  // FNV offset basis
    std::size_t count = 0;
    for (const Captured& c : run.packets) {
        if (c.groundTime >= 0.9) {
            continue;
        }
        hash = fnv1a(hash, c.bytes.data(), c.bytes.size());
        ++count;
    }
    if (hash != kGoldenHash || count != kGoldenCount) {
        std::printf("golden: count=%zu hash=0x%016llXULL\n", count, static_cast<unsigned long long>(hash));
    }
    CHECK(count == kGoldenCount);
    CHECK(hash == kGoldenHash);
}

TEST_CASE(generated_user_data_lengths_match_the_documented_lengths) {
    namespace tm = fsw::telemetry;
    static_assert(tm::kFswStatusApid == 0x100 && tm::kMissionConfigApid == 0x101 && tm::kNavigationApid == 0x110);
    static_assert(tm::kGuidanceControlApid == 0x120 && tm::kActuatorFeedbackApid == 0x130 && tm::kGnssApid == 0x140 &&
                  tm::kEngineStatusApid == 0x150);
    const std::size_t generated[7] = {tm::kFswStatusUserDataLength,       tm::kMissionConfigUserDataLength,
                                      tm::kNavigationUserDataLength,      tm::kGuidanceControlUserDataLength,
                                      tm::kActuatorFeedbackUserDataLength, tm::kGnssUserDataLength, tm::kEngineStatusUserDataLength};
    const std::size_t documented[7] = {20, 36, 116, 84, 36, 56, 72};
    for (int i = 0; i < 7; ++i) {
        CHECK(generated[i] == documented[i]);
    }
}

TEST_CASE(telemetry_has_no_gaps_on_a_lossless_link) {
    HopScenarioOptions options;
    options.downlink.lossProbability = 0.0;
    const Run run = runHop(options, true);
    std::map<uint16_t, uint16_t> lastSeq;
    bool contiguous = true;
    for (const Captured& c : run.packets) {
        const ccsds::PrimaryHeader h = ccsds::readPrimaryHeader(c.bytes.data());
        auto it = lastSeq.find(h.apid);
        if (it != lastSeq.end() && ((h.sequenceCount - it->second) & 0x3FF) != 1) {
            contiguous = false;
        }
        lastSeq[h.apid] = h.sequenceCount;
    }
    CHECK(contiguous);
    CHECK(run.lost == 0);
}

TEST_CASE(telemetry_link_loss_rate_follows_the_spec) {
    HopScenarioOptions options;
    options.downlink.lossProbability = 0.1;
    const Run run = runHop(options, true);
    CHECK(run.sent > 1000);
    const double fraction = static_cast<double>(run.lost) / static_cast<double>(run.sent);
    CHECK(fraction > 0.07);
    CHECK(fraction < 0.13);
    CHECK(run.packets.size() + run.lost <= run.sent);
}

TEST_CASE(telemetry_is_deterministic_for_a_seed) {
    HopScenarioOptions options;
    options.seed = 7;
    const Run a = runHop(options, true);
    const Run b = runHop(options, true);
    CHECK(a.packets.size() == b.packets.size());
    bool identical = a.packets.size() == b.packets.size();
    for (std::size_t i = 0; identical && i < a.packets.size(); ++i) {
        identical = a.packets[i].bytes == b.packets[i].bytes && a.packets[i].groundTime == b.packets[i].groundTime;
    }
    CHECK(identical);
}

TEST_CASE(downlink_does_not_change_the_flight) {
    HopScenarioOptions options;
    options.seed = 3;
    options.landing = fsw::LandingMode::Pad;  // touchdown time and miss are the pad flight's outcome
    const Run with = runHop(options, true);
    const Run without = runHop(options, false);
    CHECK(with.touchdownTime == without.touchdownTime);
    CHECK(with.touchdownEast == without.touchdownEast);
    CHECK(with.touchdownNorth == without.touchdownNorth);
    CHECK(with.maxAltitude == without.maxAltitude);
    CHECK(with.touchdownTime > 0.0);
}

// Same for the catch flight: the downlink must not change when the sim halts or how high it flies.
TEST_CASE(downlink_does_not_change_the_catch_flight) {
    HopScenarioOptions options;
    options.seed = 3;
    options.weather.gusts.clear();
    const Run with = runHop(options, true);
    const Run without = runHop(options, false);
    CHECK(with.endTime == without.endTime);
    CHECK(with.maxAltitude == without.maxAltitude);
    CHECK(with.endTime > 0.0);
}

TEST_CASE(ideal_gimbal_feedback_equals_truth) {
    HopScenarioOptions options;
    options.idealSensors = true;
    HopScenario scenario = sim::vehicle::buildHopScenario(options);
    auto& channel = scenario.sim->bus().channel<sim::vehicle::GimbalFeedbackMsg>(sim::vehicle::channels::kGimbalFeedback);
    double worst = 0.0;
    double largest = 0.0;
    while (scenario.sim->time() < 20.0) {
        scenario.sim->step();
        const auto& msg = channel.latest();
        const auto& truth = scenario.vehicle->state().gimbal;
        for (int i = 0; i < sim::vehicle::kSeaLevelEngineCount; ++i) {
            worst = std::max({worst, std::abs(msg.data.pitch[i] - truth(0, i)), std::abs(msg.data.yaw[i] - truth(1, i))});
            largest = std::max(largest, std::abs(truth(0, i)) + std::abs(truth(1, i)));
        }
    }
    // The model samples at 100 Hz, so the value is up to 10 ms old (gimbal slew about 0.26 rad/s).
    CHECK(worst < 4e-3);
    CHECK(largest > 1e-3);  // the gimbals actually moved
}
