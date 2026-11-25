#include "TestFramework.hpp"

#if SIM_HAVE_SERVER

#include <atomic>
#include <bit>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>
#include <string>
#include <thread>

#include "sim/server/JsonMessages.hpp"
#include "sim/server/SimRunner.hpp"
#include "sim/server/SnapshotRing.hpp"
#include "sim/server/TelemetryFrame.hpp"
#include "sim/server/TelemetryRing.hpp"
#include "sim/server/WireFormat.hpp"

namespace {

using namespace sim::server;
using sim::vehicle::TruthSnapshot;

uint64_t readLe(const uint8_t* p, int bytes) {
    uint64_t v = 0;
    for (int i = 0; i < bytes; ++i) {
        v |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return v;
}

double readF64(const WireSnapshot& w, std::size_t off) { return std::bit_cast<double>(readLe(w.data() + off, 8)); }

TruthSnapshot numbered(uint64_t n) {
    TruthSnapshot s;
    s.step = n;
    return s;
}

}  // namespace

TEST_CASE(snapshot_wire_layout) {
    TruthSnapshot s;
    s.runId = 0x01020304u;
    s.flags = 0x5u;
    s.step = 0x1122334455667788ull;
    s.simTime = 1.5;
    s.position[0] = 2.0;
    s.position[1] = 3.0;
    s.position[2] = 4.0;
    s.velocity[0] = 5.0;
    s.velocity[1] = 6.0;
    s.velocity[2] = 7.0;
    s.attitude[0] = 8.0;
    s.attitude[1] = 9.0;
    s.attitude[2] = 10.0;
    s.attitude[3] = 11.0;
    s.angularRate[0] = 12.0;
    s.angularRate[1] = 13.0;
    s.angularRate[2] = 14.0;
    s.throttle = 15.0;
    s.gimbalPitch = 16.0;
    s.gimbalYaw = 17.0;
    s.propellantMass = 18.0;
    s.totalMass = 19.0;
    for (int i = 0; i < 6; ++i) {
        s.engineThrottle[i] = 20.0 + i;
    }
    for (int i = 0; i < 3; ++i) {
        s.engineGimbalPitch[i] = 30.0 + i;
        s.engineGimbalYaw[i] = 40.0 + i;
    }
    s.engineRunning[0] = true;
    s.engineRunning[2] = true;
    s.engineRunning[5] = true;
    s.armAngle[0] = 0.5;
    s.armAngle[1] = -0.25;
    s.towerState = 3;
    s.catchResult = 1;

    WireSnapshot w{};
    encodeSnapshot(s, w);
    static_assert(kSnapshotWireSize == 296);
    static_assert(sizeof(WireSnapshot) == 296);
    CHECK(w[0] == 'S' && w[1] == 'S' && w[2] == 'T' && w[3] == 'R');
    CHECK(readLe(w.data() + 4, 2) == 3);
    CHECK(readLe(w.data() + 6, 2) == 1);
    CHECK(readLe(w.data() + 8, 4) == 0x01020304u);
    CHECK(w[8] == 0x04);  // little-endian: low byte first
    // flags 5 plus engine bits 3 (engine 0), 5 (engine 2) and 8 (engine 5).
    CHECK(readLe(w.data() + 12, 4) == (5u | (1u << 3) | (1u << 5) | (1u << 8)));
    CHECK(readLe(w.data() + 16, 8) == 0x1122334455667788ull);

    // Literal offsets from the protocol document, not the named constants.
    const std::size_t offsets[] = {24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128, 136, 144, 152, 160, 168};
    for (std::size_t i = 0; i < 19; ++i) {
        CHECK_NEAR(readF64(w, offsets[i]), i == 0 ? 1.5 : static_cast<double>(i + 1), 1e-12);
    }

    for (std::size_t i = 0; i < 6; ++i) {
        CHECK_NEAR(readF64(w, 176 + 8 * i), 20.0 + static_cast<double>(i), 1e-12);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK_NEAR(readF64(w, 224 + 8 * i), 30.0 + static_cast<double>(i), 1e-12);
        CHECK_NEAR(readF64(w, 248 + 8 * i), 40.0 + static_cast<double>(i), 1e-12);
    }

    // Version 3 fields at literal offsets (little-endian f64 and u32).
    CHECK_NEAR(readF64(w, 272), 0.5, 1e-12);
    CHECK_NEAR(readF64(w, 280), -0.25, 1e-12);
    CHECK(readLe(w.data() + 288, 4) == 3);
    CHECK(readLe(w.data() + 292, 4) == 1);
    CHECK(w[288] == 3 && w[289] == 0 && w[292] == 1);
    // 0.5 = 0x3FE0000000000000, little-endian bytes 00 00 00 00 00 00 E0 3F.
    CHECK(w[272] == 0 && w[277] == 0 && w[278] == 0xE0 && w[279] == 0x3F);

    TruthSnapshot d;
    CHECK(decodeSnapshot(w.data(), w.size(), d));
    CHECK(d.flags == 5u);
    CHECK(d.armAngle[0] == 0.5 && d.armAngle[1] == -0.25 && d.towerState == 3 && d.catchResult == 1);
    for (int i = 0; i < 6; ++i) {
        CHECK(d.engineRunning[i] == s.engineRunning[i]);
        CHECK(d.engineThrottle[i] == s.engineThrottle[i]);
    }
    for (int i = 0; i < 3; ++i) {
        CHECK(d.engineGimbalPitch[i] == s.engineGimbalPitch[i] && d.engineGimbalYaw[i] == s.engineGimbalYaw[i]);
    }
    CHECK(d.runId == s.runId && d.flags == s.flags && d.step == s.step);
    CHECK(d.simTime == 1.5 && d.position[2] == 4.0 && d.velocity[0] == 5.0 && d.attitude[3] == 11.0);
    CHECK(d.angularRate[2] == 14.0 && d.throttle == 15.0 && d.gimbalPitch == 16.0 && d.gimbalYaw == 17.0);
    CHECK(d.propellantMass == 18.0 && d.totalMass == 19.0);

    WireSnapshot bad = w;
    bad[0] = 'X';
    CHECK(!decodeSnapshot(bad.data(), bad.size(), d));
    CHECK(!decodeSnapshot(w.data(), w.size() - 1, d));
    CHECK(!decodeSnapshot(w.data(), w.size() + 1, d));
    WireSnapshot v1 = w;
    v1[4] = 1;  // a version 1 frame is rejected
    CHECK(!decodeSnapshot(v1.data(), v1.size(), d));
    WireSnapshot v2 = w;
    v2[4] = 2;  // a version 2 frame (even at the v3 length) is rejected
    CHECK(!decodeSnapshot(v2.data(), v2.size(), d));
    CHECK(!decodeSnapshot(w.data(), 272, d));  // a v2 length is rejected
}

TEST_CASE(wire_v3_caught_crashed_flags) {
    TruthSnapshot s;
    s.flags = sim::vehicle::kSnapCaught | sim::vehicle::kSnapHalted;
    s.engineRunning[1] = true;
    WireSnapshot w{};
    encodeSnapshot(s, w);
    CHECK(readLe(w.data() + 12, 4) == ((1u << 9) | (1u << 2) | (1u << 4)));
    TruthSnapshot d;
    CHECK(decodeSnapshot(w.data(), w.size(), d));
    CHECK(d.flags == (sim::vehicle::kSnapCaught | sim::vehicle::kSnapHalted));
    s.flags = sim::vehicle::kSnapCrashed;
    encodeSnapshot(s, w);
    CHECK(readLe(w.data() + 12, 4) == ((1u << 10) | (1u << 4)));
    CHECK(sim::vehicle::kSnapCaught == (1u << 9) && sim::vehicle::kSnapCrashed == (1u << 10));
}

TEST_CASE(ring_orders_and_is_bounded) {
    SnapshotRing ring;
    TruthSnapshot out;
    CHECK(!ring.tryPop(out));
    for (uint64_t i = 0; i < SnapshotRing::kCapacity; ++i) {
        CHECK(ring.tryPush(numbered(i)));
    }
    CHECK(!ring.tryPush(numbered(999999)));  // full: refused, nothing overwritten
    for (uint64_t i = 0; i < SnapshotRing::kCapacity; ++i) {
        CHECK(ring.tryPop(out));
        CHECK(out.step == i);
    }
    CHECK(!ring.tryPop(out));
    // Wraparound: push and pop past the capacity several times.
    uint64_t next = 0;
    for (uint64_t round = 0; round < 5000; ++round) {
        CHECK(ring.tryPush(numbered(round)));
        CHECK(ring.tryPop(out));
        CHECK(out.step == round);
        ++next;
    }
    CHECK(next == 5000);
}

namespace {

// Every field derives from n, so a torn copy shows up as an inconsistency.
TruthSnapshot filled(uint64_t n) {
    TruthSnapshot s;
    const double d = static_cast<double>(n);
    s.runId = static_cast<uint32_t>(n);
    s.flags = static_cast<uint32_t>(n * 3u);
    s.step = n;
    s.simTime = d * 0.5;
    for (int i = 0; i < 3; ++i) {
        s.position[i] = d + i;
        s.velocity[i] = d * 2 + i;
        s.angularRate[i] = d * 3 + i;
    }
    for (int i = 0; i < 4; ++i) {
        s.attitude[i] = d * 4 + i;
    }
    s.throttle = d * 5;
    s.gimbalPitch = d * 6;
    s.gimbalYaw = d * 7;
    s.propellantMass = d * 8;
    s.totalMass = d * 9;
    for (int i = 0; i < 6; ++i) {
        s.engineThrottle[i] = d * 10 + i;
        s.engineRunning[i] = ((n + static_cast<uint64_t>(i)) & 1u) != 0;
    }
    for (int i = 0; i < 3; ++i) {
        s.engineGimbalPitch[i] = d * 11 + i;
        s.engineGimbalYaw[i] = d * 12 + i;
    }
    return s;
}

bool consistent(const TruthSnapshot& s, uint64_t n) {
    const TruthSnapshot e = filled(n);
    bool ok = s.runId == e.runId && s.flags == e.flags && s.step == e.step && s.simTime == e.simTime;
    for (int i = 0; i < 3; ++i) {
        ok = ok && s.position[i] == e.position[i] && s.velocity[i] == e.velocity[i] &&
             s.angularRate[i] == e.angularRate[i];
    }
    for (int i = 0; i < 4; ++i) {
        ok = ok && s.attitude[i] == e.attitude[i];
    }
    for (int i = 0; i < 6; ++i) {
        ok = ok && s.engineThrottle[i] == e.engineThrottle[i] && s.engineRunning[i] == e.engineRunning[i];
    }
    for (int i = 0; i < 3; ++i) {
        ok = ok && s.engineGimbalPitch[i] == e.engineGimbalPitch[i] && s.engineGimbalYaw[i] == e.engineGimbalYaw[i];
    }
    return ok && s.throttle == e.throttle && s.gimbalPitch == e.gimbalPitch && s.gimbalYaw == e.gimbalYaw &&
           s.propellantMass == e.propellantMass && s.totalMass == e.totalMass;
}

}  // namespace

TEST_CASE(ring_spsc_threads) {
    constexpr uint64_t kCount = 1500000;
    SnapshotRing ring;
    std::atomic<bool> abort{false};
    std::thread producer([&] {
        for (uint64_t i = 0; i < kCount && !abort.load(std::memory_order_relaxed);) {
            if (ring.tryPush(filled(i))) {
                ++i;
            }
        }
    });
    uint64_t expected = 0;
    bool ok = true;
    TruthSnapshot out;
    while (expected < kCount) {
        if (ring.tryPop(out)) {
            if (!consistent(out, expected)) {
                ok = false;
                abort = true;
                break;
            }
            ++expected;
        }
    }
    producer.join();
    CHECK(ok);
    CHECK(expected == kCount);
}

TEST_CASE(json_messages) {
    HelloInfo info;
    info.runId = 3;
    const std::string hello = makeHello(info);
    CHECK(hello.find("\"type\":\"hello\"") != std::string::npos);
    CHECK(hello.find("\"protocol\":3") != std::string::npos);
    CHECK(hello.find("\"tower\":{") != std::string::npos);
    CHECK(hello.find("\"position\":[60,0]") != std::string::npos);
    CHECK(hello.find("\"approachDir\":[-1,0]") != std::string::npos);
    for (const char* key : {"mastHalfWidth", "mastHeight", "hingeHalfSpacing", "hingeSetback", "armLength", "armHalfWidth",
                            "armThickness", "railHeight", "armOpenAngle", "armClosedAngle", "pinHeight", "pinOffset"}) {
        CHECK(hello.find(std::string("\"") + key + "\":") != std::string::npos);
    }
    CHECK(hello.find("\"mastHeight\":145") != std::string::npos);
    CHECK(hello.find("\"pinOffset\":5") != std::string::npos);
    CHECK(hello.find("\"snapshotRateHz\":100") != std::string::npos);
    CHECK(hello.find("\"runId\":3") != std::string::npos);
    CHECK(hello.find("\"scenario\":\"hop\"") != std::string::npos);
    CHECK(makeRunStarted(4) == "{\"type\":\"run_started\",\"runId\":4}");

    CHECK(parseClientMessage("{\"type\":\"restart\"}") == ClientMsg::Restart);
    CHECK(parseClientMessage("  { \"type\" :\n \"restart\" , \"x\": 1 } ") == ClientMsg::Restart);
    CHECK(parseClientMessage("{\"type\":\"explode\"}") == ClientMsg::Unknown);
    CHECK(parseClientMessage("garbage") == ClientMsg::Unknown);
    CHECK(parseClientMessage("") == ClientMsg::Unknown);
    CHECK(parseClientMessage("{\"type\":") == ClientMsg::Unknown);
    CHECK(parseClientMessage("{\"type\":restart}") == ClientMsg::Unknown);

    // "type" inside string values or nested containers must not match.
    { const std::string m = R"({"note":"\"type\":\"restart\""})"; CHECK(parseClientMessage(m) == ClientMsg::Unknown); }
    { const std::string m = R"({"note":"\"type\":\"restart\"","type":"restart"})"; CHECK(parseClientMessage(m) == ClientMsg::Restart); }
    { const std::string m = R"({"a":{"type":"restart"}})"; CHECK(parseClientMessage(m) == ClientMsg::Unknown); }
    { const std::string m = R"({"a":[1,{"type":"restart"}],"type":"restart"})"; CHECK(parseClientMessage(m) == ClientMsg::Restart); }
    { const std::string m = R"([{"type":"restart"}])"; CHECK(parseClientMessage(m) == ClientMsg::Unknown); }
    { const std::string m = R"({"type":1})"; CHECK(parseClientMessage(m) == ClientMsg::Unknown); }
    { const std::string m = R"({"type":"restart)"; CHECK(parseClientMessage(m) == ClientMsg::Unknown); }
    CHECK(parseClientMessage("   ") == ClientMsg::Unknown);
    const std::string huge(2000000, '{');
    CHECK(parseClientMessage(huge) == ClientMsg::Unknown);
    const std::string hugeString = "{\"a\":\"" + std::string(2000000, 'x') + "\",\"type\":\"restart\"}";
    CHECK(parseClientMessage(hugeString) == ClientMsg::Restart);
}

TEST_CASE(hello_survives_long_names_and_bad_numbers) {
    HelloInfo info;
    info.vehicleName = std::string(2000, 'a') + "\"quoted\"";
    info.timeScale = std::numeric_limits<double>::infinity();
    const std::string hello = makeHello(info);
    CHECK(hello.size() > 2000);
    CHECK(hello.find("\\\"quoted\\\"") != std::string::npos);
    CHECK(hello.front() == '{' && hello.back() == '}');
    CHECK(hello.find("inf") == std::string::npos);
}

namespace {

using Clock = std::chrono::steady_clock;

SimRunnerConfig fastRunnerConfig() {
    SimRunnerConfig c;
    c.timeScale = 50.0;
    c.maxSimTime = 120.0;
    c.autostart = true;
    return c;
}

// Pops until `pred` is true or the deadline passes, recording what it saw.
template <typename Pred>
bool drainUntil(SnapshotRing& ring, std::vector<TruthSnapshot>& seen, Pred pred, double timeoutSeconds = 30.0) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeoutSeconds);
    TruthSnapshot s;
    while (Clock::now() < deadline) {
        if (ring.tryPop(s)) {
            seen.push_back(s);
            if (pred(s)) {
                return true;
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return false;
}

bool isHalted(const TruthSnapshot& s) { return (s.flags & sim::vehicle::kSnapHalted) != 0; }

}  // namespace

TEST_CASE(runner_restart_stamps_new_run) {
    SnapshotRing ring;
    TelemetryRing tring;
    SimRunner runner(fastRunnerConfig(), ring, tring);
    runner.start();
    std::vector<TruthSnapshot> seen;
    CHECK(drainUntil(ring, seen, [](const TruthSnapshot& s) { return s.runId == 1 && s.step >= 100; }));
    runner.requestRestart();
    CHECK(drainUntil(ring, seen, [](const TruthSnapshot& s) { return s.runId == 2 && s.step >= 100; }));
    runner.stop();

    bool sawNew = false;
    bool oldAfterNew = false;
    uint64_t firstNewStep = ~0ull;
    for (const TruthSnapshot& s : seen) {
        CHECK(s.runId == 1 || s.runId == 2);
        if (s.runId == 2) {
            if (!sawNew) {
                firstNewStep = s.step;
            }
            sawNew = true;
        } else if (sawNew) {
            oldAfterNew = true;
        }
    }
    CHECK(sawNew);
    CHECK(!oldAfterNew);
    CHECK(firstNewStep == 0);
    CHECK(runner.runId() == 2);
}

TEST_CASE(runner_stop_while_halted_is_prompt) {
    SnapshotRing ring;
    SimRunnerConfig config = fastRunnerConfig();
    config.timeScale = 100.0;
    TelemetryRing tring;
    SimRunner runner(config, ring, tring);
    runner.start();
    std::vector<TruthSnapshot> seen;
    CHECK(drainUntil(ring, seen, isHalted));
    const auto t0 = Clock::now();
    runner.stop();
    CHECK(std::chrono::duration<double>(Clock::now() - t0).count() < 1.0);
    CHECK(!runner.failed());
}

TEST_CASE(runner_waits_for_client_before_starting) {
    SnapshotRing ring;
    std::atomic<bool> present{false};
    std::atomic<int> waitingCalls{0};
    SimRunnerConfig config = fastRunnerConfig();
    config.autostart = false;
    config.clientPresent = [&present] { return present.load(); };
    config.onWaitingForClient = [&waitingCalls] { ++waitingCalls; };
    TelemetryRing tring;
    SimRunner runner(config, ring, tring);
    runner.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    TruthSnapshot s;
    CHECK(!ring.tryPop(s));
    CHECK(runner.runId() == 0);
    present = true;
    std::vector<TruthSnapshot> seen;
    CHECK(drainUntil(ring, seen, [](const TruthSnapshot&) { return true; }, 10.0));
    CHECK(waitingCalls.load() == 1);
    CHECK(!seen.empty() && seen.front().step == 0);
    runner.stop();
}

TEST_CASE(runner_final_snapshot_survives_full_ring) {
    SnapshotRing ring;
    SimRunnerConfig config = fastRunnerConfig();
    config.timeScale = 100.0;
    TelemetryRing tring;
    SimRunner runner(config, ring, tring);
    runner.start();
    // Do not consume: the ring (2048) fills before the ~4000-snapshot run ends,
    // and the final halted snapshot has to wait for room instead of being lost.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    std::vector<TruthSnapshot> seen;
    CHECK(drainUntil(ring, seen, isHalted));
    CHECK(runner.droppedSnapshots() > 0);
    runner.stop();
}

TEST_CASE(ground_frame_round_trip) {
    TelemetryItem item;
    item.runId = 0x01020304u;
    item.groundTime = 12.5;
    item.length = 10;
    for (uint8_t i = 0; i < 10; ++i) {
        item.bytes[i] = static_cast<uint8_t>(0xA0 + i);
    }
    item.bytes[0] = 0x09;  // version 0, type 0, sec hdr 1, APID high bits 001
    item.bytes[1] = 0x20;  // APID 0x120 = 288
    CHECK(packetApid(item.bytes) == 288);

    uint8_t frame[kMaxGroundFrame] = {};
    const std::size_t n = encodeGroundFrame(item, frame);
    CHECK(n == kGroundHeaderSize + 10);
    CHECK(frame[0] == 'S' && frame[1] == 'T' && frame[2] == 'L' && frame[3] == 'M');
    CHECK(readLe(frame + 4, 2) == 1);
    CHECK(readLe(frame + 6, 2) == 1);
    CHECK(readLe(frame + 8, 4) == 0x01020304u);
    CHECK(frame[8] == 0x04 && frame[11] == 0x01);
    CHECK(readLe(frame + 12, 2) == 10);
    CHECK(readLe(frame + 14, 2) == 0);
    CHECK(std::bit_cast<double>(readLe(frame + 16, 8)) == 12.5);
    CHECK(frame[24] == 0x09 && frame[33] == 0xA9);

    TelemetryItem back;
    CHECK(decodeGroundFrame(frame, n, back));
    CHECK(back.runId == item.runId && back.length == 10 && back.groundTime == 12.5);
    CHECK(back.bytes[5] == item.bytes[5]);

    CHECK(!decodeGroundFrame(frame, n - 1, back));
    CHECK(!decodeGroundFrame(frame, 10, back));
    frame[0] = 'X';
    CHECK(!decodeGroundFrame(frame, n, back));
    frame[0] = 'S';
    frame[4] = 2;
    CHECK(!decodeGroundFrame(frame, n, back));

    // Largest packet fits.
    item.length = static_cast<uint16_t>(sim::vehicle::kMaxTelemetryPacket);
    CHECK(encodeGroundFrame(item, frame) == kMaxGroundFrame);
}

TEST_CASE(telemetry_hello_content) {
    const std::string hello = makeTelemetryHello(7, 0.5);
    CHECK(hello.find("\"type\":\"hello\"") != std::string::npos);
    CHECK(hello.find("\"stream\":\"telemetry\"") != std::string::npos);
    CHECK(hello.find("\"protocol\":1") != std::string::npos);
    CHECK(hello.find("\"runId\":7") != std::string::npos);
    CHECK(hello.find("\"timeScale\":0.5") != std::string::npos);
    CHECK(hello.find("\"fsw_status\":256") != std::string::npos);
    CHECK(hello.find("\"mission_config\":257") != std::string::npos);
    CHECK(hello.find("\"navigation\":272") != std::string::npos);
    CHECK(hello.find("\"guidance_control\":288") != std::string::npos);
    CHECK(hello.find("\"actuator_feedback\":304") != std::string::npos);
    CHECK(hello.find("\"gnss\":320") != std::string::npos);
    CHECK(hello.find("\"engine_status\":336") != std::string::npos);
    CHECK(hello.find("\"catch_status\":352") != std::string::npos);
    CHECK(hello.find("\"catch_config\":353") != std::string::npos);
}

TEST_CASE(telemetry_ring_orders_and_is_bounded) {
    TelemetryRing ring;
    TelemetryItem item;
    for (std::size_t i = 0; i < TelemetryRing::kCapacity; ++i) {
        item.runId = static_cast<uint32_t>(i);
        CHECK(ring.tryPush(item));
    }
    item.runId = 999999;
    CHECK(!ring.tryPush(item));  // full: never overwrites, never blocks
    TelemetryItem out;
    for (std::size_t i = 0; i < TelemetryRing::kCapacity; ++i) {
        CHECK(ring.tryPop(out));
        CHECK(out.runId == i);
    }
    CHECK(!ring.tryPop(out));
    CHECK(ring.tryPush(item));
}

TEST_CASE(runner_delivers_telemetry_with_run_id) {
    SnapshotRing ring;
    TelemetryRing tring;
    SimRunner runner(fastRunnerConfig(), ring, tring);
    runner.start();

    std::vector<TelemetryItem> seen;
    std::vector<TruthSnapshot> snaps;
    auto drainTelemetry = [&](uint32_t run, std::size_t count) {
        const auto deadline = Clock::now() + std::chrono::seconds(30);
        TelemetryItem t;
        TruthSnapshot s;
        while (Clock::now() < deadline) {
            while (ring.tryPop(s)) {  // keep the truth ring from filling
            }
            if (tring.tryPop(t)) {
                seen.push_back(t);
                std::size_t n = 0;
                for (const TelemetryItem& x : seen) {
                    n += x.runId == run ? 1 : 0;
                }
                if (n >= count) {
                    return true;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return false;
    };
    CHECK(drainTelemetry(1, 200));
    runner.requestRestart();
    CHECK(drainTelemetry(2, 50));
    runner.stop();

    std::bitset<512> apids;
    bool sawNew = false;
    bool oldAfterNew = false;
    for (const TelemetryItem& t : seen) {
        CHECK(t.runId == 1 || t.runId == 2);
        CHECK(t.length >= kMinCcsdsPacket && t.length <= sim::vehicle::kMaxTelemetryPacket);
        CHECK(t.groundTime >= 0.0);
        const uint16_t apid = packetApid(t.bytes);
        if (t.runId == 1 && apid >= 256 && apid < 512) {
            apids.set(apid);
        }
        if (t.runId == 2) {
            sawNew = true;
        } else if (sawNew) {
            oldAfterNew = true;
        }
    }
    CHECK(sawNew);
    CHECK(!oldAfterNew);
    for (const std::size_t apid : {256u, 257u, 272u, 288u, 304u, 320u}) {
        CHECK(apids.test(apid));
    }
    CHECK(runner.droppedTelemetry() == 0);
}


#endif  // SIM_HAVE_SERVER
