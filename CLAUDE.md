# Starship Simulator

Engineering-grade simulation of SpaceX Starship (V3), flown by real flight software (in this repo) that only sees simulated sensors, run as a LAN **mission control simulation**: `sim_server` streams to browser clients where people take console positions. Realism over fun: no scores, no shortcuts, no truth on the consoles. Portfolio project, so code quality, tests and documented assumptions matter. Owner: Taras Drozd (knows Python and Java best; this is C++20).

Current status: a Starship V3 **Ship in hop configuration** (three sea-level Raptor 3s, estimated mass properties) lifts off, climbs to 50 m, then descends on a fast braking profile (no hover) while diverting to Starbase Pad 2 using simulated GNSS (WGS-84) plus IMU, and lands on the pad center (placeholder for the catch point). Sensors: IMU, radar altimeter, GNSS, with wind. `sim_server` serves a three.js outside view over the truth stream (`/`) and the first console, GNC (`/gnc/`), fed only by CCSDS telemetry the flight computer downlinks through a placeholder comms model (`/ws/telemetry`). The ground segment is fully custom (no Yamcs, no Open MCT). No sessions, roles, commanding, archive or limit alarms yet.

## More context (read when relevant)

| File | What |
|---|---|
| `docs/build_order.md` | The 51-item build order with status. `TODO(phase N, item M)` refers to it. Update Status as work lands. |
| `docs/architecture.md` | Rate plan, planned runtime architecture (sessions, crew, streams), the custom ground segment, rendering plan |
| `docs/vehicle_reference.md` | Starship V3 public facts |
| `docs/protocol/*.md` | Versioned wire protocols (treat as contracts) |
| `telemetry/dictionary.json` | Single source of truth for downlinked CCSDS packets. Flight packet code, console decoders and `docs/telemetry/gnc_packets.md` are generated from it: edit the dictionary, never the generated files |
| `<dir>/CLAUDE.md` | Per-module rules: `flight_sw`, `sim_core`, `sim_vehicle`, `sim_server`, `web`, `tests`, `tools` |
| `README.md` | Human-facing overview and run commands |

## Build and test

Use the `build-and-test` skill on this machine (Windows, MSVC, VS generator, CMake by full path). Generic:

```bash
cmake -S . -B build && cmake --build build --config Release
ctest --test-dir build -C Release          # or build/tests/Release/sim_tests.exe [filter]
python -I web/tests/run_web_tests.py       # web unit tests (Playwright)
python -I tools/e2e/server_smoke.py        # end-to-end server checks
python -I tools/telemetry/test_generate.py # telemetry generator tests
python -I tools/telemetry/generate.py --check   # generated telemetry files up to date
```

All tests must pass before any commit. Use `--host 127.0.0.1` for `sim_server` in tests and agents (avoids the Windows firewall prompt). `run-server` skill launches it for a demo.

## Layering (never include upward)

`flight_sw` <- nothing (not even Eigen) · `sim_core` <- nothing · `sim_vehicle` <- `sim_core`, `flight_sw` · apps (`sim_cli`, `sim_server`) <- `sim_vehicle`. Only `sim_server` does networking. Clients (`web/`) talk to `sim_server` only through documented protocols and never link the sim.

## The step

Base rate 1 kHz, sim time is `step * dt`, never a clock. Each step: (1) `prepare(t)` on every `ContinuousSystem` (discrete events, derived truth sensors read); (2) `Scheduler::run` runs due `Model`s in insertion order (sensors, then flight software, then loggers); (3) `integrate(t, dt)` with RK4. Continuous state lives in a `ContinuousSystem` with one `derivatives(state, u)`; everything discrete is a `Model` that talks only through the `Bus`. Every rate must divide 1 kHz (60 Hz is rejected).

## Binding rules

- **No allocation in the step loop.** Allocate in constructors or `initialize()`; fixed-size Eigen types; resolve bus channels in `initialize()` and keep the reference.
- **Determinism.** Same seed, bit-identical flight. Use `sim::Rng`, never `std::*_distribution` (they differ across MSVC/GCC). Per-model streams via `deriveSeed(masterSeed, "model_name")`.
- **Flight software never reads truth**, only `fsw::Hal`. **Consoles never see truth**, only downlinked telemetry.
- **The sim thread never blocks on networking.**
- **Animate and log actual actuator positions**, never commanded ones.
- **Sensors** model noise, bias, drift, scale factor, quantization, rate, latency, saturation, failures and mounting location, with an `ideal` switch.
- **Data-oriented layout** for repeated things (39 engines, ~18,000 tiles): structure of arrays.
- **Pacing lives only in `Pacer`.**
- **Vehicle numbers** live in parameters files with a source comment (link, estimate, or derived). SpaceX internal data is not public; never imply otherwise.

## Telemetry dictionary

Packet layouts live only in `telemetry/dictionary.json`. After editing it run `python -I tools/telemetry/generate.py` (writes `flight_sw/include/fsw/TelemetryPackets.hpp`, `web/js/gnc/gncPackets.generated.js`, `docs/telemetry/gnc_packets.md`); never hand-edit those. Generated files are committed so the C++ build needs no Python. Update the golden in `tests/test_telemetry.cpp` only when the wire bytes are meant to change.

## Conventions

- C++20, warnings clean on MSVC `/W4` and GCC/Clang `-Wall -Wextra -Wpedantic -Wshadow`.
- Types `PascalCase`, functions/variables `camelCase`, members `trailing_`, constants `kName`.
- SI units, radians internally, units in comments: `// [m/s]`.
- Frames: dynamics in a flat local ENU world (z up, origin at the landing target, tied to WGS-84 by `sim/env/Geodesy.hpp`), uniform gravity; body +z toward the nose; `attitude` rotates body to world. Vehicle state is the CG; the truth stream and contact use the base. ECI/ECEF dynamics is build order item 2.
- Label placeholders in code; tag future work `TODO(phase N, item M)`.
- No em dashes in docs or comments (a hook checks).
- Never edit `web/vendor/`.
