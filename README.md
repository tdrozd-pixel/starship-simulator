# Starship Simulator

An engineering-grade simulation of SpaceX Starship (V3): launch, hot staging, Booster boostback and tower catch, Ship entry and landing. Every system, gauge and sensor is simulated, and the vehicles are flown by real flight software that only sees simulated sensors.

It is built as a **mission control simulation**: one machine runs `sim_server`, and anyone on the local network opens a browser, takes a console position (FLIGHT, PROP, GNC, POWER, ...) and works the flight in real time. Crew size is configurable, from one person holding every position to a full room.

## Status

Early. The framework (fixed-step executive, rate-group scheduler, bus, RK4, deterministic seeding, real-time pacing, logging) flies a **Starship V3 Ship in hop configuration** on its three sea-level Raptor 3 engines: it climbs to 50 m, diverts about 36 m using simulated GPS (WGS-84 coordinates) fused with the IMU, and lands on the center of Starbase Pad 2, the stand-in for a tower catch. Mass properties are estimates where SpaceX has not published numbers. `sim_server` runs it in real time and serves a browser 3D outside view over the LAN. Consoles, sessions and telemetry downlink come next. See [docs/build_order.md](docs/build_order.md).

## Build

Requires CMake 3.21+ and a C++20 compiler (MSVC 2022, GCC 12+ or Clang 15+). Eigen 3.4 and civetweb are fetched automatically if not installed.

```bash
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release
```

With the Visual Studio generator, binaries land in `build/<target>/Release/`.

## Run

```bash
build/sim_cli/sim_cli                          # headless real-time hop
build/sim_cli/sim_cli --fast --log hop.csv     # as fast as possible, CSV log
build/sim_cli/sim_cli --ideal-sensors          # truth-mode sensors for tuning
build/sim_cli/sim_cli --time-scale 0.25 --seed 42

build/sim_server/sim_server --host 127.0.0.1   # then open http://127.0.0.1:8080/
build/sim_server/sim_server                    # LAN: binds 0.0.0.0:8080, prints a URL per address
```

`sim_server` holds the vehicle on the pad until a browser connects (`--autostart` skips that). On first LAN run Windows asks about the firewall: allow Private networks only. `-DSIM_BUILD_SERVER=OFF` skips the server. `-DSIM_ENABLE_TRACY=ON` enables Tracy profiling (GUI v0.11.1).

## Tests

```bash
ctest --test-dir build -C Release              # C++ (or build/tests/sim_tests [filter])
python -I web/tests/run_web_tests.py           # web client (Playwright, headless Chromium/Edge)
python -I tools/e2e/server_smoke.py            # end-to-end against the real server
```

## Layout

| Directory | Contents |
|---|---|
| `flight_sw/` | Flight software. Depends on nothing else in the repo, so it can target a real flight computer |
| `sim_core/` | Vehicle-agnostic framework (executive, scheduler, bus, integrator, pacer, RNG) and environment (atmosphere, gravity, weather) |
| `sim_vehicle/` | Vehicle dynamics, sensors, flight software bridge, scenarios |
| `sim_cli/` | Headless runner |
| `sim_server/` | Real-time LAN server (the only networking code) |
| `web/` | Browser clients, plain ES modules, no build step |
| `docs/` | [Architecture](docs/architecture.md), [build order](docs/build_order.md), [vehicle reference](docs/vehicle_reference.md), [protocols](docs/protocol/) |
| `tests/`, `tools/e2e/` | C++ unit tests and end-to-end tests |

## Notes

Vehicle data comes from public sources; anything else is labeled as an estimate. SpaceX internal data is not public and nothing here implies otherwise. This project is not affiliated with SpaceX.

Licensed under the GNU GPL v3; see [LICENSE](LICENSE). three.js (MIT) is vendored in `web/vendor/three/`.
