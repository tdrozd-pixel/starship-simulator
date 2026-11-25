# Starship Simulator

An engineering-grade simulation of SpaceX Starship (V3): launch, hot staging, Booster boostback and tower catch, Ship entry and landing. Every system, gauge and sensor is simulated, and the vehicles are flown by real flight software that only sees simulated sensors.

It is built as a **mission control simulation**: one machine runs `sim_server`, and anyone on the local network opens a browser, takes a console position (FLIGHT, PROP, GNC, POWER, ...) and works the flight in real time. Crew size is configurable, from one person holding every position to a full room.

## Status

Early. The framework (fixed-step executive, rate-group scheduler, bus, RK4, deterministic seeding, real-time pacing, logging) flies a **Starship V3 Ship in hop configuration** on its three sea-level Raptor 3 engines: it climbs to 250 m, translates to a point over a tower's catch seat using simulated RTK GPS (WGS-84 coordinates) fused with the IMU, then comes straight down between the open arms on a braking profile (about 10 s from 200 ft above the seat) while the arms close, and is caught on two hypothetical pins by the tower's chopstick arms (`--landing pad` lands on the center of Starbase Pad 2 instead, and a failed catch check aborts to that pad landing). Mass properties are estimates where SpaceX has not published numbers. `sim_server` runs it in real time and serves a browser 3D outside view over the LAN and a GNC console fed only by downlinked CCSDS telemetry. More consoles, sessions and commanding come next. See [docs/build_order.md](docs/build_order.md).

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
build/sim_cli/sim_cli --fast --landing pad     # land on the pad center instead of the tower catch (default: catch)
build/sim_cli/sim_cli --fast --no-gust         # catch without the default gust (100 m/s eastward, one-cosine, t = 8 s, 1 s long)
build/sim_cli/sim_cli --fast --wind-speed 12 --wind-from 0   # steady 12 m/s ground wind from the north (default: no mean wind); the run prints the landing times T1 and T2
build/sim_cli/sim_cli --time-scale 0.25 --seed 42

build/sim_server/sim_server --host 127.0.0.1   # outside view http://127.0.0.1:8080/, GNC console /gnc/
build/sim_server/sim_server                    # LAN: binds 0.0.0.0:8080, prints a URL per address
build/sim_server/sim_server --landing pad      # pad landing instead of the tower catch
```

`sim_server` holds the vehicle on the pad until a browser connects (`--autostart` skips that). On first LAN run Windows asks about the firewall: allow Private networks only. `-DSIM_BUILD_SERVER=OFF` skips the server. `-DSIM_ENABLE_TRACY=ON` enables Tracy profiling (GUI v0.11.1).

## Tests

```bash
ctest --test-dir build -C Release              # C++ (or build/tests/sim_tests [filter])
python -I web/tests/run_web_tests.py           # web client (Playwright, headless Chromium/Edge)
python -I tools/e2e/server_smoke.py            # end-to-end against the real server
python -I tools/telemetry/test_generate.py     # telemetry dictionary generator
python -I tools/telemetry/generate.py --check  # generated telemetry files are current
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
| `telemetry/` | `dictionary.json`: the single source of truth for downlinked packets (flight code, console decoders and docs are generated from it by `tools/telemetry/`) |
| `tests/`, `tools/` | C++ unit tests, end-to-end tests and the telemetry generator |

## Notes

Vehicle data comes from public sources; anything else is labeled as an estimate. SpaceX internal data is not public and nothing here implies otherwise. This project is not affiliated with SpaceX.

Licensed under the GNU GPL v3; see [LICENSE](LICENSE). three.js (MIT) is vendored in `web/vendor/three/`.
