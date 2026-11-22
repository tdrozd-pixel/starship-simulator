# Starship Simulator

Engineering-grade simulation of SpaceX Starship (V3): launch, hot staging, Booster boostback and tower catch, Ship entry and landing. Every system, gauge and sensor is simulated, and the vehicles are flown by real flight software (written in this repo) that only sees simulated sensors. The goal is extreme realism; schedule is not a constraint. It is also a portfolio project, so code quality, tests and documented assumptions matter.

Owner: Taras Drozd. Primary languages he knows best are Python and Java; this project is C++20.

## Current status

The framework skeleton is in place and flies a single-engine **test hopper** (placeholder vehicle, not Starship) through a 50 m vertical hop with modeled IMU and radar altimeter noise. That proves the sim executive, rate-group scheduler, bus, RK4 integrator, pacing, logging and the flight software seam. Next work follows the build order below.

## Build and run

Requires CMake 3.21+ and a C++20 compiler (MSVC 2022, GCC 12+, Clang 15+). Eigen 3.4 is found locally (vcpkg, apt, brew) or downloaded automatically.

```bash
cmake -S . -B build                    # add -G Ninja if available
cmake --build build --config Release
ctest --test-dir build -C Release      # or run build/tests/sim_tests directly
build/sim_cli/sim_cli                  # real-time hop
build/sim_cli/sim_cli --fast --log hop.csv
build/sim_cli/sim_cli --ideal-sensors  # "truth mode" for control tuning
build/sim_cli/sim_cli --time-scale 0.25 --seed 42
```

On Windows with the Visual Studio generator, binaries land in `build/sim_cli/Release/` and `build/tests/Release/`.

Profiling: configure with `-DSIM_ENABLE_TRACY=ON`, run the Tracy GUI (v0.11.1 to match the client), and connect while the sim runs. Every scheduled model shows up as its own zone.

Tests: `sim_tests [filter]` runs all tests, or only those whose name contains `filter`. All tests must pass before any commit.

## Repository layout

```
flight_sw/     Flight software. Depends on NOTHING else in the repo (no Eigen either):
               it must be buildable for a real flight computer against a real HAL.
  Hal.hpp                 the only interface flight software uses to touch the world
  Pid, VerticalEstimator  control and navigation building blocks
  HopFlightComputer       vertical hop mission: modes, cascade control, guidance
sim_core/      Simulation framework and environment, vehicle-agnostic
  core/  Simulation (executive), Scheduler (rate groups), Bus (typed channels),
         Model, ContinuousSystem, Integrator (RK4), Pacer, Random, CsvWriter, Profiling
  env/   Atmosphere (US76), Gravity (uniform placeholder)
sim_vehicle/   Vehicle models, sensors and the flight software bridge
  HopperDynamics          6DOF rigid body, engine, TVC and throttle actuators, ground contact
  Sensors                 ImuModel, RadarAltimeterModel
  FlightSoftwareModel     SimHal (fsw::Hal over the bus) + runs flight software in lockstep
  TelemetryLogger         truth vs flight software beliefs to CSV
  HopScenario             builds the hop scenario
sim_cli/       Headless runner (real time, fast, slow motion)
tests/         Minimal in-house test harness (swap for Catch2/GoogleTest when needed)
godot_bridge/  (planned) GDExtension wrapper around sim_core/sim_vehicle
godot_project/ (planned) Godot 4 scenes, Blender models, shaders, HUD
```

Layering rule: `flight_sw` <- nothing; `sim_core` <- nothing; `sim_vehicle` <- `sim_core`, `flight_sw`; apps <- `sim_vehicle`. Never include upward. The sim must never depend on Godot.

## Architecture

### The step

Base rate 1 kHz (dt = 1 ms). Sim time is `step * dt`, never a clock. Each base step, `Simulation::step()` does:

1. `prepare(t)` on every `ContinuousSystem`: discrete events (engine ignition, staging), and derived truth such as accelerations that sensors read this step.
2. `Scheduler::run`: every discrete `Model` whose rate is due, in the order added. Order models by data flow: sensors, then flight software, then loggers.
3. `integrate(t, dt)` on every `ContinuousSystem`: pick up the latest actuator commands from the bus and advance continuous state with RK4.

### Continuous vs discrete

- **Continuous state** (position, velocity, attitude, rates, propellant masses, actuator positions, later slosh and bending modes) lives in a `ContinuousSystem` and is integrated together by one `derivatives(state, u)` function. RK4 calls it 4 times per step.
- **Discrete models** (sensors, flight software, thermal, comms, logging) derive from `Model`, run at their own rate and talk only through the `Bus`.

### Rate groups

Every model rate must divide the base rate exactly (the scheduler throws otherwise). Planned rates:

| Model | Rate |
|---|---|
| Rigid body, slosh, bending, actuators | 1 kHz in RK4 |
| Engine internals (turbopumps, chamber) | subcycled 5 to 10 kHz, or implicit |
| Aero, thrust, low-order gravity | inside `derivatives()` (table lookups) |
| High-order gravity harmonics | 10 to 100 Hz correction |
| Tank thermodynamics | 50 to 100 Hz |
| Tile thermal | 10 to 50 Hz, multithreaded |
| IMU | 500 Hz (current) |
| Radar altimeter | 50 Hz (current) |
| GNSS | 5 to 10 Hz |
| Flight software | 100 Hz (current) |
| Snapshot to renderer | 60 to 120 Hz |
| Logging | 100 Hz (current) |

### Rules

- **No allocation in the step loop.** Allocate in constructors or `initialize()`. Use fixed-size Eigen types.
- **Resolve bus channels in `initialize()`** and keep the reference. `Bus::channel()` does a hash map lookup.
- **Flight software never reads truth.** It only uses `fsw::Hal`. `SimHal` is the seam where a real flight computer plugs in later for hardware-in-the-loop.
- **Determinism.** Same seed gives a bit-identical flight. Use `sim::Rng` (own normal/uniform on top of `std::mt19937_64`), never `std::normal_distribution` or `std::uniform_real_distribution`, which differ between MSVC and GCC. Give each model its own stream via `deriveSeed(masterSeed, "model_name")`.
- **Heavy physics is precomputed offline** into tables: aero coefficients (CFD with OpenFOAM or SU2), combustion properties (NASA CEA), upper atmosphere (NRLMSISE-00).
- **Animate actual actuator positions** (from the actuator models), never commanded ones, in logs and in the renderer.
- **Sensor models** get noise, bias, drift, scale factor, quantization, sample rate, latency, saturation, failure modes and a physical mounting location. Each has an `ideal` switch for truth-mode tuning.
- **Data-oriented layout** for things that repeat (39 engines, ~18,000 heat shield tiles): structure of arrays, not one object per tile.
- **Pacing lives only in `Pacer`.** Real time, as fast as possible (Monte Carlo), or slow motion.

### Frames and units

- SI units everywhere. Angles in radians internally. Put units in comments: `// [m/s]`.
- Current placeholder: flat local ENU world frame (z up), uniform gravity. Body frame +z along the vehicle axis toward the nose; `attitude` rotates body to world.
- Planned: WGS-84, ECI/ECEF, site ENU frames (phase 0, item 2). Godot is Y-up with -Z forward; do the conversion in exactly one function in the bridge.

## Conventions

- C++20, warnings clean on MSVC `/W4`, GCC and Clang `-Wall -Wextra -Wpedantic -Wshadow`.
- Types `PascalCase`, functions and variables `camelCase`, members `trailing_`, constants `kName`.
- Placeholders are labeled as such in code. Mark future work with `TODO(phase N, item M)` matching the build order below.
- Vehicle data: anything not from a public source is an estimate. Keep every vehicle number in a parameters file with a comment saying where it came from (source link, estimate, or derived), so it can be updated as SpaceX publishes more. SpaceX internal data is not public; do not imply otherwise.
- Docs and comments: avoid em dashes.

## Vehicle reference (Starship V3, public info as of Nov 2026)

- Super Heavy: 33 Raptor 3 engines; three grid fins (down from four on V2), each about 50% larger, mounted lower to reduce hot-staging heat, with actuators inside the fuel tank and catch points for the tower.
- Ship: six Raptor 3 engines (three sea-level, three vacuum), four flaps, hexagonal tile heat shield.
- Integrated hot staging: no disposable interstage; the Booster forward dome takes the Ship's plume directly (protected by tank pressure and steel shielding); interstage actuators retract after separation.
- Redesigned fuel transfer tube lets all 33 engines light simultaneously.
- Pad: two separate quick disconnect systems instead of one.

## Rendering plan (not started)

Godot 4, driven through a GDExtension (`godot_bridge/`) that wraps the sim. The sim runs on its own thread and publishes snapshots; the bridge interpolates between the two latest for each render frame. Godot's physics is not used: the rocket is a `Node3D` whose transform comes from the sim. Blender models: each moving part (gimbaled engines, grid fins, flaps, legs) is its own object with its origin at the pivot and local axes aligned with its rotation axes; export glTF or import `.blend` directly. Large distances: camera-relative rendering / origin shifting first, Godot double-precision build later if needed. Plumes with `GPUParticles3D` plus a shader driven by throttle and ambient pressure; entry glow driven by the heating model. Telemetry HUD in Godot `Control` nodes; detailed analysis by plotting CSV logs in Python.

## Mission control plan (phase 10)

CCSDS packets and an XTCE telemetry and command database between vehicle and ground, served by Yamcs, with Open MCT displays and a webcast-style HUD.

## Build order

Systems in dependency order. Update the Status column as work lands (To do, In progress, Blocked, Done). Flight software (phase 9) grows alongside the sim from phase 2 onward.

| # | Phase | System | Status | Notes |
|---|---|---|---|---|
| 1 | 0 Foundations | Sim executive | In progress | Done: fixed step, rate-group scheduler, deterministic seeds, CSV logging, real-time pacer. To do: save/restore, async logging |
| 2 | 0 Foundations | Reference frames and Earth model | To do | WGS-84, ECI/ECEF, Earth rotation, site ENU frames, body frames |
| 3 | 0 Foundations | Gravity | To do | J2 first, then truncated EGM2008 (currently uniform placeholder) |
| 4 | 0 Foundations | Atmosphere | In progress | Done: US76 0 to 86 km. To do: NRLMSISE-00 above ~80 km |
| 5 | 0 Foundations | Wind and weather | To do | Altitude wind profiles, Dryden/von Karman turbulence, gusts |
| 6 | 0 Foundations | Site geometry | To do | Starbase pad, tower, terrain, ocean surface |
| 7 | 1 Vehicle core | 6DOF rigid body dynamics | In progress | Done: 6DOF with quaternions for the test hopper. To do: variable-mass terms, Booster, Ship, stacked config |
| 8 | 1 Vehicle core | Mass properties | To do | Dry mass breakdown, tank geometry, live CG and inertia from propellant levels |
| 9 | 2 Propulsion | Raptor engine model | To do | 33 Booster + 6 Ship; full-flow staged combustion, start/stop sequences, Isp vs ambient, failure modes (hopper has a placeholder engine) |
| 10 | 2 Propulsion | Propellant feed system | To do | Main and header tanks, transfer tube, valves, chill-down, NPSH, cavitation |
| 11 | 2 Propulsion | Pressurization | To do | Autogenous press, tank pressure control, ullage temp, vents |
| 12 | 2 Propulsion | Cryogenic propellant thermodynamics | To do | Boil-off, stratification, subcooling, heat leak |
| 13 | 2 Propulsion | Slosh | To do | Spring-mass or pendulum per tank, baffle damping |
| 14 | 2 Propulsion | Propellant settling and ullage | To do | Low/zero-g behavior, settling burns before relight |
| 15 | 2 Propulsion | Reaction control system | To do | Ullage-gas thrusters, thrust vs tank pressure, minimum impulse bit |
| 16 | 2 Propulsion | TVC actuators | In progress | Done: first-order response with angle and rate limits. To do: motor dynamics, backlash, power draw |
| 17 | 3 Aero and structures | Vehicle aerodynamics | To do | Full Mach range, extreme AoA, base drag, plume effects; tables from research/CFD |
| 18 | 3 Aero and structures | Booster grid fins | To do | V3: 3 fins; transonic choking, actuator dynamics, hinge moments |
| 19 | 3 Aero and structures | Ship flaps | To do | 4 flaps, electric actuators, hinge moments, rate limits, power draw |
| 20 | 3 Aero and structures | Structural loads | To do | Axial, bending, shear; max-q; pressure-stabilized tanks |
| 21 | 3 Aero and structures | Flexible body dynamics | To do | First bending modes coupling into IMU and TVC |
| 22 | 4 Thermal | Entry heating | To do | Convective (Sutton-Graves first) and radiative heating |
| 23 | 4 Thermal | Thermal protection system | To do | Per-tile temperatures, heat soak, tile loss and burn-through |
| 24 | 4 Thermal | Booster thermal environment | To do | Base heating in boostback and landing; hot-staging plume heating |
| 25 | 4 Thermal | Engine bay thermal and fire detection | To do | Compartment temps, leaks, venting |
| 26 | 4 Thermal | Plasma blackout | To do | Comms effects during entry |
| 27 | 5 Power and avionics | Electrical power | To do | Batteries, buses, actuator peak loads, voltage sag |
| 28 | 5 Power and avionics | Avionics | To do | Redundant computers with voting, buses, timing jitter, latency, faults |
| 29 | 5 Power and avionics | Communications | To do | Starlink links, ground stations, downlink budget |
| 30 | 6 Sensors and gauges | Navigation sensors | In progress | Done: IMU (noise, bias, random walk, quantization), radar altimeter (noise, range, dropouts). To do: lever arms, misalignment, latency, GNSS, star trackers, redundancy |
| 31 | 6 Sensors and gauges | Engine instrumentation | To do | Chamber/preburner pressures, temps, pump speeds, valve positions, vibration |
| 32 | 6 Sensors and gauges | Propellant gauges | To do | Tank levels, pressures, temps, header tank levels |
| 33 | 6 Sensors and gauges | Actuator feedback | To do | Position, motor current and temp for gimbals, fins, flaps |
| 34 | 6 Sensors and gauges | Structural sensors | To do | Strain gauges and accelerometers along the airframe |
| 35 | 6 Sensors and gauges | Thermal sensors | To do | Thermocouples behind tiles, on steel, in engine bays |
| 36 | 6 Sensors and gauges | Power sensors | To do | Battery V/I/temp/SoC, bus voltages |
| 37 | 6 Sensors and gauges | Cameras | To do | Rendered onboard views with exposure, noise, compression, sent over comms model |
| 38 | 6 Sensors and gauges | Ground sensors | To do | Tower arm positions, load cells, pad and tank farm |
| 39 | 7 Ground and tower | Pad and propellant loading | To do | Loading sequence, chill-down, two quick disconnects (V3), deluge, clamps |
| 40 | 7 Ground and tower | Tower and catch arms | To do | Arm kinematics and control, catch-point contact, load absorption |
| 41 | 8 Multibody events | Hot staging | To do | Plume on booster dome, separation forces, contact dynamics, interstage actuators |
| 42 | 8 Multibody events | Booster flip and boostback | To do | Pitch-around with propellant motion and relights |
| 43 | 9 Flight software | Navigation (EKF) | In progress | Done: vertical complementary filter (IMU + altimeter). To do: full EKF with attitude, GNSS, sensor voting |
| 44 | 9 Flight software | Guidance | In progress | Done: hop guidance with braking limit from the throttle floor. To do: ascent, boostback targeting, entry, powered landing (G-FOLD style) |
| 45 | 9 Flight software | Control | In progress | Done: cascade altitude/velocity PID to throttle. To do: attitude control via TVC, actuator allocation, gain scheduling |
| 46 | 9 Flight software | Sequencing | In progress | Done: hop mode state machine with touchdown confirmation. To do: countdown, staging, relights, flip, catch |
| 47 | 9 Flight software | FDIR | To do | Engine-out compensation, sensor failures, aborts |
| 48 | 10 Mission control and validation | Ground segment | To do | Yamcs or Open MCT telemetry/commanding, gauges, webcast-style HUD |
| 49 | 10 Mission control and validation | Fault injection and Monte Carlo | To do | Scripted failures, thousands of randomized runs |
| 50 | 10 Mission control and validation | Validation against real flights | To do | Extract webcast telemetry (speed, altitude, propellant, engines) and compare |

## Lessons already learned (keep these in mind)

- The throttle floor limits braking. At 40% minimum throttle and near full propellant, the hopper can only decelerate a climb at about 1.5 m/s², so guidance must limit climb rate to `sqrt(2 * a_brake * distance_to_go)` or it overshoots (it went to 59 m on a 50 m hop before this fix). The same physics drives Starship's hoverslam landing.
- Declare touchdown on contact (low altitude AND stopped, confirmed over time), not at an altitude threshold, or the engine cuts early and the vehicle drops the last stretch.
