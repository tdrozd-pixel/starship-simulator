# Starship Simulator

Engineering-grade simulation of SpaceX Starship (V3): launch, hot staging, Booster boostback and tower catch, Ship entry and landing. Every system, gauge and sensor is simulated, and the vehicles are flown by real flight software (written in this repo) that only sees simulated sensors. The goal is extreme realism; schedule is not a constraint. It is also a portfolio project, so code quality, tests and documented assumptions matter.

Scope: a **mission control simulation**, not a game. A simulation server streams live data to clients on the local network, and anywhere from one person to a group (for example friends with laptops) can take console positions (propulsion, GNC, power, flight director, and so on) and work the flight together in real time. Crew size is configurable: positions without a person are consolidated into someone else's seat (and later, optionally, run by automated virtual controllers). Every client is a web page in a browser, including all 3D views, the way modern operations teams (SpaceX included) run most of their tooling, so joining takes a URL and nothing to install. Realism wins over fun wherever they conflict: no scores, no shortcuts, no truth on the consoles.

Owner: Taras Drozd. Primary languages he knows best are Python and Java; this project is C++20.

## Current status

The framework skeleton is in place and flies a single-engine **test hopper** (placeholder vehicle, not Starship) through a 50 m vertical hop with modeled IMU and radar altimeter noise. That proves the sim executive, rate-group scheduler, bus, RK4 integrator, pacing, logging and the flight software seam.

`sim_server` runs the hop in real time and serves a browser outside view (three.js, Ship-sized placeholder cylinder) over the LAN via the truth stream (`docs/protocol/truth_stream.md`). There are no consoles, sessions, roles or telemetry downlink yet. Next work follows the build order below.

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

build/sim_server/sim_server --host 127.0.0.1   # then open http://127.0.0.1:8080/
build/sim_server/sim_server                    # LAN: binds 0.0.0.0:8080, prints a URL per address
build/sim_server/sim_server --time-scale 0.25 --seed 42 --ideal-sensors --autostart

python -I web/tests/run_web_tests.py           # web client unit tests (Playwright, headless Edge/Chromium)
python -I tools/e2e/server_smoke.py            # end-to-end: protocol, restart, stalled client, shutdown
```

On Windows with the Visual Studio generator, binaries land in `build/<target>/Release/` (for example `build/sim_server/Release/sim_server.exe`).

`sim_server` holds each run on the pad until a browser connects (`--autostart` skips that). The first configure downloads civetweb v1.16 (HTTP + WebSocket); `-DSIM_BUILD_SERVER=OFF` skips the server. On first LAN run Windows asks about the firewall: allow Private networks only. Use `--host 127.0.0.1` in tests and agents to avoid the prompt.

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
  TruthSnapshotPublisher  100 Hz truth snapshots into a TruthSnapshotSink (no networking)
  HopScenario             builds the hop scenario
sim_cli/       Headless runner (real time, fast, slow motion)
sim_server/    Real-time sim process on the LAN (the only code that does networking)
  SimRunner               sim thread: build, run paced, hold, restart; wait for a client
  SnapshotRing            lock-free SPSC ring, sim thread -> broadcaster
  TruthServer             civetweb: static files from web/, /ws/truth, per-client send queues
  WireFormat, JsonMessages  truth stream encoding (docs/protocol/truth_stream.md)
  Planned: telemetry downlink, commands, sessions and roles, command log
web/           Browser clients, plain ES modules, no build step
  js/frames.js            the only sim-frame to renderer-frame conversion
  js/protocol.js, interpolator.js, truthClient.js   pure, testable without a DOM
  vendor/three/           three.js 0.180.0, vendored unmodified (never edit)
  Planned: session lobby, Open MCT consoles with per-position camera views, supervisor station
docs/protocol/ Versioned wire protocols (truth_stream.md)
tools/e2e/     End-to-end tests that run the real server (server_smoke.py)
tests/         Minimal in-house test harness (swap for Catch2/GoogleTest when needed)
```

Layering rule: `flight_sw` <- nothing; `sim_core` <- nothing; `sim_vehicle` <- `sim_core`, `flight_sw`; apps (`sim_cli`, `sim_server`) <- `sim_vehicle`. Never include upward. Only `sim_server` may use networking (civetweb). The sim must never depend on any client. Clients run in separate processes, usually on other machines, and talk to `sim_server` only over the network through documented protocols; they never link the sim.

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
| Snapshot to renderer | 100 Hz (current; browser interpolates to display rate) |
| Logging | 100 Hz (current) |

### Rules

- **No allocation in the step loop.** Allocate in constructors or `initialize()`. Use fixed-size Eigen types.
- **Resolve bus channels in `initialize()`** and keep the reference. `Bus::channel()` does a hash map lookup.
- **The sim thread never blocks on networking.** Truth leaves the sim through `TruthSnapshotPublisher` into the SPSC `SnapshotRing`; a broadcaster thread fans out to per-client bounded queues, each drained by its own sender thread. A slow client only loses its own snapshots.
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
- Planned: WGS-84, ECI/ECEF, site ENU frames (phase 0, item 2). The web renderer has its own convention (three.js is Y-up; Cesium uses ECEF); do the conversion from sim frames in exactly one module in `web/`.

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

## Runtime architecture (planned)

Flight controllers at consoles watch live telemetry and make real-time decisions, with visuals alongside. One machine runs the server; everyone else joins from a browser on the same local network. The sim is the authority and everything else is a client.

```
                               one host machine                          other laptops on the LAN
  +------------------------------------------------------------------+
  |  sim_server (sim_core + sim_vehicle, flight software in lockstep)|
  |  owns sim time and truth; session and roles; command log         |-- truth snapshots ---> browser outside view, webcast HUD
  +------------------------------------------------------------------+   (WebSocket, binary)
        |                                                             \-- camera feeds -----> camera views inside consoles
        |                                                                 (via video downlink model)
        | downlink (via comms model)        ^ uplink (via comms model)
        v CCSDS packets                     | commands                  <--- supervisor station (browser,
  +------------------------------------------------------------------+         privileged: faults, time, weather)
  |  Yamcs (XTCE telemetry/command database, archive, web API)       |
  +------------------------------------------------------------------+
        | WebSocket                         ^ commands
        v                                   |
     Open MCT consoles in the browser: FLIGHT, PROP, GNC, POWER, THERMAL, STRUCT, COMMS, ...
```

- **`sim_server` is a standalone real-time process.** It owns sim time and truth, and `Pacer` still sets the pace. A client crashing, stalling, losing Wi-Fi or dropping frames never affects the sim. A client that reconnects catches up from current state (and from the Yamcs archive for history), not by asking the sim to wait.
- **Browser first.** Every client is a web page served on the LAN: lobby, consoles, 3D views, HUD, supervisor. Nothing to install on participant laptops; any OS works. There is no native renderer: 3D is rendered in the browser so it can be embedded directly in console layouts.
- **Sessions and roles.** The host starts a scenario; participants open the server URL, pick a console position and a name. Each position sees the displays and holds the command permissions of that discipline (PROP can command propulsion, not GNC), enforced on the server, the way real consoles have command authority per position. The flight director position can see everything downlinked and gives the go/no-go calls.
- **Configurable crew size, from one person to a full room.** The position list and how each position is staffed come from a crew configuration chosen in the lobby (presets plus a custom option, stored as a file so sessions are repeatable). Each position is in one of these states:
  - *Staffed*: a person holds it.
  - *Consolidated*: merged into another staffed position, which then gets its displays and command authority (for example PROP and POWER consolidated into one seat). Real control rooms consolidate positions during quiet phases, so this is realistic, not a shortcut.
  - *Automated* (optional, later; build order item 51): a virtual controller holds it. It watches the same downlinked telemetry a person would (never truth), makes the routine calls, follows the flight rules, and escalates anomalies to the flight director rather than silently fixing them. It is deliberately no better than a competent human, and its calls and commands go in the session log like anyone else's.

  Example presets: *Solo* (one person holds every position consolidated, with scripted faults in place of a human supervisor), *Small crew* (2 to 4 people on consolidated positions), *Full room* (one person per position plus a human supervisor). Once virtual controllers exist, Solo and Small crew can hand some positions to automation instead. The crew configuration can change mid-session (a friend joins or leaves and positions are consolidated or split again); the handover is logged. The flight software flies the vehicle regardless of staffing. Positions only monitor, decide and command, as in a real control room.
- **LAN scope and security.** The server binds to the local network only and is never exposed to the internet by default. A session code to join and a separate password for the supervisor role are enough for friends; this is not a hardened public service.
- **Time.** Sim time from the server is the only clock. Clients show mission elapsed time from telemetry timestamps, never their local clock. Only the supervisor can pause or change the time scale, and it applies to everyone.
- **Three output streams, with different trust levels.**
  - *Truth snapshots* (60 to 120 Hz, binary over WebSocket) go only to the outside view and webcast HUD, which play the "outside camera" and may see truth. Consoles never subscribe to this stream.
  - *Camera feeds* go to the camera views embedded in consoles. Each onboard camera is a sensor model (phase 6, item 37) with a mount point, field of view and exposure, and its feed passes through a video downlink model (frame rate, latency, bandwidth, dropouts, blackout) before it leaves the server. The browser renders the camera's view from that degraded stream and adds noise and compression artifacts driven by link quality. If the link drops, the view freezes or goes dark, exactly as a real feed would.
  - *Downlinked telemetry* (CCSDS) goes through the comms model, with its latency, bandwidth limits, dropouts and plasma blackout, before it reaches Yamcs. **Consoles see only downlinked telemetry, never truth**, the same way flight software sees only `fsw::Hal`. A telemetry gap should look like a telemetry gap, not like perfect data.
- **Commands take the real path.** A console sends a command; Yamcs checks it against the XTCE command database; it goes uplink through the comms model (latency, blackout); flight software receives and executes it. Nothing on the ground writes sim state directly.
- **Determinism with humans in the loop.** Every command is recorded with the sim step at which it is applied on board. The seed plus the command log reproduces the run bit for bit, for replay and debrief.
- **Supervisor (instructor) station.** A separate privileged client that injects faults, changes weather and controls time (pause, slow motion). Controllers do not see its actions except through their effects on telemetry. The supervisor can be a person or a script: solo or without a volunteer, a fault script picked in the lobby (or randomized from a seed, without telling the crew which faults it contains) plays the role. Supervisor actions go in the same log so replays include them.
- **Visual clients are only stream consumers.** Any number of 3D views (outside view, and camera views inside each console) can run at once, and the rendering library can change without touching the sim.
- **Documented protocols.** The snapshot format and the session/role API are versioned and documented, like the XTCE database, so clients can be written in any language.

## Rendering plan (not started)

All rendering is in the browser (WebGL, WebGPU later if useful), in `web/`. No native renderer. One shared rendering module draws the vehicle and scene; it is used in two ways:

- **Outside view and webcast HUD.** Driven by the truth snapshot stream. CesiumJS is the leading candidate because it gives a WGS-84 globe, terrain and imagery for free, which fits ascent, entry and the planned ECEF frames (phase 0, item 2); three.js for close-in scenes (pad, tower catch, hot staging) if Cesium is too limiting there. Large distances: camera-relative rendering (Cesium does this already).
- **Camera views embedded in consoles.** Each position's layout includes the onboard cameras that discipline would watch: PROP gets engine bay and plume cameras, GNC gets views that show attitude and grid fin or flap motion, STRUCT gets cameras on the interstage and tank domes, and so on. They are panels inside the Open MCT layout (a custom Open MCT view), driven by the degraded camera feed stream, never the truth stream. The camera list per position lives in the crew configuration.

Both interpolate between the two latest samples per frame and must run on an ordinary laptop: several camera panels plus plots on one screen. Budget for that with LODs, a capped resolution per panel, and frame rate tied to the feed's own frame rate (a 30 fps camera feed does not need 60 fps rendering).

Models and effects:
- Blender models: each moving part (gimbaled engines, grid fins, flaps, legs) is its own object with its origin at the pivot and local axes aligned with its rotation axes. Export glTF, with web-weight LODs so laptops on Wi-Fi load it quickly.
- Animate actual actuator positions from the stream, never commanded ones.
- Plumes driven by throttle and ambient pressure; entry glow driven by the heating model.

Detailed analysis by plotting CSV logs in Python.

## Mission control plan (phase 10)

CCSDS packets and an XTCE telemetry and command database between vehicle and ground, served by Yamcs (Java, runs on the host). Operator consoles are an Open MCT web app served on the LAN and connected to Yamcs, with one layout per position (flight director, propulsion, GNC, power, thermal, structures, comms): live plots, limit alarms, event logs and command panels, all fed by downlinked telemetry only. Commanding follows the uplink path described in the runtime architecture, with command permissions per position. The webcast-style HUD on the 3D view is the public-facing display, separate from the consoles.

Later, optional: voice loops between positions (WebRTC in the browser), since real mission control coordinates by voice; flight rules and procedures shown alongside the consoles.

## Build order

Systems in dependency order. Update the Status column as work lands (To do, In progress, Blocked, Done). Flight software (phase 9) grows alongside the sim from phase 2 onward.

| # | Phase | System | Status | Notes |
|---|---|---|---|---|
| 1 | 0 Foundations | Sim executive | In progress | Done: fixed step, rate-group scheduler, deterministic seeds, CSV logging, real-time pacer, standalone `sim_server` on the LAN with truth stream v1 over WebSocket and browser outside view. To do: save/restore, async logging, sessions and console roles, configurable crew (staffed and consolidated positions), command log for replay |
| 2 | 0 Foundations | Reference frames and Earth model | To do | WGS-84, ECI/ECEF, Earth rotation, site ENU frames, body frames |
| 3 | 0 Foundations | Gravity | To do | J2 first, then truncated EGM2008 (currently uniform placeholder) |
| 4 | 0 Foundations | Atmosphere | In progress | Done: US76 0 to 86 km. To do: NRLMSISE-00 above ~80 km |
| 5 | 0 Foundations | Wind and weather | Done | Altitude wind profiles, seeded Dryden/von Karman turbulence, one-cosine gusts, and coupling to air-relative hopper drag. Default weather is an illustrative generic estimate, not Starship or Starbase flight data |
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
| 37 | 6 Sensors and gauges | Cameras | To do | Camera sensor models (mount point, FOV, exposure) and video downlink model; feeds rendered in the browser inside console layouts, degraded by link quality |
| 38 | 6 Sensors and gauges | Ground sensors | To do | Tower arm positions, load cells, pad and tank farm |
| 39 | 7 Ground and tower | Pad and propellant loading | To do | Loading sequence, chill-down, two quick disconnects (V3), deluge, clamps |
| 40 | 7 Ground and tower | Tower and catch arms | To do | Arm kinematics and control, catch-point contact, load absorption |
| 41 | 8 Multibody events | Hot staging | To do | Plume on booster dome, separation forces, contact dynamics, interstage actuators |
| 42 | 8 Multibody events | Booster flip and boostback | To do | Pitch-around with propellant motion and relights |
| 43 | 9 Flight software | Navigation (EKF) | In progress | Done: vertical complementary filter (IMU + altimeter), gyro-integrated attitude, and IMU-only ENU velocity dead reckoning. To do: GNSS aiding, full EKF and sensor voting |
| 44 | 9 Flight software | Guidance | In progress | Done: hop guidance with braking limit from the throttle floor. To do: ascent, boostback targeting, entry, powered landing (G-FOLD style) |
| 45 | 9 Flight software | Control | In progress | Done: cascade altitude/velocity PID to throttle, pitch/yaw attitude PD via TVC, and horizontal velocity damping. To do: actuator allocation and gain scheduling |
| 46 | 9 Flight software | Sequencing | In progress | Done: hop mode state machine with touchdown confirmation. To do: countdown, staging, relights, flip, catch |
| 47 | 9 Flight software | FDIR | To do | Engine-out compensation, sensor failures, aborts |
| 48 | 10 Mission control and validation | Ground segment | To do | Yamcs telemetry/command database, Open MCT web consoles per position (downlinked telemetry only), commanding through the uplink path with per-position permissions, per-position camera views, browser outside view and webcast-style HUD |
| 49 | 10 Mission control and validation | Fault injection and Monte Carlo | To do | Scripted failures, live fault injection from the supervisor station, thousands of randomized runs |
| 50 | 10 Mission control and validation | Validation against real flights | To do | Extract webcast telemetry (speed, altitude, propellant, engines) and compare |
| 51 | 10 Mission control and validation | Virtual controllers (optional) | To do | Only after the staffed mission control loop works end to end. Automated positions that read downlinked telemetry only, follow flight rules, escalate to the flight director |

## Lessons already learned (keep these in mind)

- The throttle floor limits braking. At 40% minimum throttle and near full propellant, the hopper can only decelerate a climb at about 1.5 m/s², so guidance must limit climb rate to `sqrt(2 * a_brake * distance_to_go)` or it overshoots (it went to 59 m on a 50 m hop before this fix). The same physics drives Starship's hoverslam landing.
- Declare touchdown on contact (low altitude AND stopped, confirmed over time), not at an altitude threshold, or the engine cuts early and the vehicle drops the last stretch.
- One stalled network client must never stall the others. The first server wrote to every client from one thread with blocking writes, and a single client that stopped reading froze everyone after about 9 s. Per-client queues and sender threads fixed it; `tools/e2e/server_smoke.py` keeps it fixed.
- Snapshot rates must divide 1 kHz: 60 Hz is rejected by the scheduler, so the truth stream runs at 100 Hz and the browser interpolates.
- A one-shot message (the final halted snapshot) needs a strictly increasing `step` and a guaranteed path through the ring, or clients that de-duplicate by step silently miss it.
- civetweb matches MIME types by path suffix (`.js=text/javascript`, with the dot), and browsers refuse ES modules served with the wrong type.
