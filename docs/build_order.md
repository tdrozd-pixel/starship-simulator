# Build order

Systems in dependency order. Update the Status column as work lands (To do, In progress, Blocked, Done). Flight software (phase 9) grows alongside the sim from phase 2 onward. `TODO(phase N, item M)` tags in code refer to the `#` column.

| # | Phase | System | Status | Notes |
|---|---|---|---|---|
| 1 | 0 Foundations | Sim executive | In progress | Done: fixed step, rate-group scheduler, deterministic seeds, CSV logging, real-time pacer, standalone `sim_server` on the LAN with truth stream v1 over WebSocket and browser outside view, telemetry stream v1 (`/ws/telemetry`) carrying downlinked CCSDS packets. To do: save/restore, async logging, sessions and console roles, configurable crew (staffed and consolidated positions), command log for replay |
| 2 | 0 Foundations | Reference frames and Earth model | In progress | Done: WGS-84 geodetic/ECEF/local ENU conversions (sim and an independent flight software copy); sim ENU is the tangent plane at the Starbase Pad 2 target. To do: ECI, Earth rotation, dynamics in ECEF |
| 3 | 0 Foundations | Gravity | To do | J2 first, then truncated EGM2008 (currently uniform placeholder) |
| 4 | 0 Foundations | Atmosphere | In progress | Done: US76 0 to 86 km. To do: NRLMSISE-00 above ~80 km |
| 5 | 0 Foundations | Wind and weather | Done | Altitude wind profiles, seeded Dryden/von Karman turbulence, one-cosine gusts, and coupling to air-relative hopper drag. Default weather is an illustrative generic estimate, not Starship or Starbase flight data |
| 6 | 0 Foundations | Site geometry | To do | Starbase pad, tower, terrain, ocean surface |
| 7 | 1 Vehicle core | 6DOF rigid body dynamics | In progress | Done: 6DOF with quaternions about the CG, Ship V3 hop configuration (three engines at their gimbal points), cross-flow drag with a center-of-pressure moment. To do: variable-mass terms, Booster, stacked config |
| 8 | 1 Vehicle core | Mass properties | In progress | Done: placeholder model (dry shell plus propellant column, fixed CG, inertia scaled with propellant). Dry mass is an estimate. To do: dry mass breakdown, tank geometry, live CG and inertia from propellant levels |
| 9 | 2 Propulsion | Raptor engine model | In progress | Done: Raptor 3 sea-level performance (280 tf, 350 s vacuum Isp) with thrust vs ambient pressure, 40% throttle floor, three Ship sea-level engines. To do: 33 Booster engines, RVacs, full-flow staged combustion, start/stop sequences, failure modes |
| 10 | 2 Propulsion | Propellant feed system | To do | Main and header tanks, transfer tube, valves, chill-down, NPSH, cavitation |
| 11 | 2 Propulsion | Pressurization | To do | Autogenous press, tank pressure control, ullage temp, vents |
| 12 | 2 Propulsion | Cryogenic propellant thermodynamics | To do | Boil-off, stratification, subcooling, heat leak |
| 13 | 2 Propulsion | Slosh | To do | Spring-mass or pendulum per tank, baffle damping |
| 14 | 2 Propulsion | Propellant settling and ullage | To do | Low/zero-g behavior, settling burns before relight |
| 15 | 2 Propulsion | Reaction control system | To do | Ullage-gas thrusters, thrust vs tank pressure, minimum impulse bit |
| 16 | 2 Propulsion | TVC actuators | In progress | Done: first-order response with angle and rate limits, per engine. To do: motor dynamics, backlash, power draw |
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
| 29 | 5 Power and avionics | Communications | In progress | Done: placeholder downlink model (bit rate, latency with jitter, seeded loss, FIFO) carrying CCSDS packets. To do: Starlink links, ground stations, downlink budget, uplink |
| 30 | 6 Sensors and gauges | Navigation sensors | In progress | Done: IMU (noise, bias, random walk, quantization), radar altimeter (slant range from the base, noise, range, dropouts), GNSS receiver (WGS-84 output at 10 Hz, antenna lever arm, latency, Gauss-Markov plus white errors, RTK and standalone presets, dropouts, placeholder satellite counts). To do: IMU lever arm, misalignment, GNSS multipath near the tower, star trackers, redundancy, tower-relative sensing for the catch (reported tower radar; catch-arm lidar is a candidate design, not public) |
| 31 | 6 Sensors and gauges | Engine instrumentation | To do | Chamber/preburner pressures, temps, pump speeds, valve positions, vibration |
| 32 | 6 Sensors and gauges | Propellant gauges | To do | Tank levels, pressures, temps, header tank levels |
| 33 | 6 Sensors and gauges | Actuator feedback | In progress | Done: gimbal position sensors per axis per engine (noise, bias, quantization, latency), downlinked. To do: motor current and temperature, fins, flaps |
| 34 | 6 Sensors and gauges | Structural sensors | To do | Strain gauges and accelerometers along the airframe |
| 35 | 6 Sensors and gauges | Thermal sensors | To do | Thermocouples behind tiles, on steel, in engine bays |
| 36 | 6 Sensors and gauges | Power sensors | To do | Battery V/I/temp/SoC, bus voltages |
| 37 | 6 Sensors and gauges | Cameras | To do | Camera sensor models (mount point, FOV, exposure) and video downlink model; feeds rendered in the browser inside console layouts, degraded by link quality |
| 38 | 6 Sensors and gauges | Ground sensors | To do | Tower arm positions, load cells, pad and tank farm |
| 39 | 7 Ground and tower | Pad and propellant loading | To do | Loading sequence, chill-down, two quick disconnects (V3), deluge, clamps |
| 40 | 7 Ground and tower | Tower and catch arms | To do | Arm kinematics and control, catch-point contact, load absorption |
| 41 | 8 Multibody events | Hot staging | To do | Plume on booster dome, separation forces, contact dynamics, interstage actuators |
| 42 | 8 Multibody events | Booster flip and boostback | To do | Pitch-around with propellant motion and relights |
| 43 | 9 Flight software | Navigation (EKF) | In progress | Done: vertical complementary filter (IMU + tilt-corrected altimeter), gyro-integrated attitude with GNSS levelling, GNSS + IMU horizontal Kalman filter (position, velocity, apparent acceleration bias) at the IMU point with lever-arm and latency compensation. To do: full EKF, GNSS height aiding, heading reference, sensor voting |
| 44 | 9 Flight software | Guidance | In progress | Done: hop guidance with braking limit from the throttle floor, divert to a target given in WGS-84 coordinates with a braking-limited approach speed, alignment gate before descent. To do: ascent, boostback targeting, entry, powered landing (G-FOLD style) |
| 45 | 9 Flight software | Control | In progress | Done: cascade altitude/velocity PID to throttle through the pressure-corrected thrust model, attitude PD via common gimbal, roll via differential gimbal across three engines, horizontal position/velocity PI. To do: engine-out allocation, gain scheduling |
| 46 | 9 Flight software | Sequencing | In progress | Done: hop mode state machine with touchdown confirmation. To do: countdown, staging, relights, flip, catch |
| 47 | 9 Flight software | FDIR | To do | Engine-out compensation, sensor failures, aborts |
| 48 | 10 Mission control and validation | Ground segment | In progress | Yamcs telemetry/command database, Open MCT web consoles per position (downlinked telemetry only), commanding through the uplink path with per-position permissions, per-position camera views, browser outside view and webcast-style HUD. Done so far: CCSDS telemetry from the flight computer (`docs/telemetry/gnc_packets.md`), `/ws/telemetry`, and the GNC console at `/gnc/` (interim, before Yamcs/Open MCT) |
| 49 | 10 Mission control and validation | Fault injection and Monte Carlo | To do | Scripted failures, live fault injection from the supervisor station, thousands of randomized runs |
| 50 | 10 Mission control and validation | Validation against real flights | To do | Extract webcast telemetry (speed, altitude, propellant, engines) and compare |
| 51 | 10 Mission control and validation | Virtual controllers (optional) | To do | Only after the staffed mission control loop works end to end. Automated positions that read downlinked telemetry only, follow flight rules, escalate to the flight director |
