# sim_vehicle

Vehicle models, sensors and the flight software bridge. Namespace `sim::vehicle`. Depends on `sim_core` and `flight_sw` only.

- `HopperDynamics`: 6DOF rigid body (quaternions) about the CG for the Ship V3 hop: three sea-level Raptors at their gimbal points, per-engine throttle and gimbal actuators, thrust vs ambient pressure, axial plus cross-flow drag with a center-of-pressure moment, ground contact at the base. `state().position` is the CG; use `basePosition()` / `pointWorld()` for the base, sensors and the truth stream. A `ContinuousSystem`.
- `HopperParams.hpp`: every vehicle and site number with its source or "estimate" (see `docs/vehicle_reference.md`). `LaunchSite` puts the sim ENU origin on the landing target in WGS-84.
- `Sensors`: `ImuModel` (500 Hz, at the CG), `RadarAltimeterModel` (50 Hz, slant range from the base), `GnssModel` (100 Hz model, 10 Hz solutions in WGS-84 with latency, antenna lever arm, RTK or standalone errors, placeholder satellite counts on their own RNG stream), `GimbalFeedbackModel` (measured gimbal positions, one-sample latency). Each has an `ideal` switch and its own RNG stream from `deriveSeed`.
- `FlightSoftwareModel`: `SimHal` (implements `fsw::Hal` over the bus) and runs the flight computer at 100 Hz. This is the hardware-in-the-loop seam; keep truth out of it.
- `Messages.hpp`: bus message structs.
- `Downlink`: `DownlinkQueue` (fixed ring the flight computer's packets go into via `SimHal::downlink`) and `DownlinkModel` (placeholder link: bit rate, latency, jitter, loss; FIFO) delivering to a `TelemetrySink` (`TelemetrySink.hpp`). Consoles only ever get data through this path.
- `TelemetryLogger`: truth vs flight software beliefs to CSV at 100 Hz.
- `TruthSnapshotPublisher`: 100 Hz snapshots into a `TruthSnapshotSink`. No networking here; `sim_server` provides the sink.
- `HopScenario`: wires the hop and hands the flight software its vehicle knowledge (mass properties, engine model, antenna and IMU positions) and the target coordinates. Model order: sensors, gimbal feedback, flight software, downlink, loggers.

Rules that bite here: publish actual actuator positions (from the actuator state), never commanded ones; no allocation in `update()`/`derivatives()`; repeated parts (engines, tiles) as structure of arrays.
