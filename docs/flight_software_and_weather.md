# Hopper flight software and weather

This note records the current test-hopper implementation and its assumptions.
It is not a Starship flight-control design or a source of Starship vehicle data.

## Flight software navigation and TVC

The hop flight computer uses only its `fsw::Hal` sensor samples and commands.
It does not read simulator truth:

- Vertical altitude and velocity use the IMU specific force and radar-altimeter
  range in the existing vertical complementary estimator.
- Attitude is integrated from measured body rates. While on the pad, the
  accelerometer provides a gravity reference to align tilt. There is no
  magnetometer, star tracker, or other heading reference, so yaw is not
  independently aligned.
- East and north velocity are obtained by rotating measured specific force into
  the ENU frame and integrating it. These horizontal velocity estimates are
  IMU-only dead reckoning and will drift; they are not corrected by GNSS.

The attitude controller uses a proportional attitude-error term and measured
body-rate damping to command pitch and yaw gimbal angles. Its desired tilt
provides horizontal acceleration to damp estimated east/north velocity. The
command is limited by the configured gimbal angle. Torque-to-gimbal scaling
uses the hopper's estimated transverse inertia, engine lever arm, and commanded
thrust; the vehicle dynamics separately model the gimbal actuator response and
rate limit. Current controller tuning is in `fsw::HopConfig` in
`flight_sw/include/fsw/HopFlightComputer.hpp`.

This is a single-engine test hopper: its engine can control pitch and yaw, but
cannot directly control roll. Actuator allocation and gain scheduling remain
future work. The model's current mass/inertia assumptions are placeholders.

## Weather configuration

The full-run hop takes weather settings from
`sim::vehicle::HopScenarioOptions::weather` in
`sim_vehicle/include/sim/vehicle/HopScenario.hpp`. There is no separate weather
file or server command-line override; rebuild after changing the scenario
configuration.

- `profile` is a piecewise-linear altitude profile of ENU wind in m/s.
- `turbulence.sigma` sets per-axis RMS velocity in m/s, and
  `turbulence.scaleLength` sets the correlation scale in m. The seeded
  Dryden-inspired process uses the scenario seed.
- Each entry in `gusts` has an ENU peak `amplitude` in m/s, `startTime` in
  seconds, and `duration` in seconds. A one-cosine gust is zero at the start
  and end of that interval, and reaches its specified peak halfway through.
  Gust entries add together.

The current default scenario constructor adds an illustrative eastward gust:

```cpp
weather.gusts.push_back({Eigen::Vector3d(100.0, 0.0, 0.0), 8.0, 1.0});
```

That means a 100 m/s peak eastward gust from T+8 s to T+9 s, peaking at T+8.5 s.
It is an intentionally configured test condition, not typical weather and not
Starbase flight data. Clear `weather.gusts` in scenario setup to remove it, or
change its amplitude, start time, or duration to adjust the event.

Weather is applied to air-relative hopper drag. It does not directly command
the vehicle or flight software; any attitude response occurs through sensed
motion and the normal flight-control loop.
