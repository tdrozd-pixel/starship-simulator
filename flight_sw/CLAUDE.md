# flight_sw

Flight software: navigation, guidance, control, sequencing (build order items 43 to 47). Namespace `fsw`.

- **Depends on nothing else in the repo, not even Eigen.** It must build for a real flight computer against a real HAL. Standard library only.
- `Hal.hpp` is the only way to touch the world (`timeSec`, `readImu`, `readAltimeter`, `readGnss`, `commandActuators` per engine, `publishTelemetry`). Never read truth; if you need data, add a sensor and a HAL call. `sim_vehicle/SimHal` implements it over the bus.
- Runs at 100 Hz in lockstep with the sim. No allocation in `step()`/update paths; no clocks, threads or I/O.
- Contents: `Pid`, `VerticalEstimator` (complementary filter), `HorizontalEstimator` (GNSS + IMU Kalman filter per axis), `Geodesy` (WGS-84, independent of the sim copy), `HopFlightComputer` (hop modes, divert guidance, cascade control, per-engine TVC with roll).

## Lessons learned

- The throttle floor limits braking. At 40% minimum throttle and near full propellant, the hopper can only decelerate a climb at about 1.5 m/s², so guidance must limit climb rate to `sqrt(2 * a_brake * distance_to_go)` or it overshoots (it reached 59 m on a 50 m hop before this). The same physics drives Starship's hoverslam landing.
- Declare touchdown on contact (low altitude AND stopped, confirmed over time), not at an altitude threshold, or the engine cuts early and the vehicle drops the last stretch.
- The Ship is slow in attitude: the lever arm (CG to gimbal, ~9.5 m) is about the radius of gyration (~10 m), full gimbal gives only ~0.25 rad/s^2, and a gimbal deflection first pushes the base the wrong way. With horizontal loops near the attitude bandwidth the hop whirled in a circular limit cycle; keep horizontal position/velocity gains well below the attitude loop.
- Navigate the point where the IMU is. Filtering the base while the IMU measures the CG made angular acceleration look like translation (0.1 rad/s^2 x 13 m = 1.3 m/s^2) and destabilized the loop.
- Gyro-only attitude drifts (placeholder IMU: ~0.6 deg in 100 s). A level controller cannot see that, and it also corrupts the GNSS lever-arm correction (0.6 deg x 33 m = 0.35 m). The horizontal filter's apparent-acceleration bias measures the tilt error; feeding it back into the attitude (GNSS levelling) fixed both.
