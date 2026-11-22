# sim_core

Vehicle-agnostic framework and environment. Namespace `sim`. Depends on nothing in the repo (Eigen is allowed).

- `core/`: `Simulation` (executive), `Scheduler` (rate groups; throws if a rate does not divide the base rate), `Bus` (latest-value typed channels, `seq == 0` means never published), `Model` (discrete, `initialize(Bus&)` then `update(StepContext)`), `ContinuousSystem` (`prepare`, `derivatives`, `integrate`), `Integrator` (RK4), `Pacer`, `Random` (`sim::Rng`, `deriveSeed`), `CsvWriter`, `Profiling` (Tracy zones).
- `env/`: `Atmosphere` (US76 0 to 86 km), `Gravity` (uniform placeholder), `Weather` (wind profiles, seeded Dryden/von Karman turbulence, gusts), `Geodesy` (WGS-84 geodetic/ECEF/local ENU; `flight_sw` has its own copy by design, cross-checked in `tests/test_navigation.cpp`).
- Nothing vehicle-specific belongs here. Nothing here knows about networking, clients or flight software.
- `Pacer` is the only place that looks at wall-clock time.
- Channel publish must stay allocation-free; keep it that way when extending `Bus`.
- Rate plan for future models: `docs/architecture.md`. Planned frames and gravity work: build order items 2 and 3.
