# tests

C++ unit and scenario tests in one executable, `sim_tests`, using the in-house harness in `TestFramework.hpp` (`TEST_CASE(name)`; swap for Catch2 or GoogleTest when it gets limiting).

- Run: `build/tests/Release/sim_tests.exe [filter]` (filter matches a substring of the test name) or `ctest --test-dir build -C Release`.
- One file per area: `test_core`, `test_environment`, `test_flight_software`, `test_hop_scenario`, `test_navigation`, `test_telemetry`, `test_truth_snapshot`, `test_server` (only built when `SIM_BUILD_SERVER` is on). Add new `.cpp` files to `tests/CMakeLists.txt`.
- Changes that should not alter flight results need a test that proves it (same seed, same outcome). `telemetry_stream_matches_golden` pins the exact downlinked bytes of the first 0.9 s (on the pad) of the seed-1 hop; update its constants only when the wire format is meant to change.
- Do not write tests of flight performance or tuning (landing accuracy, touchdown speed, gains): run the simulation and look at the results. Tests cover physics models, interfaces, determinism and wire formats. The golden hashes only the on-pad part of the flight so guidance tuning cannot break it.
- All tests must pass before any commit.
