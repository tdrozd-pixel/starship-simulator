# tests

C++ unit and scenario tests in one executable, `sim_tests`, using the in-house harness in `TestFramework.hpp` (`TEST_CASE(name)`; swap for Catch2 or GoogleTest when it gets limiting).

- Run: `build/tests/Release/sim_tests.exe [filter]` (filter matches a substring of the test name) or `ctest --test-dir build -C Release`.
- One file per area: `test_core`, `test_environment`, `test_flight_software`, `test_hop_scenario`, `test_navigation`, `test_truth_snapshot`, `test_server` (only built when `SIM_BUILD_SERVER` is on). Add new `.cpp` files to `tests/CMakeLists.txt`.
- Changes that should not alter flight results need a test that proves it (same seed, same outcome).
- All tests must pass before any commit.
