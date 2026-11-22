add_executable(sim_tests
    TestMain.cpp
    test_core.cpp
    test_environment.cpp
    test_flight_software.cpp
    test_hop_scenario.cpp
    test_truth_snapshot.cpp)

target_include_directories(sim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sim_tests PRIVATE sim_vehicle)
if(TARGET sim_server_core)
    target_sources(sim_tests PRIVATE test_server.cpp)
    target_link_libraries(sim_tests PRIVATE sim_server_core)
    target_compile_definitions(sim_tests PRIVATE SIM_HAVE_SERVER=1)
endif()
sim_set_warnings(sim_tests)

add_test(NAME sim_tests COMMAND sim_tests)
