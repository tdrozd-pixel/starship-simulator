add_library(sim_vehicle STATIC
    src/HopperDynamics.cpp
    src/Sensors.cpp
    src/Downlink.cpp
    src/FlightSoftwareModel.cpp
    src/TelemetryLogger.cpp
    src/Tower.cpp
    src/TruthSnapshotPublisher.cpp
    src/HopScenario.cpp)

target_include_directories(sim_vehicle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_vehicle PUBLIC sim_core flight_sw)
sim_set_warnings(sim_vehicle)
