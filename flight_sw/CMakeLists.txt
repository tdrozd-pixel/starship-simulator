# Flight software. Deliberately depends on nothing else in this repo (and no Eigen):
# it must be able to build for a real flight computer against a real HAL.
add_library(flight_sw STATIC
    src/Pid.cpp
    src/VerticalEstimator.cpp
    src/HorizontalEstimator.cpp
    src/Geodesy.cpp
    src/Ccsds.cpp
    src/GncTelemetry.cpp
    src/HopFlightComputer.cpp)

target_include_directories(flight_sw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
sim_set_warnings(flight_sw)
