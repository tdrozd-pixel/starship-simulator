cmake_minimum_required(VERSION 3.21)

project(StarshipSimulator
    VERSION 0.1.0
    DESCRIPTION "Engineering-grade Starship launch, entry and landing simulator"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(SIM_ENABLE_TRACY "Build with the Tracy profiler client" OFF)
option(SIM_BUILD_TESTS "Build unit and scenario tests" ON)

include(cmake/Dependencies.cmake)
include(cmake/CompilerWarnings.cmake)

# Layering (lower layers never include higher ones):
#   flight_sw   : flight software, depends on nothing in this repo
#   sim_core    : simulation framework and environment models
#   sim_vehicle : vehicle models, sensors and the flight software bridge
#   sim_cli     : headless runner
add_subdirectory(flight_sw)
add_subdirectory(sim_core)
add_subdirectory(sim_vehicle)
add_subdirectory(sim_cli)

if(SIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
