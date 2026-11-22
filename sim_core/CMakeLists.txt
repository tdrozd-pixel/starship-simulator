add_library(sim_core STATIC
    src/Scheduler.cpp
    src/Simulation.cpp
    src/Pacer.cpp
    src/CsvWriter.cpp
    src/Atmosphere.cpp
    src/Weather.cpp
    src/Geodesy.cpp)

target_include_directories(sim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_core PUBLIC Eigen3::Eigen)
sim_set_warnings(sim_core)

if(SIM_ENABLE_TRACY)
    target_link_libraries(sim_core PUBLIC Tracy::TracyClient)
    target_compile_definitions(sim_core PUBLIC SIM_ENABLE_TRACY)
endif()

if(WIN32)
    # timeBeginPeriod for 1 ms sleep resolution in the real-time pacer
    target_link_libraries(sim_core PRIVATE winmm)
endif()
