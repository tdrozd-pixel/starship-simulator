add_library(sim_server_core STATIC
    src/WireFormat.cpp
    src/JsonMessages.cpp
    src/SimRunner.cpp
    src/TruthServer.cpp
    src/WsEndpoint.cpp
    src/TelemetryFrame.cpp
    src/NetUtil.cpp)

target_include_directories(sim_server_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim_server_core PUBLIC sim_vehicle PRIVATE civetweb)
if(WIN32)
    target_link_libraries(sim_server_core PRIVATE ws2_32 iphlpapi)
endif()
sim_set_warnings(sim_server_core)

add_executable(sim_server main.cpp)
target_link_libraries(sim_server PRIVATE sim_server_core)
target_compile_definitions(sim_server PRIVATE SIM_DEFAULT_WEB_ROOT="${CMAKE_SOURCE_DIR}/web")
sim_set_warnings(sim_server)
