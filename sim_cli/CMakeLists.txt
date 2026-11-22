add_executable(sim_cli main.cpp)
target_link_libraries(sim_cli PRIVATE sim_vehicle)
sim_set_warnings(sim_cli)
