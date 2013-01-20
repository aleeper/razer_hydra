add_executable(hydra_node hydra_node.cpp)
add_dependencies(hydra_node razer_hydra)
target_link_libraries(hydra_node razer_hydra ${catkin_LIBRARIES})
