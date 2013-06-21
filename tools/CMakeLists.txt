add_executable(print print.cpp)
add_dependencies(print razer_hydra)
target_link_libraries(print razer_hydra ${catkin_LIBRARIES})

add_executable(record record.cpp)
add_dependencies(record razer_hydra)
target_link_libraries(record razer_hydra ${catkin_LIBRARIES})
