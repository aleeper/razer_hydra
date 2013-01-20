rosbuild_add_executable(print print.cpp)
target_link_libraries(print hydra)

rosbuild_add_executable(record record.cpp)
target_link_libraries(record hydra)
