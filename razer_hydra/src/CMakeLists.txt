rosbuild_add_library(hydra hydra.cpp)
target_link_libraries(hydra usb yaml-cpp)
