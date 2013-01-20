add_library(razer_hydra hydra.cpp)
target_link_libraries(razer_hydra usb yaml-cpp ${catkin_LIBRARIES})
