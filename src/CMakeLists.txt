add_library(razer_hydra hydra.cpp)
add_dependencies(razer_hydra razer_hydra_gencpp)
target_link_libraries(razer_hydra usb yaml-cpp ${catkin_LIBRARIES})
