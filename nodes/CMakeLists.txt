add_executable(hydra_node hydra_node.cpp)
add_dependencies(hydra_node razer_hydra)
target_link_libraries(hydra_node razer_hydra ${catkin_LIBRARIES})

install(TARGETS hydra_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
