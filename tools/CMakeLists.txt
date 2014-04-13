set(PRINT_NODE ${PROJECT_NAME}_print)
add_executable(${PRINT_NODE} print.cpp)
add_dependencies(${PRINT_NODE} razer_hydra)
target_link_libraries(${PRINT_NODE} razer_hydra ${catkin_LIBRARIES})
set_target_properties(${PRINT_NODE} PROPERTIES OUTPUT_NAME print)

set(RECORD_NODE ${PROJECT_NAME}_record)
add_executable(${RECORD_NODE} record.cpp)
add_dependencies(${RECORD_NODE} razer_hydra)
target_link_libraries(${RECORD_NODE} razer_hydra ${catkin_LIBRARIES})
set_target_properties(${RECORD_NODE} PROPERTIES OUTPUT_NAME record)

install(TARGETS ${PRINT_NODE} ${RECORD_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
