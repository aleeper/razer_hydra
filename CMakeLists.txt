cmake_minimum_required(VERSION 2.8.3)
project(razer_hydra)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf roscpp genmsg)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_message_files(
  FILES
  Hydra.msg
  HydraPaddle.msg
  HydraRaw.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES razer_hydra 
    CATKIN_DEPENDS geometry_msgs tf
)
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    )

link_directories(${catkin_LIBRARY_DIRS})


add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(nodes)

install(DIRECTORY include/
  DESTINATION include)

#install(FILES
#  plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

