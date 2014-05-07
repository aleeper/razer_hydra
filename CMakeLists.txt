cmake_minimum_required(VERSION 2.8.3)
project(razer_hydra)

find_package(catkin REQUIRED COMPONENTS
             std_msgs geometry_msgs message_generation
             genmsg roscpp tf)

find_package(PkgConfig)
pkg_check_modules(PC_LIBUSB REQUIRED libusb-1.0)

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
    CATKIN_DEPENDS std_msgs geometry_msgs tf message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})



add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(nodes)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install udev rules
option(${PROJECT_NAME}_INSTALL_UDEV_RULES "Install udev rules for Razer Hydra" ON)
if (${PROJECT_NAME}_INSTALL_UDEV_RULES)
  install (
    FILES config/99-hydra-indigo.rules
    DESTINATION "/etc/udev/rules.d"
    COMPONENT "udev")
else ()
  message (STATUS "Udev rules for Razer Hydra not being installed, install them with -DINSTALL_RAZER_HYDRA_UDEV_RULES=ON")
endif ()
