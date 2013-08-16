cmake_minimum_required(VERSION 2.8.3)
project(razer_hydra)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS geometry_msgs tf roscpp genmsg)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

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
    CATKIN_DEPENDS geometry_msgs tf
)
include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    )

link_directories(${catkin_LIBRARY_DIRS})


add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(nodes)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install udev rules
option(INSTALL_RAZER_HYDRA_UDEV_RULES "Install udev rules for Razer Hydra" ON)
IF (INSTALL_RAZER_HYDRA_UDEV_RULES)
  INSTALL (
    FILES config/99-hydra.rules
    DESTINATION "/etc/udev/rules.d/99-hydra.$ENV{ROS_DISTRO}.rules"
    COMPONENT "udev"
    )
else (INSTALL_RAZER_HYDRA_UDEV_RULES)
  message (STATUS "Udev rules for Razer Hydra not being installed, install them with -DINSTALL_RAZER_HYDRA_UDEV_RULES=ON")
endif (INSTALL_RAZER_HYDRA_UDEV_RULES)
