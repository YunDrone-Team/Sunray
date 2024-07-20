cmake_minimum_required(VERSION 2.8.3)
project(sunray_wheeltec_robot)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
  sunray_msgs
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


###############################
##      声明可执行cpp文件     ##
###############################
add_executable(wheeltec_control_node src/wheeltec_control_node.cpp)
add_dependencies(wheeltec_control_node sunray_wheeltec_robot_gencpp)
target_link_libraries(wheeltec_control_node ${catkin_LIBRARIES})


add_executable(ugv_terminal_control src/ugv_terminal_control.cpp)
add_dependencies(ugv_terminal_control sunray_wheeltec_robot_gencpp)
target_link_libraries(ugv_terminal_control ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
