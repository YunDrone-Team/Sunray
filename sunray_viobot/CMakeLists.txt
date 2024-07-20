cmake_minimum_required(VERSION 2.8.3)
project(sunray_viobot)

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
  system_ctrl
  actionlib_msgs
  loop_action
  visualization_msgs
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
  LIBRARIES sunray_viobot
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


###############################
##      声明可执行cpp文件     ##
###############################
add_library(sunray_viobot lib/sunray_viobot.cpp)

add_executable(sunray_viobot_node src/sunray_viobot_node.cpp)
add_dependencies(sunray_viobot_node sunray_viobot_gencpp)
target_link_libraries(sunray_viobot_node ${catkin_LIBRARIES})
target_link_libraries(sunray_viobot_node sunray_viobot)

add_executable(viobot_demo src/viobot_demo.cpp)
add_dependencies(viobot_demo sunray_viobot_gencpp)
target_link_libraries(viobot_demo ${catkin_LIBRARIES})

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