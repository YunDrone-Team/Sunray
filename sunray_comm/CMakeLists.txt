cmake_minimum_required(VERSION 2.8.3)
project(sunray_comm)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  sunray_msgs
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
  CATKIN_DEPENDS  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include_mavlink
  ${catkin_INCLUDE_DIRS}
)

###############################
##      声明可执行cpp文件     ##
###############################
add_library(serial_port src/lib/serial_port.cpp)
add_library(udp_port src/lib/udp_port.cpp)

###### Main File ##########

add_executable(mavlink_node src/mavlink_node.cpp)
add_dependencies(mavlink_node sunray_ros_gencpp)
target_link_libraries(mavlink_node ${catkin_LIBRARIES})
target_link_libraries(mavlink_node serial_port)
target_link_libraries(mavlink_node udp_port)

add_executable(fake_topic_pub_timer src/fake_topic_pub_timer.cpp)
add_dependencies(fake_topic_pub_timer sunray_ros_gencpp)
target_link_libraries(fake_topic_pub_timer ${catkin_LIBRARIES})

add_executable(fake_topic_pub src/fake_topic_pub.cpp)
add_dependencies(fake_topic_pub sunray_ros_gencpp)
target_link_libraries(fake_topic_pub ${catkin_LIBRARIES})

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
