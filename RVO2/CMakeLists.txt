cmake_minimum_required(VERSION 2.8.3)
project(RVO2)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  mavros_msgs
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
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
)


###############################
##      声明可执行cpp文件     ##
###############################

add_library(changing_formation src/changing_formation.cpp)
add_library(RVO2 include/RVOSimulator.cc include/Agent.cc include/Export.cc include/KdTree.cc include/Obstacle.cc include/Vector2.cc include/Line.cc)
add_executable(changing_formation_node src/changing_formation_node.cpp)
target_link_libraries(changing_formation_node ${catkin_LIBRARIES})
target_link_libraries(changing_formation_node changing_formation)
target_link_libraries(changing_formation_node RVO2)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)