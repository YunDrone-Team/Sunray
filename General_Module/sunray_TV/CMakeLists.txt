cmake_minimum_required(VERSION 2.8.3)
project(sunray_TV)

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
  tf2_ros
  tf2_eigen
  sunray_msgs
  tf2_geometry_msgs
  mavros_msgs

  message_filters
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

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
  CATKIN_DEPENDS  message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


###############################
##      声明可执行cpp文件     ##
###############################

add_executable(auto_waypoint src/auto_waypoint.cpp)
target_link_libraries(auto_waypoint ${catkin_LIBRARIES} yaml-cpp)

add_executable(viobot_pointCloud  src/viobot.cpp)
target_link_libraries(viobot_pointCloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# catkin_install_python(PROGRAMS
# scripts/key_move_car.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
