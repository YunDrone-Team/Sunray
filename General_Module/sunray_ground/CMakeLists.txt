cmake_minimum_required(VERSION 2.8.3)
project(sunray_ground)

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

add_library(${PROJECT_NAME}
  src/ground_control.cpp
)

add_executable(ground_control_node src/ground_control_node.cpp)
# add_dependencies(ground_control_node sunray_control_gencpp)
target_link_libraries(ground_control_node ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_executable(send_test src/send_test.cpp)
# target_link_libraries(send_test ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_executable(recv_test src/recv_test.cpp)
# target_link_libraries(recv_test ${catkin_LIBRARIES} ${PROJECT_NAME})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)