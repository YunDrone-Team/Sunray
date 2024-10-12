cmake_minimum_required(VERSION 2.8.3)
project(sunray_tutorial)

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
  CATKIN_DEPENDS  message_runtime
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
add_executable(follow_a_car src/follow_a_car.cpp)
add_dependencies(follow_a_car sunray_tutorial_gencpp)
target_link_libraries(follow_a_car ${catkin_LIBRARIES})

add_executable(follow_a_tag src/follow_a_tag.cpp)
add_dependencies(follow_a_tag sunray_tutorial_gencpp)
target_link_libraries(follow_a_tag ${catkin_LIBRARIES})

add_executable(takeoff_hover_land src/takeoff_hover_land.cpp)
add_dependencies(takeoff_hover_land sunray_tutorial_gencpp)
target_link_libraries(takeoff_hover_land ${catkin_LIBRARIES})

add_executable(block_pos src/block_pos.cpp)
add_dependencies(block_pos sunray_tutorial_gencpp)
target_link_libraries(block_pos ${catkin_LIBRARIES})

add_executable(circle_z_pos src/circle_z_pos.cpp)
add_dependencies(circle_z_pos sunray_tutorial_gencpp)
target_link_libraries(circle_z_pos ${catkin_LIBRARIES})

add_executable(circle_vel src/circle_vel.cpp)
add_dependencies(circle_vel sunray_tutorial_gencpp)
target_link_libraries(circle_vel ${catkin_LIBRARIES})

add_executable(pos_body_hexagon src/pos_body_hexagon.cpp)
add_dependencies(pos_body_hexagon sunray_tutorial_gencpp)
target_link_libraries(pos_body_hexagon ${catkin_LIBRARIES})

add_executable(vel_body_follow_car src/vel_body_follow_car.cpp)
add_dependencies(vel_body_follow_car sunray_tutorial_gencpp)
target_link_libraries(vel_body_follow_car ${catkin_LIBRARIES})

add_executable(vel_body_z_pos_follow_car src/vel_body_z_pos_follow_car.cpp)
add_dependencies(vel_body_z_pos_follow_car sunray_tutorial_gencpp)
target_link_libraries(vel_body_z_pos_follow_car ${catkin_LIBRARIES})

add_executable(auto_land src/auto_land.cpp)
add_dependencies(auto_land sunray_tutorial_gencpp)
target_link_libraries(auto_land ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
scripts/key_move_car.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)