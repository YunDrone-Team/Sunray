cmake_minimum_required(VERSION 2.8.3)
project(sunray_formation)

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
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
# opencv
find_package(OpenCV REQUIRED)
# sudo apt-get install libyaml-cpp-dev
find_package(yaml-cpp REQUIRED)
# sudo apt-get install libsfml-dev
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

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
  ${YAML_CPP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../General_Module/sunray_common/common_lib
)


###############################
##      声明可执行cpp文件     ##
###############################
add_library(uav_formation src/uav_formation.cpp)
add_executable(uav_formation_node src/uav_formation_node.cpp)
target_link_libraries(uav_formation_node uav_formation ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

add_library(traj_generator src/traj_generator.cpp)
add_executable(traj_generator_node src/traj_generator_node.cpp)
target_link_libraries(traj_generator_node traj_generator ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})

add_executable(formation_switch_node src/formation_switch.cpp)
target_link_libraries(formation_switch_node  ${catkin_LIBRARIES})

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