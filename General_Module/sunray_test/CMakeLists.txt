cmake_minimum_required(VERSION 3.0.2)
project(sunray_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  sunray_msgs
  # serial
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# GeographicLib
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../sunray_common_uav_control/utils
  ${PROJECT_SOURCE_DIR}/../uav_control/pos_controller
  ${PROJECT_SOURCE_DIR}/../externalFusion
  ${PROJECT_SOURCE_DIR}/../mavlink
  ${PROJECT_SOURCE_DIR}/../mavlink/include
  ${PROJECT_SOURCE_DIR}/../mavlink/include/mavlink/include/mavlink/v2.0/c_library_v2/
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
  ${GeographicLib_INCLUDE_DIRS}
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  sunray_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/test.py
  scripts/report_output.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

