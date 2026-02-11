cmake_minimum_required(VERSION 3.0.2)
project(ekf_odometry)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    eigen_conversions
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# IMU Preintegration library
add_library(imu_preintegration
  src/imu_preintegration.cpp
)
target_link_libraries(imu_preintegration
  ${catkin_LIBRARIES}
)

# EKF library
add_library(ekf_filter
  src/ekf_filter.cpp
)
target_link_libraries(ekf_filter
  ${catkin_LIBRARIES}
)

# Main node executable
add_executable(ekf_odometry_node
  src/ekf_odometry_node.cpp
)
target_link_libraries(ekf_odometry_node
  imu_preintegration
  ekf_filter
  ${catkin_LIBRARIES}
)

install(TARGETS imu_preintegration ekf_filter ekf_odometry_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
