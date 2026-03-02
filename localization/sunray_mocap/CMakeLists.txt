cmake_minimum_required(VERSION 3.0.2)
project(sunray_mocap)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_filters
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/spdlog/include
)

message(${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/)

add_executable(mocap_odom_node src/mocap_odom_node.cpp)
target_link_libraries(mocap_odom_node ${catkin_LIBRARIES})
