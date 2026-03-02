cmake_minimum_required(VERSION 3.0.2)
project(sunray_viobot)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/
  ${CMAKE_CURRENT_SOURCE_DIR}/../../common/sunray_log/spdlog/include
)

add_executable(viobot_odom_node src/viobot_odom_node.cpp)
target_link_libraries(viobot_odom_node ${catkin_LIBRARIES})
