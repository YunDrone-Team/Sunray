cmake_minimum_required(VERSION 3.0.2)
project(orca_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  sunray_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(RVO_LIBRARY RVO)

set(RVO_HEADERS
  include/RVO/Line.h
  include/RVO/RVO.h
  include/RVO/RVOSimulator.h
  include/RVO/Vector2.h
)

set(RVO_SOURCES
  include/RVO/Agent.cc
  include/RVO/Agent.h
  include/RVO/KdTree.cc
  include/RVO/KdTree.h
  include/RVO/Line.cc
  include/RVO/Obstacle.cc
  include/RVO/Obstacle.h
  include/RVO/RVOSimulator.cc
  include/RVO/Vector2.cc
)

add_library(${RVO_LIBRARY} ${RVO_HEADERS} ${RVO_SOURCES})

add_library(${PROJECT_NAME}
  src/orca_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${RVO_LIBRARY}
)

add_executable(orca_test_node src/orca_test_node.cpp)
# add_dependencies(orca_test_node orca_test_node)
target_link_libraries(orca_test_node ${catkin_LIBRARIES})
target_link_libraries(orca_test_node ${PROJECT_NAME})