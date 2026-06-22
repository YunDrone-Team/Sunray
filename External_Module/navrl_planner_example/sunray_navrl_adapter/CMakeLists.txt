cmake_minimum_required(VERSION 3.0.2)
project(sunray_navrl_adapter)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  navigation_runner
  nav_msgs
  roscpp
  sunray_common
  sunray_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    navigation_runner
    nav_msgs
    roscpp
    sunray_common
    sunray_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(NavRL2Sunray_node
  src/NavRL2Sunray.cpp
)

add_dependencies(NavRL2Sunray_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(NavRL2Sunray_node
  ${catkin_LIBRARIES}
)

add_executable(NavRL2SunrayUGV_node
  src/NavRL2SunrayUGV.cpp
)

add_dependencies(NavRL2SunrayUGV_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(NavRL2SunrayUGV_node
  ${catkin_LIBRARIES}
)

add_executable(NavRLTerminalControl_node
  src/NavRLTerminalControl.cpp
)

add_dependencies(NavRLTerminalControl_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(NavRLTerminalControl_node
  ${catkin_LIBRARIES}
)
