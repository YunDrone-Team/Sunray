cmake_minimum_required(VERSION 3.0.2)
project(viobot_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosmsg
  system_ctrl
  std_msgs
)

catkin_package(

)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(viobot_demo
  src/viobot_ctrl.cpp
)
target_link_libraries(viobot_demo  ${catkin_LIBRARIES})




