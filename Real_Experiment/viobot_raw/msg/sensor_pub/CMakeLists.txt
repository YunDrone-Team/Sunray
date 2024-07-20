cmake_minimum_required(VERSION 3.0.2)
project(sensor_pub)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  ImageInfo.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_connect
 CATKIN_DEPENDS roscpp rosmsg message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${catkin_LIB_DIRS}
)
