cmake_minimum_required(VERSION 2.8.3)
project(detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_generation
  sensor_msgs
  actionlib_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  TargetMsg.msg
  TargetsInFrameMsg.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs 
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



