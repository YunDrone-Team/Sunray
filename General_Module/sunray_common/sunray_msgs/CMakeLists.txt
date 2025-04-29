cmake_minimum_required(VERSION 2.8.3)
project(sunray_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
)

add_message_files(
  DIRECTORY msg
  FILES
  UAVControlCMD.msg
  UAVState.msg
  UAVSetup.msg
  PX4State.msg
  UAVWayPoint.msg
  RcState.msg
  TextInfo.msg
  ExternalOdom.msg
  SetMode.msg
  UGVControlCMD.msg
  ViobotState.msg
  PositionCommand.msg
  OrcaCmd.msg
  TargetMsg.msg
  TargetsInFrameMsg.msg
  UGVControlCMD.msg
  UGVState.msg
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
  message_runtime
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)



