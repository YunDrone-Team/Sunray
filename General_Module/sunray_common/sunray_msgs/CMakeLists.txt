cmake_minimum_required(VERSION 2.8.3)
project(sunray_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
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
  DIRECTORY mavlink_msgs
  FILES
  Heartbeat.msg
  SysStatus.msg
  Attitude.msg
  AttitudeQuaternion.msg
  LocalPositionNED.msg
  AttitudeSetpoint.msg
  AttitudeTarget.msg
  GlobalPositionSetpoint.msg
  LocalPositionSetpoint.msg
  PositionTargetLocalNED.msg
  VisionPositionEstimate.msg
  RCChannels.msg
)

add_message_files(
  DIRECTORY msg
  FILES
  UAVControlCMD.msg
  UAVState.msg
  UAVSetup.msg
  TextInfo.msg
  ExternalOdom.msg
  SetMode.msg
  UGVControlCMD.msg
  ViobotState.msg
  PositionCommand.msg
  OrcaCmd.msg
  TargetMsg.msg
  TargetsInFrameMsg.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ArmCmd.srv
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



