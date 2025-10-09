cmake_minimum_required(VERSION 2.8.3)
project(sunray_uav_control)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  mavros_msgs
  tf2_ros
  tf2_eigen
  sunray_msgs
  tf2_geometry_msgs
  # serial
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  uav_control/pos_controller
  externalFusion
  mavlink
  mavlink/include
  mavlink/include/mavlink/include/mavlink/v2.0/c_library_v2/
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
)

###############################
##      声明可执行cpp文件     ##
###############################
## 无人机控制
add_library(UAVControl uav_control/UAVControl.cpp)
add_executable(uav_control_node uav_control/uav_control_node.cpp)
add_dependencies(uav_control_node sunray_control_gencpp)
target_link_libraries(uav_control_node ${catkin_LIBRARIES})
target_link_libraries(uav_control_node UAVControl)

## rviz 2D Nav Goal桥接节点
add_library(RvizGoalBridge uav_control/RvizGoalBridge.cpp)
add_executable(rviz_goal_bridge_node uav_control/rviz_goal_bridge_node.cpp)
add_dependencies(rviz_goal_bridge_node sunray_control_gencpp)
target_link_libraries(rviz_goal_bridge_node ${catkin_LIBRARIES})
target_link_libraries(rviz_goal_bridge_node RvizGoalBridge)

## 外部定位
add_library(externalFusion externalFusion/externalFusion.cpp)
target_link_libraries(externalFusion ${catkin_LIBRARIES})

add_executable(external_fusion_node 
  externalFusion/external_fusion_node.cpp
  mavlink/mavlink_control.cpp
  mavlink/autopilot_interface.cpp
  mavlink/serial_port.cpp)
add_dependencies(external_fusion_node sunray_control_gencpp)
target_link_libraries(external_fusion_node ${catkin_LIBRARIES})
target_link_libraries(external_fusion_node externalFusion)

## 遥控器输入模块
add_executable(rc_input_node rc_input/rc_input_node.cpp)
target_link_libraries(rc_input_node ${catkin_LIBRARIES})

add_executable(joy_node rc_input/joy_node.cpp)
target_link_libraries(joy_node ${catkin_LIBRARIES})

## 航点任务模块
add_executable(waypoint_mission_node waypoint/waypoint_mission_node.cpp)
add_dependencies(waypoint_mission_node sunray_control_gencpp)
target_link_libraries(waypoint_mission_node ${catkin_LIBRARIES})

add_executable(waypoint_pub waypoint/waypoint_pub.cpp)
add_dependencies(waypoint_pub sunray_control_gencpp)
target_link_libraries(waypoint_pub ${catkin_LIBRARIES})

## 其他
add_executable(control_evaluation_node utils/control_evaluation_node.cpp)
add_dependencies(control_evaluation_node sunray_control_gencpp)
target_link_libraries(control_evaluation_node ${catkin_LIBRARIES})

add_executable(terminal_control utils/terminal_control.cpp)
add_dependencies(terminal_control sunray_control_gencpp)
target_link_libraries(terminal_control ${catkin_LIBRARIES})



add_executable(positioning_accuracy utils/positioning_accuracy.cpp)
add_dependencies(positioning_accuracy sunray_control_gencpp)
target_link_libraries(positioning_accuracy ${catkin_LIBRARIES})

add_executable(sunray_logger_test utils/sunray_logger_test.cpp)
add_dependencies(sunray_logger_test sunray_control_gencpp)
target_link_libraries(sunray_logger_test ${catkin_LIBRARIES})

# add_executable(uav_control_mode_pub utils/uav_control_mode_pub.cpp)
# add_dependencies(uav_control_mode_pub sunray_control_gencpp)
# target_link_libraries(uav_control_mode_pub ${catkin_LIBRARIES})

# add_executable(show_detail_node utils/show_detail.cpp)
# add_dependencies(show_detail_node sunray_control_gencpp)
# target_link_libraries(show_detail_node ${catkin_LIBRARIES})

# add_executable(logTest test/logTest.cpp)
# target_link_libraries(logTest ${catkin_LIBRARIES})



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)