cmake_minimum_required(VERSION 2.8.3)

project(sunray_fmt_control)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf
  sunray_msgs
  mavros_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
  ${PROJECT_SOURCE_DIR}/../sunray_uav_control/uav_control/pos_controller
  ${PROJECT_SOURCE_DIR}/../sunray_uav_control/include
)


add_executable(vrpn_to_vision_pose test/vrpn_to_vision_pose.cpp)
target_link_libraries(vrpn_to_vision_pose ${catkin_LIBRARIES})
add_dependencies(vrpn_to_vision_pose ${catkin_EXPORTED_TARGETS})

add_executable(fmt_topic_test test/fmt_topic_test.cpp)
target_link_libraries(fmt_topic_test ${catkin_LIBRARIES})

add_executable(fmt_auto_mission test/fmt_auto_mission.cpp)
target_link_libraries(fmt_auto_mission ${catkin_LIBRARIES})

add_library(fmt_control src/fmt_control.cpp)
add_executable(fmt_control_node src/fmt_control_node.cpp)
add_dependencies(fmt_control_node sunray_control_gencpp)
target_link_libraries(fmt_control_node ${catkin_LIBRARIES})
target_link_libraries(fmt_control_node fmt_control)

add_library(fmt_externalFusion src/fmt_externalFusion.cpp)
target_link_libraries(fmt_externalFusion ${catkin_LIBRARIES})

add_executable(fmt_external_fusion_node src/fmt_external_fusion_node.cpp)
add_dependencies(fmt_external_fusion_node sunray_control_gencpp)
target_link_libraries(fmt_external_fusion_node ${catkin_LIBRARIES})
target_link_libraries(fmt_external_fusion_node fmt_externalFusion)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)