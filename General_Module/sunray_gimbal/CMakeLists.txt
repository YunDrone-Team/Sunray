cmake_minimum_required(VERSION 3.0.2)
project(sunray_gimbal)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  roscpp
  sensor_msgs
  std_msgs
  sunray_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GimbalParams.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GimbalStatus.srv
  GimbalReboot.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs#   std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  test/test_set_angles.py
  test/test_get_rate.py
  scripts/gimbal_control.py
  scripts/decompress_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

add_executable(gimbal_target_lock_cpp demo/gimbal_target_lock.cpp)
target_link_libraries(gimbal_target_lock_cpp ${catkin_LIBRARIES})

add_executable(gimbal_scan_keyboard demo/gimbal_scan_keyboard.cpp)
target_link_libraries(gimbal_scan_keyboard ${catkin_LIBRARIES})

add_executable(gimbal_angle_photo demo/gimbal_angle_photo.cpp)
target_link_libraries(gimbal_angle_photo ${catkin_LIBRARIES})

add_executable(gimbal_capture_switch demo/gimbal_capture_switch.cpp)
target_link_libraries(gimbal_capture_switch ${catkin_LIBRARIES})

add_executable(gimbal_qrtracker demo/gimbal_qrtracker.cpp)
target_link_libraries(gimbal_qrtracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
  yaml-cpp)

add_executable(image_saver demo/image_saver.cpp)
target_link_libraries(image_saver ${catkin_LIBRARIES})