cmake_minimum_required(VERSION 2.8.3)
project(sunray_media)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  cv_bridge
)

################################################
## Declare ROS messages, services and actions ##
################################################

catkin_package(
  INCLUDE_DIRS include
  DEPENDS OpenCV
  CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


###############################
##      声明可执行cpp文件     ##
###############################


add_executable(rtsp_push_node src/rtsp_push.cpp)
# add_dependencies(uav_command_pub sunray_control_gencpp)
target_link_libraries(rtsp_push_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
