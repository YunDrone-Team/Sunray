cmake_minimum_required(VERSION 2.8.3)
project(sunray_tutorial)

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
  tf2_ros
  tf2_eigen
  sunray_msgs
  # detection_msgs
  tf2_geometry_msgs
  mavros_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

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
  # detection_msgs

)

catkin_package(
  CATKIN_DEPENDS  message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
  ${OpenCV_INCLUDE_DIRS}
)


###############################
##      声明可执行cpp文件     ##
###############################

add_executable(takeoff_hover_land uav_basic/takeoff_hover_land.cpp)
add_dependencies(takeoff_hover_land sunray_tutorial_gencpp)
target_link_libraries(takeoff_hover_land ${catkin_LIBRARIES})

add_executable(block_xyzpos uav_basic/block_xyzpos.cpp)
add_dependencies(block_xyzpos sunray_tutorial_gencpp)
target_link_libraries(block_xyzpos ${catkin_LIBRARIES})

add_executable(circle_xyvelzpos uav_basic/circle_xyvelzpos.cpp)
add_dependencies(circle_xyvelzpos sunray_tutorial_gencpp)
target_link_libraries(circle_xyvelzpos ${catkin_LIBRARIES})

add_executable(circle_xyzvel uav_basic/circle_xyzvel.cpp)
add_dependencies(circle_xyzvel sunray_tutorial_gencpp)
target_link_libraries(circle_xyzvel ${catkin_LIBRARIES})

add_executable(hexagon_xyzposyawbody uav_basic/hexagon_xyzposyawbody.cpp)
add_dependencies(hexagon_xyzposyawbody sunray_tutorial_gencpp)
target_link_libraries(hexagon_xyzposyawbody ${catkin_LIBRARIES})

add_executable(follow_car_xyvelzposyawbody uav_basic/follow_car_xyvelzposyawbody.cpp)
add_dependencies(follow_car_xyvelzposyawbody sunray_tutorial_gencpp)
target_link_libraries(follow_car_xyvelzposyawbody ${catkin_LIBRARIES})

add_executable(follow_tag_xyzvelyawbody uav_basic/follow_tag_xyzvelyawbody.cpp)
add_dependencies(follow_tag_xyzvelyawbody sunray_tutorial_gencpp)
target_link_libraries(follow_tag_xyzvelyawbody ${catkin_LIBRARIES})

add_executable(ugv_block_pos ugv_basic/ugv_block_pos.cpp)
add_dependencies(ugv_block_pos sunray_tutorial_gencpp)
target_link_libraries(ugv_block_pos ${catkin_LIBRARIES})

add_executable(ugv_circle_vel ugv_basic/ugv_circle_vel.cpp)
add_dependencies(ugv_circle_vel sunray_tutorial_gencpp)
target_link_libraries(ugv_circle_vel ${catkin_LIBRARIES})

add_executable(ugv_circle_vel_body ugv_basic/ugv_circle_vel_body.cpp)
add_dependencies(ugv_circle_vel_body sunray_tutorial_gencpp)
target_link_libraries(ugv_circle_vel_body ${catkin_LIBRARIES})

add_executable(auto_land advanced/auto_land.cpp)
add_dependencies(auto_land sunray_tutorial_gencpp)
target_link_libraries(auto_land ${catkin_LIBRARIES})

add_executable(auto_land_by_pose advanced/auto_land_by_pose.cpp)
add_dependencies(auto_land_by_pose sunray_tutorial_gencpp)
target_link_libraries(auto_land_by_pose ${catkin_LIBRARIES})

# add_executable(land_on_a_moving_car advanced/land_on_a_moving_car.cpp)
# add_dependencies(land_on_a_moving_car sunray_tutorial_gencpp detection_msgs_gencpp)
# target_link_libraries(land_on_a_moving_car ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(simple_obstacle_avoidance advanced/simple_obstacle_avoidance.cpp)
add_dependencies(simple_obstacle_avoidance sunray_tutorial_gencpp)
target_link_libraries(simple_obstacle_avoidance ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


catkin_install_python(PROGRAMS
scripts/key_move_car.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  uav_python/takeoff_hover_land.py
  uav_python/block_pos.py
  uav_python/circle_vel.py
  uav_python/circle_z_pos.py
  uav_python/pos_body_hexagon.py
  ugv_python/ugv_block_pos.py
  ugv_python/ugv_circle_vel.py
  ugv_python/ugv_circle_vel_body.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)