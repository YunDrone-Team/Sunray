cmake_minimum_required(VERSION 2.8.3)
project(sunray_planner_utils)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  message_generation      
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  gazebo_msgs
  sunray_msgs
  pcl_conversions
  pcl_ros
  laser_geometry
  tf2_eigen
  # livox_ros_driver2
)

# 判断是否有livox_ros_driver2包，如果有则添加
find_package(livox_ros_driver2)
if(livox_ros_driver2_FOUND)
  message(STATUS "Found livox_ros_driver2")
else()
  message(STATUS "livox_ros_driver2 not found")
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime laser_geometry roscpp std_msgs tf
)

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../sunray_common/common_lib
)
###############################
##      声明可执行cpp文件     ##
###############################

add_executable(point_cloud_transform src/point_cloud_transform.cpp)
add_dependencies(point_cloud_transform ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_transform ${catkin_LIBRARIES})

add_executable(scan2point src/scan2Point.cpp)
add_dependencies(scan2point ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan2point ${catkin_LIBRARIES})

add_executable(positionCmd2sunray src/positionCmd2sunray.cpp)
add_dependencies(positionCmd2sunray ${catkin_EXPORTED_TARGETS})
target_link_libraries(positionCmd2sunray ${catkin_LIBRARIES})

# sudo apt-get install libyaml-cpp-dev
add_executable(goal2swarm src/goal2swarm.cpp)
add_dependencies(goal2swarm ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal2swarm ${catkin_LIBRARIES} yaml-cpp)

if(livox_ros_driver2_FOUND)
  add_executable(livox2Point src/livox2Point.cpp)
  add_dependencies(livox2Point ${catkin_EXPORTED_TARGETS})
  target_link_libraries(livox2Point ${catkin_LIBRARIES})
endif()

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)