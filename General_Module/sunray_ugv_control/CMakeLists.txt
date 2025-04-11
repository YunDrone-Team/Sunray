cmake_minimum_required(VERSION 3.0.2)
project(sunray_ugv_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


set(octomap_msgs_DIR /home/yundrone/app/octomap_msgs/build/devel/share/octomap_msgs/cmake/octomap_msgsConfig.cmake)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_msgs
  laser_geometry
  nav_msgs
  pcl_ros
  sunray_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ugv_planning
#  CATKIN_DEPENDS octomap roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../sunray_common/common_lib
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# add_library(astar_lib src/Astar.cpp)
# add_library(map_generator_lib src/map_generator.cpp)
add_library(ugv_control src/ugv_control.cpp)

add_executable(map_generator src/map_generator.cpp)
target_link_libraries(map_generator 
                        ${catkin_LIBRARIES}
                        ${OCTOMAP_LIBRARIES}
                        ${PCL_LIBRARIES}
                        # astar_lib
                        )

add_executable(astar src/Astar.cpp)
target_link_libraries(astar ${catkin_LIBRARIES})    

add_executable(ugv_terminal_control src/ugv_terminal_control.cpp)
target_link_libraries(ugv_terminal_control
    ${catkin_LIBRARIES}
    octomap
    octomath
)

add_executable(ugv_control_node src/ugv_control_node.cpp)
target_link_libraries(ugv_control_node 
                        ${catkin_LIBRARIES}
                        ${OCTOMAP_LIBRARIES}
                        ${PCL_LIBRARIES}
                        ugv_control
                        # astar_lib
                        # map_generator_lib
                        )

# add_executable(ugv_control src/ugv_control.cpp)
# target_link_libraries(ugv_control 
#                         ${catkin_LIBRARIES}
#                         ${OCTOMAP_LIBRARIES}
#                         ${PCL_LIBRARIES}
#                         )                        