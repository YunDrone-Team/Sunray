cmake_minimum_required(VERSION 2.8.3)
project(sunray_map_generator)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../../General_Module/sunray_common/common_lib
)

add_library(map_generator src/map_generator.cpp)
add_executable (map_generator_node src/map_generator_node.cpp)
target_link_libraries(map_generator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  map_generator
)


add_executable(laser_sim_node src/laser_sim_node.cpp)
target_link_libraries(laser_sim_node ${catkin_LIBRARIES})
target_link_libraries(laser_sim_node ${PCL_LIBRARIES})
  
