cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
#   livox_ros_driver
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})


#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-11/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation 
        # sunray_ros_p3d
        CATKIN_DEPENDS tf
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)

# add_library(sunray_ros_p3d src/gazebo_ros_p3d.cpp)
# target_link_libraries(sunray_ros_p3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(livox_laser_simulation libprotobuf.so.9)