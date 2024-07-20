cmake_minimum_required(VERSION 2.8.3)
project(sunray_ugv_planning)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	laser_geometry
	std_msgs
	sensor_msgs
	geometry_msgs
	nav_msgs
	pcl_ros
	visualization_msgs
	sunray_msgs
	tf2
)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES 
	#  DEPENDS system_lib
)

include_directories(
	SYSTEM
	include
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${Eigen3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_executable(ugv_planing_node
	src/ugv_planing_node.cpp
	src/planning_fsm.cpp
	src/A_star.cpp
	src/occupy_map.cpp
)

target_link_libraries(ugv_planing_node
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
