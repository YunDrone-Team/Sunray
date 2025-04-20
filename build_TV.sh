#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译vrpn_client_ros功能包
catkin_make --source External_Module/vrpn_client_ros --build build/vrpn_client_ros
# 编译sunray_simulator模块
catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
# 编译sunray_planner模块
catkin_make --source General_Module/sunray_planner --build build/sunray_planner
# 编译sunray_ground模块
catkin_make --source General_Module/sunray_ground --build build/sunray_ground
# 编译ego-planner-swarm模块
catkin_make --source External_Module/ego-planner-swarm --build build/ego-planner
# 编译simulator_utils模块
catkin_make --source Simulation/simulator_utils --build build/simulator_utils
# 编译turn_on_wheeltec_robot模块
catkin_make --source External_Module/turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot

# catkin_make --source sunray_msgs --build build/sunray_msgs
# catkin_make --source sunray_comm --build build/sunray_comm

# catkin_make --source turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
# catkin_make --source sunray_wheeltec_robot --build build/sunray_wheeltec_robot
# catkin_make --source sunray_ugv_planning --build build/sunray_ugv_planning