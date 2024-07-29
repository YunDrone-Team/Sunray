#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译sunray_simulator模块
catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
# 编译sunray_ground模块
# catkin_make --source General_Module/sunray_ground --build build/sunray_ground
# 编译sunray_tutorial模块
catkin_make --source General_Module/sunray_tutorial --build build/sunray_tutorial
# 编译ego-planner-swarm模块
catkin_make --source ego-planner-swarm --build build/ego-planner
# 编译FUEL模块
catkin_make --source FUEL --build build/FUEL
# catkin_make --source orca_planenr --build build/orca_planenr

# catkin_make --source sunray_msgs --build build/sunray_msgs
# catkin_make --source sunray_comm --build build/sunray_comm

# catkin_make --source turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
# catkin_make --source sunray_wheeltec_robot --build build/sunray_wheeltec_robot
# catkin_make --source sunray_ugv_planning --build build/sunray_ugv_planning

