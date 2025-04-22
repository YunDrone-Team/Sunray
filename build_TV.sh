#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译sunray_planner模块
catkin_make --source General_Module/sunray_planner --build build/sunray_planner
# 编译ego-planner-swarm模块
catkin_make --source External_Module/ego-planner-swarm --build build/ego-planner
# 编译turn_on_wheeltec_robot模块
catkin_make --source External_Module/turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
# 编译sunray_ugv_control模块
catkin_make --source General_Module/sunray_ugv_control --build build/sunray_ugv_control
# 编译sunray_TV模块
catkin_make --source General_Module/sunray_TV --build build/sunray_TV