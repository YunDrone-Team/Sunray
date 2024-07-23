#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译sunray_simulator模块
# catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
# 编译sunray_ground模块
# catkin_make --source General_Module/sunray_ground --build build/sunray_ground
# 编译sunray_tutorial模块
catkin_make --source General_Module/sunray_tutorial --build build/sunray_tutorial
