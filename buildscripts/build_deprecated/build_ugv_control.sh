#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_ugv_control模块
catkin_make --source General_Module/sunray_ugv_control --build build/sunray_ugv_control
# 编译turn_on_wheeltec_robot模块
catkin_make --source External_Module/turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
