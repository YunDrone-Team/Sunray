#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_simulator模块
catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
# 编译gazebo_plugin模块
catkin_make --source Simulation/gazebo_plugin --build build/gazebo_plugin
# 编译simulator_utils模块
catkin_make --source Simulation/simulator_utils --build build/simulator_utils
#编译sunray_communication_bridge模块
catkin_make --source Comunication_Module/sunray_communication_bridge --build build/sunray_communication_bridge
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译sunray_tutorial模块
catkin_make --source General_Module/sunray_tutorial --build build/sunray_tutorial