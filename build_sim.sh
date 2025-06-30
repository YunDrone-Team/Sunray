#!/bin/bash

# 编译sunray_common模块
catkin_make --source General_Module/sunray_common --build build/sunray_common
# 编译sunray_uav_control模块
catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
# 编译vrpn_client_ros功能包
catkin_make --source External_Module/vrpn_client_ros --build build/vrpn_client_ros
# 编译sunray_tutorial模块
catkin_make --source General_Module/sunray_tutorial --build build/sunray_tutorial
# 编译sunray_simulator模块
catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
# 编译simulator_utils模块
catkin_make --source Simulation/simulator_utils --build build/simulator_utils
# 编译gazebo_plugin模块
catkin_make --source Simulation/gazebo_plugin --build build/gazebo_plugin
# 编译gazebo_plugin模块
catkin_make --source General_Module/mavlink --build build/mavlink