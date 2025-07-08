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
# 编译sunray_planner模块
catkin_make --source General_Module/sunray_planner_utils --build build/sunray_planner_utils
#编译sunray_communication_bridge模块
catkin_make --source Comunication_Module/sunray_communication_bridge --build build/sunray_communication_bridge
# 编译sunray_media模块
# catkin_make --source General_Module/sunray_media --build build/sunray_media
## 注意：ego-planner与FUEL不能同时编译 会存在一些链接上的问题
# 编译ego-planner-swarm模块
catkin_make --source External_Module/ego-planner-swarm --build build/ego-planner
# 编译FUEL模块
# catkin_make --source External_Module/FUEL --build build/FUEL
# 编译simulator_utils模块
catkin_make --source Simulation/simulator_utils --build build/simulator_utils
# 编译gazebo_plugin模块
catkin_make --source Simulation/gazebo_plugin --build build/gazebo_plugin
# 编译sunray_formation模块
# catkin_make --source sunray_formation --build build/sunray_formation
# 编译2025_uav_competition_demo模块
catkin_make --source Simulation/2025_uav_competiton_demo --build build/2025_uav_competiton_demo
# 编译wind_zone_plugin模块
catkin_make --source Simulation/gazebo_plugin/wind_zone_plugin --build build/wind_zone_plugin

catkin_make --source Simulation/future_2025_score --build build/future_2025_score
# ugv需要octomap、serial相关以依赖
# 编译sunray_ugv_control模块
# catkin_make --source General_Module/sunray_ugv_control --build build/sunray_ugv_control
# 编译turn_on_wheeltec_robot模块
# catkin_make --source External_Module/turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
