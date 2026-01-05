#!/bin/bash
# 脚本：单个无人机仿真（VINS + EGO Planner + 控制 + 规划）
# 说明：该脚本用于启动一个包含VINS定位、EGO规划和控制的单无人机仿真环境。
#      该脚本会在一个终端中启动多个标签页，每个标签页运行一个ROS launch文件。
# 使用方法：
#      1. 确保已经配置好ROS环境，并且所有相关的包已经编译。
#      2. 运行该脚本：bash launch_single_uav_vins_ego.sh
# 注意事项：
#      - 请根据电脑性能调整gazebo的gui参数，以确保仿真流畅运行。

# 启动ROS核心和仿真环境,注意下面四行之间不能有空行以及注释，这是由gnome-terminal的语法限制的
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_vins.launch gui:=false rviz_enable:=true; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch vins vins_start.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_planner_utils sunray_vins_ego.launch; exec bash"' \

# 启动控制节点
gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0 position_topic:=/vins_estimator/odometry; exec bash"' \



