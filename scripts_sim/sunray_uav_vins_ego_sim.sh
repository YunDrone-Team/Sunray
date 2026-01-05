#!/bin/bash
# 脚本：单个无人机仿真（VINS + EGO Planner + 控制 + 规划）
# 说明：该脚本用于启动一个包含VINS定位、EGO规划和控制的单无人机仿真环境。
#      该脚本会在一个终端中启动多个标签页，每个标签页运行一个ROS launch文件。
# 使用方法：
#      1. 确保已经配置好ROS环境，并且所有相关的包已经编译。
#      2. 运行该脚本：bash launch_single_uav_vins_ego.sh
# 注意事项：
#      - 请根据电脑性能调整gazebo的gui参数，以确保仿真流畅运行。


# 启动ros节点
gnome-terminal --window -- bash -c "roscore; exec bash"

# 启动一个无人机仿真环境，启动控制节点
gnome-terminal --window -- bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_vins.launch gui:=false enable_control:=true rviz_enable:=true; exec bash"

# 启动vins 
gnome-terminal --window -- bash -c "sleep 2.0; roslaunch vins vins_start.launch; exec bash"

# 启动ego
gnome-terminal --window -- bash -c "sleep 5.0; roslaunch ego_planner single_run_in_exp.launch; exec bash"
