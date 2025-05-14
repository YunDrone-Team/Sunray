#!/bin/bash
# 脚本：单个无人机EGO测试脚本（EGO地图输入来自三维激光雷达实时点云）

gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_uav_planning.launch rviz_enable:=false; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_ego.launch; exec bash"' \
