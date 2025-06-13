#!/bin/bash
# 脚本：单个无人机EGO测试脚本（EGO地图输入来自三维激光雷达实时点云）

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch ; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_planner_utils sunray_ego_single_mid360.launch; exec bash"' \