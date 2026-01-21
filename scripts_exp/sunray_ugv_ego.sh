#!/bin/bash
# 脚本：单个无人车EGO脚本（EGO地图输入来自Mid360激光雷达实时点云）
# 在RVIZ中使用 2D Nav Goal 设置目标点

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 1.0; roslaunch sunray_ugv_control wheeltec_robot.launch  ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1 location_source:=2 odom_topic:=/sunray/odometry enable_rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_ugv_control ugv_terminal_control.launch ugv_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 5.0; roslaunch sunray_planner_utils sunray_ego_ugv.launch; exec bash"' \
