#!/bin/bash
# 脚本：单个无人车EGO测试脚本（EGO地图输入来自Mid360激光雷达实时点云）
# 使用方法: ./sunray_ugv_ego_sim.sh
# 在RVIZ中使用 2D Nav Goal 设置目标点

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_ugv_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_ugv_control ugv_control_sim.launch location_source:=2 odom_topic:=/odom vel_topic:=/cmd_vel; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_ugv_control ugv_terminal_control.launch ugv_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 5.0; roslaunch sunray_simulator sunray_sim_ego_ugv.launch; exec bash"' \
