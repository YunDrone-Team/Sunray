#!/bin/bash
# 脚本：无人机控制测试脚本 - GPS
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion.launch external_source:=6; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch; exec bash"' \