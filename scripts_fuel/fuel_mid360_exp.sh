#!/bin/bash
# 脚本：实机 FUEL，地图输入来自 MID360 + FAST-LIO 实时点云

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils FuelTerminalControl.launch agent_id:=1; exec bash"'

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_planner_utils exploration_mid360.launch cloud_topic:=/cloud_registered; exec bash"'
