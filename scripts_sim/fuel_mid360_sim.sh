#!/bin/bash
# 脚本：Gazebo 仿真 FUEL，地图输入来自 Sunray MID360 实时点云

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_uav_planning.launch vehicle:=sunray150_with_mid360 world:=$(rospack find sunray_simulator)/worlds/planning_test.world gui:=true; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2 position_topic:=/uav1/sunray/gazebo_pose; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils FuelTerminalControl.launch agent_id:=1; exec bash"'

gnome-terminal --window -e 'bash -c "sleep 6.0; roslaunch sunray_simulator sunray_sim_fuel.launch local_pointcloud_topic_1:=/uav1/livox/lidar global_pointcloud_topic:=/uav1/global_points enable_rviz:=true; exec bash"'
