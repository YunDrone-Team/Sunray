#!/bin/bash
# 脚本：无人机控制测试脚本(3机)
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 10.0; roslaunch sunray_simulator sunray_sim_3uav.launch vehicle:=sunray150; exec bash"' \
--tab -e 'bash -c "sleep 10.0; roslaunch sunray_uav_control external_fusion_swarm.launch uav_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node_swarm.launch uav_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_num:=3; exec bash"' \

