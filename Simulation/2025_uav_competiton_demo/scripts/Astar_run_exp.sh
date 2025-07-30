#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=3 enable_rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 8.0; roslaunch 2025_uav_competiton_demo Astar.launch enable_rviz:=true; exec bash"' \
