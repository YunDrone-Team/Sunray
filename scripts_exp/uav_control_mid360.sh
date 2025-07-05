#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"'