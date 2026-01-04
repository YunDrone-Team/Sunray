#!/bin/bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=6; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control waypoint_mission_node.launch; exec bash"' \

