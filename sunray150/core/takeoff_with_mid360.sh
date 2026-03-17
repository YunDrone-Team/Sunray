#!/bin/bash

gnome-terminal --window -e 'bash -c "roslaunch sunray150 sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 external_fusion.launch external_source:=0; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray150 waypoint_mission_node.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray150 msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray150 mapping_mid360.launch rviz:=false; exec bash"'
