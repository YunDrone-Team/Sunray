#!/bin/bash
# start gazebo
# start ugv control
# start map

gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_uav_planning.launch rviz_enable:=false; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"' \
# --tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sim_rviz.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_ego.launch; exec bash"' \
