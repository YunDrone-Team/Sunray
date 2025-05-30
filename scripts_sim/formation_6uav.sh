#!/bin/bash
# start gazebo
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_6uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control external_fusion_swarm.launch uav_num:=6; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_control_node_swarm.launch uav_num:=6; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_num:=6; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_orca orca_swarm.launch agent_num:=6; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_formation formation.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; rosrun sunray_formation formation_switch_node"' \