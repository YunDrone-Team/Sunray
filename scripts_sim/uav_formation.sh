#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_3uav.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node_swarm.launch uav_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion_swarm.launch uav_num:=3 external_source:=2; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_orca orca_swarm_uav.launch agent_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_formation formation_uav_sim.launch agent_num:=3; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_formation formation_switch.launch; exec bash"'