#!/bin/bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_simulator sunray300_sim_3uav.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node_swarm.launch uav_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control external_fusion_swarm.launch uav_num:=3 external_source:=5; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_orca orca_swarm_uav.launch agent_num:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_formation leader_follower_sim.launch agent_num:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_formation missioncmd_pub.launch; exec bash"' \
