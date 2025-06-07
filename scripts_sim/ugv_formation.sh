#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_simulator sunray_sim_3ugv.launch; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_ugv_control ugv_control_swarm.launch ugv_num:=3; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_orca orca_swarm_ugv.launch agent_num:=3; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_formation formation_ugv_sim.launch agent_num:=3; exec bash"' \

