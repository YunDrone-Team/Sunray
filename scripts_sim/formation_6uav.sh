#!/bin/bash
num=6
if [ -n "$1" ]; then
  num=$(($1))
fi
# start gazebo
gnome-terminal --window -e "bash -c \"roscore; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_simulator sunray_sim_6uav.launch; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_uav_control external_fusion_swarm.launch uav_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_uav_control sunray_control_node_swarm.launch uav_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_num:=${num}; exec bash\"" \

gnome-terminal --window -e "bash -c \"sleep 3.0; roslaunch sunray_orca orca_swarm_uav.launch agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 3.0; roslaunch sunray_formation formation_uav_sim.launch agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_formation formation_switch.launch; exec bash\"" 