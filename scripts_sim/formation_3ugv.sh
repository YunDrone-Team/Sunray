#!/bin/bash
num=3
if [ -n "$1" ]; then
  num=$(($1))
fi

echo "Starting formation simulation with ${num} UGVs..."
# start gazebo
gnome-terminal --window -e "bash -c \"roscore; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0;roslaunch sunray_simulator sunray_sim_3ugv.launch; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0;roslaunch sunray_ugv_control ugv_control_swarm.launch ugv_num:=${num}; exec bash\"" \

gnome-terminal --window -e "bash -c \"sleep 3.0; roslaunch sunray_orca orca_swarm_ugv.launch agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 3.0; roslaunch sunray_formation formation_ugv_sim.launch agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 2.0; roslaunch sunray_formation formation_switch.launch; exec bash\"" 