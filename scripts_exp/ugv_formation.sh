#!/bin/bash
id=1
num=3
if [ -n "$1" ]; then
  id=$(($1))
fi
if [ -n "$2" ]; then
  num=$(($2))
fi

gnome-terminal --window -e "bash -c \"roslaunch sunray_ugv_control wheeltec_robot.launch  ugv_id:=${id}; exec bash; exec bash\"" \
                --tab -e "bash -c \"sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=${id}; exec bash\"" \

gnome-terminal --window -e "bash -c \"sleep 3.0; roslaunch sunray_orca orca_ugv.launch agent_id:=${id} agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 5.0; roslaunch sunray_formation formation_single_ugv.launch agent_id:=${id}  agent_num:=${num}; exec bash\"" \
                --tab -e "bash -c \"sleep 5.0; roslaunch sunray_communication_bridge sunray_communication_bridge.launch ugv_id:=${id} ugv_experiment_num:=${num}; exec bash\"" \

