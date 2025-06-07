#!/bin/bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_ugv_control wheeltec_robot.launch ugv_id:=1; exec bash; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 3.0; roslaunch sunray_orca orca_ugv.launch agent_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_formation formation_single_ugv.launch agent_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_communication_bridge sunray_communication_bridge.launch ugv_id:=1 ugv_experiment_num:=3; exec bash"' \

