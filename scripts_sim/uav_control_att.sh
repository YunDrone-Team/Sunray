#!/bin/bash
# 脚本：无人机控制测试脚本
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_simulator sunray_sim_1uav.launch vehicle:=sunray150_with_mid360; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_uav_control sunray_control_test.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_simulator sim_rviz.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_uav_control control_evaluation_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_communication_bridge sunray_communication_bridge_sim.launch; exec bash"' \

