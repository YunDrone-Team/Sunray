#!/bin/bash
# 无人机搜索二维码并发送给无人车 - 实验脚本

gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=3 enable_rviz:=false uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=2; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=2; exec bash"'

gnome-terminal --window -e 'bash -c "sleep 8.0; roslaunch sunray_tutorial search_target_and_send_to_ugv.launch uav_id:=2 search_mode:=0; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch web_cam web_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 7.0; roslaunch sunray_tutorial landmark_detection.launch; exec bash"'