#!/bin/bash
# 无人机搜索二维码并发送给无人车 - 实验脚本

gnome-terminal --window -e 'bash -c "roslaunch sunray_uav_control sunray_mavros_exp.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0 enable_rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control terminal_control.launch uav_id:=1; exec bash"'

# 云台控制
gnome-terminal --window -e 'bash -c "sleep 5.0; roslaunch sunray_gimbal gimbal_qrtracker.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_gimbal gimbal_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 14.0; rostopic pub -1 /sunray/gimbal_down_cmd std_msgs/Bool \"data: true\"; exec bash"'

gnome-terminal --window -e 'bash -c "sleep 5.0; roslaunch sunray_tutorial landmark_detection.launch; exec bash"' \
--tab -e 'bash -c "sleep 15.0; roslaunch sunray_tutorial search_target_and_send_to_ugv.launch search_mode:=0; exec bash"'
