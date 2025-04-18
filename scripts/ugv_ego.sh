#!/bin/bash
# 运行轮趣无人车底层驱动节点
gnome-terminal --window -e 'bash -c "roslaunch sunray_ugv_control wheeltec_robot.launch; exec bash"' \
# 运行无人车控制节点
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_ugv_control ugv_control_exp_with_ego.launch; exec bash"' \
# 运行EGO算法 & 运行EGO算法指令to无人车指令
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_TV sunray_ego_exp_ugv.launch; exec bash"' \

