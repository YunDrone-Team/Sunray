#!/bin/bash
# 无人机搜索二维码并发送给无人车 - 仿真脚本

# 启动Gazebo仿真环境、UAV控制
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_search_target.launch; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2; exec bash"' \
--tab -e 'bash -c "sleep 10.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"'
# --tab -e 'bash -c "sleep 5.0; roslaunch sunray_simulator sim_rviz.launch; exec bash"'

# 启动UGV控制与EGO规划器
gnome-terminal --window -e 'bash -c "sleep 8.0; roslaunch sunray_ugv_control ugv_control_sim.launch location_source:=2 odom_topic:=/odom vel_topic:=/cmd_vel; exec bash"' \
--tab -e 'bash -c "sleep 10.0; roslaunch sunray_ugv_control ugv_terminal_control.launch ugv_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 12.0; roslaunch sunray_simulator sunray_sim_ego_ugv.launch; exec bash"'

# 启动二维码检测节点
gnome-terminal --window -e 'bash -c "sleep 12.0; roslaunch sunray_tutorial landmark_detection_sim.launch; exec bash"'

# 启动二维码检测和搜索任务
gnome-terminal --window -e 'bash -c "sleep 18.0; roslaunch sunray_tutorial search_target_and_send_to_ugv.launch search_mode:=0; exec bash"'


