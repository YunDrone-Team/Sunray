#!/bin/bash

# 引入 TMUX 会话管理模块
source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.4"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 5 && roslaunch web_cam web_cam.launch
        sleep 5 && roslaunch sunray_uav_control external_fusion.launch external_source:=3 enable_rviz:=false uav_id:=${UAV_ID}
        sleep 2 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
        sleep 2 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
    "
    ["extra"]="
        sleep 3 && roslaunch sunray_tutorial qrcode_detection_down.launch uav_id:=${UAV_ID}
        sleep 5 && rosrun rqt_image_view rqt_image_view
        sleep 8 && roslaunch sunray_tutorial follow_a_car.launch uav_id:=${UAV_ID}
    "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
