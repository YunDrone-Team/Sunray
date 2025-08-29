#!/bin/bash

# 引入 TMUX 会话管理模块
source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.0"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control external_fusion.launch external_source:=0 enable_rviz:=false uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
        sleep 4 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
    "
    ["extra"]="
        sleep 4 && roslaunch sunray_planner_utils msg_MID360.launch
        sleep 6 && roslaunch sunray_planner_utils mapping_mid360.launch
        sleep 8 && roslaunch sunray_planner_utils sunray_ego_single_mid360.launch
    "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
