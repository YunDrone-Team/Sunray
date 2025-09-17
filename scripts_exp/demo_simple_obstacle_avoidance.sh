#!/bin/bash

# 引入 TMUX 会话管理模块
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "${SCRIPT_DIR}/auto_tmux.sh"

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="extrn.1"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 5 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
    "
    ["extrn"]="
        sleep 3 && roslaunch sunray_uav_control external_fusion.launch external_source:=4 position_topic:=/baton/stereo3/odometry enable_rviz:=false use_vision_pose:=false tilted:=true uav_id:=${UAV_ID}
        sleep 5 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
        sleep 5 && roslaunch sunray_tutorial simple_obstacle_avoidance.launch uav_id:=${UAV_ID}
    "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
