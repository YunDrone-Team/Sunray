#!/bin/bash

# 引入 TMUX 会话管理模块
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "${SCRIPT_DIR}/auto_tmux.sh"

UAV_ID=1    # 无人机ID
SESSION_NAME=sunray_tmux  # 会话名称，统一使用sunray_tmux
FIRST_WINDOW="main.2"

# 自定义命令配置
declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${UAV_ID}
        sleep 8 && roslaunch sunray_uav_control external_fusion.launch external_source:=4 enable_rviz:=false uav_id:=${UAV_ID}
        sleep 5 && roslaunch sunray_viobot_unit mavlink.launch
        sleep 5 && roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${UAV_ID}
        sleep 2 && roslaunch sunray_uav_control terminal_control.launch uav_id:=${UAV_ID}
    "
)

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
