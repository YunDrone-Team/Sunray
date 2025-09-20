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
        roslaunch aidetection_yolov7_ros aidetection_yolov7_viobot.launch
        sleep 3 && roslaunch sunray_media rtsp_push.launch
    "
)

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
