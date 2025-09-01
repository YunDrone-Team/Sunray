#!/bin/bash
id=1
num=3
if [ -n "$1" ]; then
  id=$(($1))
fi
if [ -n "$2" ]; then
  num=$(($2))
fi

# 引入 TMUX 会话管理模块
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
source "${SCRIPT_DIR}/auto_tmux.sh"

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.2"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_ugv_control wheeltec_robot.launch  ugv_id:=${id}
        sleep 5.0; roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=${id}
    "
    ["extra"]="
        sleep 3.0; roslaunch sunray_orca orca_ugv.launch agent_id:=${id} agent_num:=${num}
        sleep 5.0; roslaunch sunray_formation formation_single_ugv.launch agent_id:=${id}  agent_num:=${num}
    "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
