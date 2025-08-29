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
source /home/ray/Sunray/scripts_tmux/auto_tmux.sh

# ===================== 配置区域 =====================
UAV_ID=1
SESSION_NAME=sunray_tmux
FIRST_WINDOW="main.2"
LAYOUT="even-horizontal"

declare -A TMUX_CONFIG=(
    ["main"]="
        roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${id}
        sleep 5.0; roslaunch sunray_uav_control external_fusion.launch uav_id:=${id} external_source:=3
        sleep 2.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${id}
    "
    ["extra"]="
        sleep 3.0; roslaunch sunray_orca orca_uav.launch agent_id:=${id} agent_num:=${num}
        sleep 5.0; roslaunch sunray_formation formation_single_uav.launch agent_id:=${id}  agent_num:=${num}
    "
)
# ===================== 配置结束 =====================

# 创建会话
create_tmux_session

# 可选：附加到会话
attach_to_tmux_session
