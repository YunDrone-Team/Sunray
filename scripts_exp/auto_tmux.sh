#!/bin/bash

# TMUX 会话管理模块
# 用法: source auto_tmux.sh
#        create_tmux_session

# 默认配置（可以在引入文件前覆盖这些变量）
SESSION_NAME=${SESSION_NAME:-"sunray_tmux"} # TMUX 会话名称
UAV_ID=${UAV_ID:-1} # UAV_ID
FIRST_WINDOW=${FIRST_WINDOW:-"main.0"} # 最后选择的窗口和窗格（格式：窗口名.窗格索引）
LAYOUT=${LAYOUT:-"even-horizontal"} # 可选: tiled, even-horizontal, even-vertical, main-horizontal, main-vertical

# 默认命令配置（可以在引入文件前覆盖）
# 窗口名称:命令数组
declare -A TMUX_CONFIG
if [ -z "${TMUX_CONFIG+set}" ]; then
    # 如果 TMUX_CONFIG 未设置，使用默认值
    TMUX_CONFIG=(
        ["main"]="
            echo '默认主窗口命令'
            echo '这是第二个命令'
        "
        ["extra"]="
            echo '额外窗口命令'
        "
    )
fi

# 创建 TMUX 会话函数
create_tmux_session() {
    # 关闭可能存在的会话
    tmux kill-session -t ${SESSION_NAME} 2> /dev/null   

    # 创建新会话
    tmux new-session -d -s $SESSION_NAME

    # 处理每个窗口
    for window_name in "${!TMUX_CONFIG[@]}"; do
        # 获取该窗口的命令列表
        IFS=$'\n' read -r -d '' -a raw_commands <<< "${TMUX_CONFIG[$window_name]}"
        
        # 清理命令：移除空行和首尾空格
        commands=()
        for cmd in "${raw_commands[@]}"; do
            # 移除首尾空格
            trimmed_cmd=$(echo "$cmd" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')
            # 跳过空行
            if [ -n "$trimmed_cmd" ]; then
                commands+=("$trimmed_cmd")
            fi
        done
        
        # 如果没有命令，跳过
        if [ ${#commands[@]} -eq 0 ]; then
            continue
        fi
        
        # 创建窗口（如果是第一个窗口，使用new-session已经创建）
        if [ "$window_name" != "main" ]; then
            tmux new-window -d -t $SESSION_NAME -n "$window_name"
        else
            tmux rename-window -t $SESSION_NAME:0 "$window_name"
        fi
        
        # 执行第一个命令在主窗格
        tmux send-keys -t $SESSION_NAME:$window_name "${commands[0]}" C-m
        
        # 为后续命令创建新窗格
        for ((i=1; i<${#commands[@]}; i++)); do
            tmux split-window -h -t $SESSION_NAME:$window_name
            tmux send-keys -t $SESSION_NAME:$window_name "${commands[$i]}" C-m
        done
        
        # 应用布局
        tmux select-layout -t $SESSION_NAME:$window_name $LAYOUT
    done

    # 设置最后选择的窗口和窗格
    if [[ -n "$FIRST_WINDOW" ]]; then
        IFS='.' read -r target_window target_pane <<< "$FIRST_WINDOW"
        tmux select-window -t $SESSION_NAME:$target_window
        tmux select-pane -t $SESSION_NAME:$target_window.$target_pane
    fi

    # 返回会话信息
    echo "Tmux会话 '$SESSION_NAME' 已创建:"
    tmux list-windows -t $SESSION_NAME
}

# 附加到会话函数
attach_to_tmux_session() {
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        read -p "是否附加到会话 '$SESSION_NAME'? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            tmux attach -t $SESSION_NAME
        fi
    else
        echo "会话 '$SESSION_NAME' 不存在"
    fi
}

# 检查是否直接执行此脚本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    create_tmux_session
    attach_to_tmux_session
fi