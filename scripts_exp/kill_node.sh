#!/bin/bash

# ROS节点白名单（用空格分隔）
WHITELIST="rosout rosmaster /sunray_communication_bridge"

# 获取所有非白名单节点
NODE_LIST=$(rosnode list | grep -v -E "$(echo $WHITELIST | tr ' ' '|')")

for NODE in $NODE_LIST; do
    # 杀死ROS节点
    rosnode kill $NODE
    
    # 杀死相关进程
    pkill -f "python.*$NODE"
    pkill -f "$NODE"

    # 查找px4
    PX4_PID=$(ps aux | grep "px4" | grep -v grep | awk '{print $2}')
    if [ -n "$PX4_PID" ]; then
        kill -9 $PX4_PID
    fi
    
    echo "已清理节点: $NODE"
done