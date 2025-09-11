#!/bin/bash

# 网络就绪检查函数（不依赖公网）
check_network() {
    echo "正在检查本地网络连接..."
    
    local max_attempts=180  # 最大尝试次数
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        # 检查是否有至少一个非环回网络接口处于运行状态
        local active_interfaces=$(ip link show | grep -v LOOPBACK | grep -c "state UP")
        
        # 检查是否有默认网关
        local default_gateway=$(ip route show default | wc -l)
        
        # 检查是否获取到IP地址（非环回）
        local ip_address=$(ip -4 addr show | grep -v LOOPBACK | grep -oP '(?<=inet\s)\d+(\.\d+){3}/\d+' | wc -l)
        
        # 如果有活动接口、默认网关和IP地址，则认为网络就绪
        if [ $active_interfaces -gt 0 ] && [ $default_gateway -gt 0 ] && [ $ip_address -gt 0 ]; then
            echo "本地网络连接已就绪！"
            return 0
        fi
        
        echo "本地网络未就绪，等待中... ($attempt/$max_attempts)"
        attempt=$((attempt + 1))
        sleep 1
    done
    
    echo "错误：等待本地网络超时！"
    return 1
}

# 执行网络检查，如果失败则退出
if ! check_network; then
    exit 1
fi

# 等待系统完全启动
sleep 15

# 加载环境配置
source /home/PRR/Sunray/devel/setup.sh

# 等待roscore就绪
echo "等待roscore启动..."

max_wait=1000
timeout_flag=true

for ((i=1; i<=$max_wait; i++)); do
    if rosnode list 2>/dev/null | grep -q '/rosout'; then
        echo "roscore 已就绪！"
        timeout_flag=false
        break
    fi
    echo "等待中... ($i/$max_wait)"
    sleep 1
done

# 超时后退出脚本
if $timeout_flag; then
    echo "错误：等待 roscore 超时！"
    exit 1
fi

# 启动节点
roslaunch sunray_communication_bridge sunray_communication_bridge.launch uav_id:=1 uav_experiment_num:=1

