#! /bin/bash
# /bin/bash -c "sleep 5 && gnome-terminal --title="sunray_server" -- bash -c "/home/yundrone/Sunray/server/server.sh; exec bash""
env_file=/home/yundrone/Sunray/server/server.env
## 虽然环境变量里面已经做了如下设置，但是设置为开机自启动脚本的时候，系统还没有完全启动，所以需要手动source
source /opt/ros/noetic/setup.bash
source ~/Sunray/devel/setup.bash

# 安全加载 .env 文件（避免代码注入）
while IFS='=' read -r key value || [[ -n "$key" ]]; do
        # 跳过注释和空行
        [[ -z "$key" || "$key" == \#* ]] && continue

        # 清理键和值
        key=$(echo "$key" | xargs)
        value=$(echo "$value" | xargs | tr -d '"'"'")

        # 导出变量
        export "$key"="$value"
        echo "参数设置: $key=$value"
    done < "$env_file"

# 检查布尔参数是否合法
validate_bool() {
    local var_name="$1"
    local var_value="${!var_name,,}"  # 转换为小写

    if ! [[ "$var_value" =~ ^(true|false|1|0)$ ]]; then
        echo "错误：$var_name 必须是 true/false/1/0，当前值为 '$var_value'"
        exit 1
    fi
}

if [[ "${RUN_SEVER,,}" == "true" ]]; then
    gnome-terminal --window --title="roscore" -- bash -c "source /opt/ros/noetic/setup.bash; ~/Sunray/devel/setup.bash; roscore; exec bash"
fi


# 获取ip地地址
IP=$(ip route get 1.2.3.4 2>/dev/null | awk '{print $7}' | head -1)
echo "IP地址为：$IP"

# 等待开发主机中的roscore就绪
echo "等待roscore启动..."

max_wait=1000
timeout_flag=true  # 添加超时标志

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
    exit 1  # 非零退出码表示异常退出
fi




# 启动地面站后台节点
start_ground_station() {
    gnome-terminal --title="sunray_communication_bridge" -- bash -c "source /opt/ros/noetic/setup.bash; sleep 5; \
    source ~/Sunray/devel/setup.bash; roslaunch sunray_communication_bridge sunray_communication_bridge.launch uav_id:=${ID:=1} uav_experiment_num:=${NUM:=1}; exec bash"
}

# start_ground_station() {
#     gnome-terminal --title="sunray_communication_bridge" -- bash -c "source /opt/ros/noetic/setup.bash; sleep 5; \
#     source ~/Sunray/devel/setup.bash; roslaunch sunray_communication_bridge sunray_communication_bridge.launch ugv_id:=${ID:=1} ugv_experiment_num:=${NUM:=1}; exec bash"
# }

# 启动mavros节点
start_mavros_station() {
    gnome-terminal --title="sunray_mavros" -- bash -c "source /opt/ros/noetic/setup.bash; sleep 5; \
    source ~/Sunray/devel/setup.bash; roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=${ID} ip:=${IP}; exec bash"
}

# 启动外部定位节点
start_external_position() {
    gnome-terminal --title="external_fusion" -- bash -c "source /opt/ros/noetic/setup.bash; sleep 5; \
     source ~/Sunray/devel/setup.bash; roslaunch sunray_uav_control external_fusion.launch uav_id:=${ID} external_source:=${EXTERNAL_SOURCE}; exec bash"
}

# 启动控制节点
start_control() {
    gnome-terminal --title="external_fusion" -- bash -c "source /opt/ros/noetic/setup.bash; sleep 5; \
    source ~/Sunray/devel/setup.bash; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=${ID}; exec bash"
}

# 主逻辑
main() {
    

    # 验证布尔参数
    validate_bool "START_GROUND_STATION"
    validate_bool "START_EXTERNAL_POSITION"
    validate_bool "START_CONTROL"

    # 根据参数决定启动哪些节点
    if [[ "${START_GROUND_STATION,,}" == "true" ]]; then
        start_ground_station
    else
        echo "跳过地面站后台节点启动"
    fi
    if [[ "${START_MAVROS,,}" == "true" ]]; then
        start_mavros_station
    else
        echo "跳过来MAVROS节点启动"
    fi
    if [[ "${START_EXTERNAL_POSITION,,}" == "true" ]]; then
        start_external_position
    else
        echo "跳过来外部定位节点启动"
    fi

    if [[ "${START_CONTROL,,}" == "true" ]]; then
        start_control
    else
        echo "跳过控制节点启动"
    fi

    echo "所有配置已处理完成！"
}

main

