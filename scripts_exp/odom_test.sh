#!/bin/bash
# 加载bashrc
source /home/PRR/.bashrc

# 在后台启动第一个进程并记录PID
roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=2 &
mavros_pid=$!
# echo "启动sunray_mavros_exp.launch，进程ID: $mavros_pid"

# 等待3秒
sleep 3

# 在后台启动第二个进程并记录PID
roslaunch sunray_uav_control external_fusion.launch external_source:=4 enable_rviz:=false uav_id:=2 &
fusion_pid=$!
# echo "启动external_fusion.launch，进程ID: $fusion_pid"

# 等待所有后台进程完成
wait
echo "所有进程已完成"

# 捕获Ctrl+C信号，确保能够优雅终止进程
trap "kill $mavros_pid 2>/dev/null; kill $fusion_pid 2>/dev/null; sleep 1; exit 0" SIGINT

# 保持脚本运行，直到用户中断
while true; do
    sleep 1
done
