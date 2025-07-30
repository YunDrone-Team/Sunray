#!/bin/bash

trap "echo '捕获到 Ctrl+C，正在关闭...'; kill $roscore_pid $sim_pid $fusion_pid $control_pid $demo_pid; exit 1" SIGINT

echo "启动 roscore ..."
roscore &
roscore_pid=$!
sleep 5

echo "启动 sim ..."
roslaunch sunray_simulator sunray_sim_1uav.launch &
sim_pid=$!
sleep 5

echo "启动 publisher ..."
roslaunch 2025_uav_competiton_demo position_pub.launch &
sim_pid=$!
sleep 2

echo "启动 fusion ..."
roslaunch sunray_uav_control external_fusion.launch external_source:=2 enable_rviz:=false &
fusion_pid=$!
sleep 2

echo "启动 control ..."
roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1 &
control_pid=$!
sleep 2

echo "运行 demo ..."
roslaunch 2025_uav_competiton_demo Astar.launch enable_rviz:=true &
demo_pid=$!

wait
