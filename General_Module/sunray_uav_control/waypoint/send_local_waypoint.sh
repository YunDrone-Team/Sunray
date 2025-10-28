#!/bin/bash
# 发送局部坐标航点脚本

# 设置无人机ID和名称
UAV_ID=1
UAV_NAME="uav"

echo "============================================"
echo "发送局部坐标航点到 /${UAV_NAME}${UAV_ID}/sunray/uav_waypoint"
echo "航点: (0,0,2) -> (3,0,2) -> (3,3,2) -> (0,3,2)"
echo "============================================"

rostopic pub -1 /${UAV_NAME}${UAV_ID}/sunray/uav_waypoint sunray_msgs/WayPoint "
header:
  stamp: now
start: true
wp_num: 4
wp_end_type: 1
wp_yaw_type: 2
wp_coordinate_type: 1
wp_move_vel: 1.5
wp_points:
- x: 0.0
  y: 0.0
  z: 2.0
  yaw: 0.0
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
- x: 3.0
  y: 0.0
  z: 2.0
  yaw: 0.0
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
- x: 3.0
  y: 3.0
  z: 2.5
  yaw: 0.0
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
- x: 0.0
  y: 3.0
  z: 2.5
  yaw: 0.0
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0
"

echo ""
echo "局部坐标航点已发送！"
