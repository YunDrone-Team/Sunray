#!/bin/bash
# 发送全局坐标（经纬高）航点脚本

# 设置无人机ID和名称
UAV_ID=1
UAV_NAME="uav"

echo "============================================"
echo "发送全局坐标（GPS）航点到 /${UAV_NAME}${UAV_ID}/sunray/uav_waypoint"
echo "注意：请根据实际GPS坐标修改航点！"
echo "============================================"

# 示例GPS坐标
LAT1=47.39774
LON1=8.545593
ALT1=488

LAT2=47.39776
LON2=8.545593
ALT2=488

LAT3=47.39778
LON3=8.545593
ALT3=488

rostopic pub -1 /${UAV_NAME}${UAV_ID}/sunray/uav_waypoint sunray_msgs/WayPoint "
header:
  stamp: now
start: true
wp_num: 3
wp_end_type: 1
wp_yaw_type: 1
wp_coordinate_type: 2
wp_move_vel: 2.0
wp_points:
- x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
  latitude: ${LAT1}
  longitude: ${LON1}
  altitude: ${ALT1}
- x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
  latitude: ${LAT2}
  longitude: ${LON2}
  altitude: ${ALT2}
- x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
  latitude: ${LAT3}
  longitude: ${LON3}
  altitude: ${ALT3}
"

echo ""
echo "全局坐标（GPS）航点已发送！"
echo "高度类型: AMSL（平均海平面高度）"

