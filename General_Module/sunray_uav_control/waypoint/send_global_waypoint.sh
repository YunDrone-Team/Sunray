#!/bin/bash
# 当前 WayPoint.msg 只包含局部坐标 x/y/z/yaw，不能直接表达经纬高航点。
# 如果需要 GPS/RTK 全局航点，请先扩展 sunray_msgs/Point 或新增独立全局航点消息，
# 再在 waypoint_mission_node 中完成经纬高到本地 ENU 的转换。

echo "当前航点接口仅支持局部坐标 x/y/z/yaw。"
echo "send_global_waypoint.sh 暂不发布消息，避免向 /uavX/sunray/uav_waypoint 发送无效字段。"
echo "请使用 send_local_waypoint.sh，或先扩展 WayPoint/Point 消息以支持 latitude/longitude/altitude。"
exit 1
