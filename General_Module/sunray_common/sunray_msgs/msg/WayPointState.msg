## 自定义消息：无人机航点任务状态

## 消息header
std_msgs/Header header

## 当前航点状态 
uint8 wp_state

## 当前航点状态枚举
uint8 NOT_READY = 1
uint8 UAV_READY = 2
uint8 WAYPOINT = 3
uint8 LAND = 4

## 当前正在执行的航点序号
uint8 wp_index

## 所有航点数量
uint8 wp_num

## 当前飞行速度
float32[3] waypoint                 ## 单位：[m]
float32[2] velocity                 ## 单位：[m/s]
float32 yaw

