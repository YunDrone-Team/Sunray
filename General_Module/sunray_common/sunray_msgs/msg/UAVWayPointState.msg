## 自定义消息：无人机航点任务状态

## 消息header
std_msgs/Header header

## 当前正在执行的航点序号
uint8 wp_index

## 当前航点状态 0: 未执行 1: 执行中 2: 已完成
uint8 wp_state

uint8 NOT_READY = 0
uint8 UAV_READY = 1
uint8 TAKEOFF = 2
uint8 WAYPOINT = 3
uint8 RETURN = 4
uint8 LAND = 5

