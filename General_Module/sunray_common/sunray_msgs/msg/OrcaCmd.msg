# OrcaCmd.msg
# Orca节点状态
## 消息header
std_msgs/Header header

## 状态类型
uint8 state
## 状态枚举
uint8 INIT=0
uint8 RUN=1
uint8 STOP=2
uint8 ARRIVED=3

## 期望位置，单位：[m]
float32[3] goal_pos
## 期望速度，单位：[rad]
float32 goal_yaw

## 速度指令
float32[3] linear
float32[3] angular
