# OrcaCmd.msg
# 发布指令到Orca节点
## 消息header
std_msgs/Header header

## 指令类型
uint8 cmd
## 指令枚举
uint8 RUN=1
uint8 STOP=2
## 设置期望位置 配合desired_pos+desired_yaw使用
uint8 GOAL=3 
## 设置期望位置并切换为RUN状态 配合desired_pos+desired_yaw使用
uint8 GOAL_RUN=4
## 期望位置，单位：[m]
float32[3] desired_pos
## 期望速度，单位：[rad]
float32 desired_yaw