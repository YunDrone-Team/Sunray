## 消息header
std_msgs/Header header

## 控制命令
uint8 control_cmd

## 移动命令
uint8 move_cmd

## 移动命令枚举(基于惯性系)
uint8 XyzPosYaw = 1                  # 移动指令：惯性系定点控制 带偏航角, 需要配合desired_pos和desired_yaw使用

## 控制命令枚举 
uint8 Takeoff = 100               # 起飞，home点上方悬停
uint8 Land = 101                  # 原地降落
uint8 Hover = 102                 # 当前位置悬停
uint8 Move = 103                  # 移动
uint8 Return = 104                # 返航

## 移动命令期望值
geometry_msgs/Point desired_position         ## 期望位置，单位：[m]
geometry_msgs/Point desired_velocity         ## 期望速度，单位：[m/s]
geometry_msgs/Point acceleration             ## 期望加速度，单位：[m/s^2]
float32 desired_yaw                          ## 期望偏航角，单位：[rad]
float32 desired_yaw_rate                     ## 期望偏航角角速率，单位：[rad/s]



