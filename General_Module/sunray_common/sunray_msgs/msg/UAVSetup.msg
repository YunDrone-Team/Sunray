## 自定义消息：无人机设置指令

## 消息header
std_msgs/Header header

## 无人机Setup类型（可用于模拟遥控器）
uint8 cmd

## 无人机Setup类型枚举
uint8 DISARM = 0                ## 解锁无人机
uint8 ARM = 1                   ## 无人机上锁
uint8 SET_PX4_MODE = 2          ## 设置PX4模式，需要配合px4_mode进行设置
uint8 REBOOT_PX4 = 3            ## 重启PX4飞控
uint8 SET_CONTROL_MODE = 4      ## 设置无人机控制模式，需要配合control_mode进行设置
uint8 EMERGENCY_KILL = 5        ## 无人机紧急上锁

## PX4模式名查询:http://wiki.ros.org/mavros/CustomModes
## 常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
string px4_mode

## 参见uav_control.h中无人机控制模式枚举：INIT,RC_CONTROL,CMD_CONTROL,LAND_CONTROL
string control_mode

## 无人机control_mode类型枚举
uint8 INIT = 0                      ## 初始模式
uint8 RC_CONTROL = 1                ## 遥控器控制模式
uint8 CMD_CONTROL = 2               ## 外部指令控制模式
uint8 LAND_CONTROL = 3              ## 降落模式
uint8 WITHOUT_CONTROL = 4           ## 无控制模式