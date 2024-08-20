std_msgs/Header header

## 无人机Setup类型（可用于模拟遥控器）
uint8 cmd
uint8 DISARM = 0
uint8 ARM = 1
uint8 SET_PX4_MODE = 2
uint8 REBOOT_PX4 = 3
uint8 SET_CONTROL_MODE = 4
uint8 EMERGENCY_KILL = 5

## PX4模式名查询:http://wiki.ros.org/mavros/CustomModes
## 常用模式名:OFFBOARD,AUTO.LAND,AUTO.RTL,POSCTL
string px4_mode

## INIT,RC_CONTROL,HOVER_CONTROL,CMD_CONTROL,LAND_CONTROL
string control_state
