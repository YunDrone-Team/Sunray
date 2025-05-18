## 自定义消息：无人车状态

## 消息header
std_msgs/Header header

## 无人车基本信息
uint8 ugv_id                ## 无人车编号
bool connected              ## 无人车连接状态
## 无人车电池状态
float32 battery_state       ## [V]
float32 battery_percentage   ## [0-1]

## 无人车里程计信息
uint8 location_source       ## 无人车定位来源
bool odom_valid             ## 里程计是否有效

## 无人车状态信息
float32[2] position         ## [m]
float32[2] velocity         ## [m/s]
float32 yaw                 ## [rad]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数

## 无人车控制命令及目标设定值
uint8 control_mode
float32[2] pos_setpoint     ## [m]  位置控制目标点
float32[2] vel_setpoint     ## [m/s]  速度控制目标
float32 yaw_setpoint        ## [rad]  姿态控制目标

## 无人车控制状态（control_mode）枚举 
uint8 INIT=0
uint8 HOLD=1
uint8 POS_CONTROL_ENU=2
## 以下需要有定位的情况下才支持
uint8 POS_CONTROL_BODY=3
uint8 VEL_CONTROL_ENU=4
uint8 VEL_CONTROL_BODY=5
uint8 Point_Control_with_Astar=6

## 无人车参数信息
float32[2] home_pos             ## 原点
float32 home_yaw                ## 原点
float32[2] hover_pos            ## 悬停点
float32 hover_yaw               ## 悬停点