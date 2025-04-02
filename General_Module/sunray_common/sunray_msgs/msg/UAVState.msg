## 自定义消息：无人机状态

## 消息header
std_msgs/Header header

## 无人机基本信息
uint8 uav_id                ## 无人机编号
bool connected              ## 是否与飞控建立连接
bool armed                  ## 是否解锁，true为已解锁，false则不是
string mode                 ## PX4飞行模式（PX4飞控端）
uint8 control_mode          ## 无人机控制模式（Sunray端无人机控制）
uint8 move_mode             ## 无人机移动模式（Sunray端无人机控制）

## 无人机里程计信息
uint8 location_source               ## 无人机定位来源
bool odom_valid                     ## 里程计是否有效

## 无人机状态信息
float32[3] position                 ## 位置，单位：[m]
float32[3] velocity                 ## 速度，单位：[m/s]
float32[3] attitude                 ## 姿态，单位：[rad]
geometry_msgs/Quaternion attitude_q ## 姿态，四元数
float32[3] attitude_rate            ## 角速率，单位[rad/s]

## PX4飞控控制器的目标设定值
float32[3] pos_setpoint             ## 位置控制设定值，单位：[m]  
float32[3] vel_setpoint             ## 速度控制设定值，单位：[m/s] 
float32[3] att_setpoint             ## 姿态控制设定值，单位：[rad] 

## 无人机电池状态
float32 battery_state               ## 电池电压，单位：[V]
float32 battery_percentage           ## 电池百分比，[0-1]