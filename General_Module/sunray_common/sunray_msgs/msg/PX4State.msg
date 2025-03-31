## 自定义消息：PX4飞控所有状态汇总

## 消息header
std_msgs/Header header

## 无人机基本信息
bool connected                  ## 是否与飞控建立连接
bool armed                      ## 是否解锁，true为已解锁，false则不是
string mode                     ## PX4飞行模式（PX4飞控端）
float32 battery_state           ## 电池电压，单位：[V]
float32 battery_percetage       ## 电池百分比，[0-1]

## 外部定位数据的原始值
ExternalOdom external_odom

## 无人机本地定位信息
float32[3] position                 ## 位置，单位：[m]
float32[3] velocity                 ## 速度，单位：[m/s]
float32[3] attitude                 ## 姿态，单位：[rad]

## 无人机全局定位信息
## TODO

## 无人机姿态信息
geometry_msgs/Quaternion attitude_q ## 姿态，四元数
float32[3] attitude_rate            ## 角速率，单位[rad/s]

## PX4飞控控制器的目标设定值
float32[3] pos_setpoint             ## 位置控制设定值，单位：[m]  
float32[3] vel_setpoint             ## 速度控制设定值，单位：[m/s] 
float32[3] att_setpoint             ## 姿态控制设定值，单位：[rad] 
float thrust_setpoint               ## 推力设定值，[0-1]





