## 自定义消息：PX4飞控所有状态汇总

## 消息header
std_msgs/Header header

## PX4状态信息 - 来自~/mavros/state
bool connected                  ## 是否与飞控建立连接
bool armed                      ## 是否解锁，true为已解锁，false则不是
string mode                     ## PX4飞行模式（PX4飞控端）

## PX4状态信息 - 来自~/mavros/extendedstate
uint8 landed_state              ## 降落状态，枚举参见mavros_msgs::msg::ExtendedState
## landed_state枚举
uint8 LANDED_STATE_UNDEFINED = 0
uint8 LANDED_STATE_ON_GROUND = 1
uint8 LANDED_STATE_IN_AIR = 2
uint8 LANDED_STATE_TAKEOFF = 3
uint8 LANDED_STATE_LANDING = 4

## 无人机电池信息 - 来自~/mavros/battery
float32 battery_state           ## 电池电压，单位：[V]
float32 battery_percentage      ## 电池百分比，[0-1]

## 外部定位数据的原始值 - 与发送到~/mavros/vision_pose的内容一致
sunray_msgs/ExternalOdom external_odom

## 无人机本地定位信息（位置） - 来自~/mavros/local_position/pose
float32[3] position                 ## 位置，单位：[m]

## 无人机本地定位信息（速度） - 来自~/mavros/local_position/velocity_local
float32[3] velocity                 ## 速度，单位：[m/s]

## 无人机姿态信息 - 来自~/mavros/imu/data
float32[3] attitude                 ## 姿态，单位：[rad]
geometry_msgs/Quaternion attitude_q ## 姿态，四元数
float32[3] attitude_rate            ## 角速率，单位[rad/s]

## 无人机全局定位信息（卫星数量） - 来自~/mavros/global_position/raw/satellites
uint8 satellites                 ## GPS卫星数量

## 无人机全局定位信息（GPS状态） - 来自~/mavros/global_position/global
uint8 gps_status                 ## GPS状态
uint8 gps_service                ## GPS服务

## 无人机全局定位信息（经纬度） - 来自~/mavros/gps1/raw
float64 latitude                 ## 纬度，单位：[°]
float64 longitude                ## 经度，单位：[°]
float64 altitude                 ## 高度，单位：[m]

## PX4中无人机的位置/速度/加速度设定值 - 来自~/mavros/setpoint_raw/target_local
float32[3] pos_setpoint             ## 位置控制设定值，单位：[m]  
float32[3] vel_setpoint             ## 速度控制设定值，单位：[m/s] 

## PX4中无人机的姿态设定值 - 来自~/mavros/setpoint_raw/target_attitude
float32[3] att_setpoint             ## 姿态控制设定值，单位：[rad] 
geometry_msgs/Quaternion q_setpoint ## 姿态，四元数
float32 thrust_setpoint             ## 推力设定值，[0-1]