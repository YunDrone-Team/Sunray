std_msgs/Header header

## 基本状态
uint8 uav_id                ## 无人机编号
bool connected              ## 是否与飞控建立连接
bool armed                  ## 是否解锁，true为已解锁，false则不是
string mode                 ## 飞控当前飞行模式

## 里程计状态
uint8 location_source               ## 无人机定位来源
bool odom_valid                     ## 里程计是否有效
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
float32[3] attitude_rate            ## [rad/s]

float32[3] pos_setpoint             ## [m]  位置控制目标点
float32[3] vel_setpoint             ## [m/s]  速度控制目标
float32[3] att_setpoint             ## [rad]  姿态控制目标

## 无人机电池状态
float32 battery_state               ## [V]
float32 battery_percetage           ## [0-1]

uint8 control_mode


    
