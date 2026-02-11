## 消息header
std_msgs/Header header

## 无人机PX4基本信息
uint8 uav_id                        ## 无人机编号
bool connected                      ## 是否与飞控建立连接
bool armed                          ## 是否解锁，true为已解锁，false则不是

uint8 control_mode                  ## 无人机控制模式
uint8 move_mode                     ## 无人机移动模式
 
float32 battery_state               ## 电池电压，单位：[V]
float32 battery_percentage          ## 电池百分比，[0-1]

bool odom_valid                     ## 里程计是否有效

## 无人机位姿
geometry_msgs/Point position        ## 当前位置，单位：[m]
geometry_msgs/Point velocity        ## 当前速度，单位：[m/s]
geometry_msgs/Quaternion attitude   ## 当前姿态，四元数
geometry_msgs/Point acceleration    ## 当前加速度，单位：[m/s^2]
float32 yaw                         ## 当前偏航角，单位：[rad]
float32 yaw_rate                    ## 当前偏航角角速率，单位：[rad/s]

## 无人机目标设定值 
geometry_msgs/Point position_setpoint             ## 位置设定值，单位：[m]  
geometry_msgs/Point velocity_setpoint             ## 速度设定值，单位：[m/s] 
geometry_msgs/Quaternion attitude_setpoint        ## 姿态设定值，四元数
geometry_msgs/Point acceleration_setpoint         ## 加速度设定值，单位：[m/s^2]
float32 yaw_setpoint                              ## 偏航角设定值，单位：[rad]
float32 yaw_rate_setpoint                         ## 偏航角角速率设定值，单位：[rad/s]






