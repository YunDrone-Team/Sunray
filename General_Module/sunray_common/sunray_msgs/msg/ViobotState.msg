std_msgs/Header header

## 基本状态
uint8 uav_id                ## 无人机编号
bool vio_start              ## 是否与飞控建立连接

## 里程计状态
bool odom_valid                     ## 里程计是否有效
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数





    
