std_msgs/Header header

bool vio_start                      ## viobot算法是否开启
string algo_status                  ## viobot算法状态
bool odom_valid                     ## 外部定位数据是否有效

## 里程计状态
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
