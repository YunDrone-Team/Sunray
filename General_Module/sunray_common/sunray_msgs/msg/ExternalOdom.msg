std_msgs/Header header

## 无人机外部定位来源
uint8 external_source
## 定位来源枚举
uint8 ODOM=0
uint8 POSE=1
uint8 GAZEBO=2
uint8 MOCAP=3
uint8 VIOBOT = 4
uint8 GPS = 5
uint8 RTK = 6
uint8 VINS = 7

## 里程计状态
bool odom_valid                     ## 里程计是否有效
float32[3] position                 ## [m]
float32[3] velocity                 ## [m/s]
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数