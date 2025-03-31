## 自定义消息：外部定位数据的原始值

## 消息header
std_msgs/Header header

## 外部定位数据来源
uint8 external_source
## 外部定位数据来源枚举
uint8 ODOM=0
uint8 POSE=1
uint8 GAZEBO=2
uint8 MOCAP=3
uint8 VIOBOT = 4
uint8 GPS = 5
uint8 RTK = 6
uint8 VINS = 7

## 外部定位数据来源
bool odom_valid                     ## 外部定位数据是否有效
float32[3] position                 ## 位置，单位：[m]
float32[3] velocity                 ## 速度，单位：[m/s]
float32[3] attitude                 ## 姿态，单位：[rad]
geometry_msgs/Quaternion attitude_q ## 姿态，四元数