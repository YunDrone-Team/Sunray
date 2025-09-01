## 自定义消息：外部定位数据的原始值

## 消息header
std_msgs/Header header

## 外部定位数据来源
uint8 external_source

## 外部定位数据来源枚举
uint8 ODOM = 0                  ## nav_msgs::Odometry类型的话题
uint8 POSE = 1                  ## geometry_msgs::PoseStamped类型的话题
uint8 GAZEBO = 2                ## GAZEBO仿真时使用Gazebo插件提供的位姿真值
uint8 MOCAP = 3                 ## 动作捕捉系统
uint8 VIOBOT = 4                ## VIOBOT
uint8 GPS = 5                   ## GPS
uint8 RTK = 6                   ## RTK
uint8 VINS = 7                  ## VINS 

## 外部定位数据状态
bool odom_valid                     ## 外部定位数据是否有效（根据订阅数据间隔时间判断）
bool fusion_success                 ## 外部定位数据与飞控融合是否成功
float32[3] position                 ## 位置，单位：[m]
float32[3] velocity                 ## 速度，单位：[m/s]
float32[3] attitude                 ## 姿态，单位：[rad]
geometry_msgs/Quaternion attitude_q ## 姿态，四元数

bool vio_start                      ## viobot算法是否开启
string algo_status                  ## viobot算法状态