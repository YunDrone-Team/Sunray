std_msgs/Header header

## 时间
float32 time_boot_ms                ## Timestamp (time since system boot).

## 无人机姿态（欧拉角、四元数）
float32[3] attitude                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数
## 无人机姿态角速度
float32[3] attitude_rate            ## [rad/s]




    
