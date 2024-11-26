std_msgs/Header header

## 控制命令
uint8 cmd
## cmd 枚举

## 控制命令期望值
float32[3] desired_pos          ## [m]
float32[3] desired_vel          ## [m/s]
float32[3] desired_acc          ## [m/s^2]
float32[3] desired_att          ## [rad]
float32 desired_thrust          ## [0-1]
float32 desired_yaw             ## [rad]
float32 desired_yaw_rate        ## [rad/s]
bool enable_yawRate
float32 latitude                ## 无人机经度、纬度、高度                 
float32 longitude               ## 无人机经度、纬度、高度
float32 altitude                ## 无人机经度、纬度、高度

