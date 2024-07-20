std_msgs/Header header

## 控制命令的编号
uint32 cmd_id

## 控制命令
uint8 cmd
## cmd 枚举
uint8 Hold=0
uint8 Direct_Control_BODY=1
uint8 Direct_Control_ENU=2
uint8 Point_Control=3
uint8 Point_Control_with_Astar=4
uint8 Test=5

## 控制命令期望值
float64[2] desired_pos          ## [m]
float64 desired_yaw             ## [rad]
float64[2] linear_vel       ## [m/s]
float64 angular_vel         ## [rad/s]

