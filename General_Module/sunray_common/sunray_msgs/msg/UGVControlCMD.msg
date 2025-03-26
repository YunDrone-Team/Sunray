std_msgs/Header header

## 控制命令的编号
uint32 cmd_id

## 控制命令
uint8 cmd
## cmd 枚举
uint8 INIT=0
uint8 HOLD=1
uint8 POS_CONTROL=2
## 以下需要有定位的情况下才支持
uint8 POS_CONTROL_BODY=3
uint8 VEL_CONTROL_BODY=4
uint8 VEL_CONTROL_ENU=5
uint8 Point_Control_with_Astar=6

## 偏航角类型 0 角速度 1 角度（需要有定位支持）
uint8 yaw_type

## 控制命令期望值
float64[2] desired_pos          ## [m]
float64[2] desired_vel          ## [m/s]
float64 desired_yaw             ## [rad]
float64 angular_vel             ## [rad/s]

