## 自定义消息：无人车控制指令

## 消息header
std_msgs/Header header

## 控制命令
uint8 cmd

## 控制命令枚举
uint8 INIT=0                        ## 初始化，不做任何控制
uint8 HOLD=1                        ## 当前点停止
uint8 POS_CONTROL_ENU=2             ## 惯性系位置控制, 需要配合desired_pos和desired_yaw使用       
uint8 VEL_CONTROL_ENU=3             ## 惯性系速度控制, 需要配合desired_vel和desired_yaw使用
uint8 VEL_CONTROL_BODY=4            ## 车体系速度控制, 需要配合desired_vel和desired_yaw使用
uint8 Point_Control_with_Astar=5    ## Astar避障控制, 需要配合desired_pos和desired_yaw使用  
uint8 POS_VEL_CONTROL_ENU= 6

## 偏航角类型 0 角速度 1 角度（需要有定位支持）
uint8 yaw_type

## 控制命令期望值
float32[2] desired_pos          ## [m]
float32[2] desired_vel          ## [m/s]
float32 desired_yaw             ## [rad]
float32 angular_vel             ## [rad/s]

