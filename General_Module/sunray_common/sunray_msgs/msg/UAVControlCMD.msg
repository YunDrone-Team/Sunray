std_msgs/Header header

## 控制命令的编号
uint32 cmd_id

## 控制命令
uint8 cmd
## cmd 枚举
uint8 Takeoff = 1               ## home点上方悬停
uint8 Hover = 2                 ## 当前位置上方悬停
uint8 Land = 3                  ## 原地降落
uint8 XYZ_POS = 4               ## 惯性系定点控制
uint8 XY_VEL_Z_POS = 5          ## 惯性系定高速度控制
uint8 XYZ_VEL = 6               ## 惯性系速度控制
uint8 XYZ_POS_BODY = 7          ## 机体系位置控制
uint8 XYZ_VEL_BODY = 8          ## 机体系速度控制
uint8 XY_VEL_Z_POS_BODY = 9     ## 机体系定高速度控制
uint8 TRAJECTORY = 10           ## 轨迹追踪控制
uint8 XYZ_ATT = 11              ## 姿态控制（来自外部控制器）
uint8 LAT_LON_ALT = 12          ## 绝对坐标系下的经纬度

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

