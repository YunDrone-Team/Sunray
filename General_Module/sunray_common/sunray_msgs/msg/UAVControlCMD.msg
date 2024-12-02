std_msgs/Header header

## 控制命令
uint8 cmd
## cmd 枚举
uint8 XyzPos = 1                # XYZ位置 不带偏航角
uint8 XyzVel = 2                # XYZ速度 不带偏航角
uint8 XyVelZPos = 3             # XY速度 Z位置 不带偏航角
uint8 XyzPosYaw = 4             # XYZ位置 带偏航角
uint8 XyzPosYawrate = 5         # XYZ位置 带偏航角速率
uint8 XyzVelYaw = 6             # XYZ速度 带偏航角
uint8 XyzVelYawrate = 7         # XYZ速度 带偏航角速率
uint8 XyVelZPosYaw = 8          # XY速度 Z位置 带偏航角
uint8 XyVelZPosYawrate = 9      # XY速度 Z位置 带偏航角速率
uint8 XyzPosVelYaw = 10         # XYZ位置速度 带偏航角
uint8 XyzPosVelYawrate = 11     # XYZ位置速度 带偏航角速率
uint8 PosVelAccYaw = 12         # XYZ位置速度加速度 带偏航角
uint8 PosVelAccYawrate = 13     # XYZ位置速度加速度 带偏航角速率
uint8 XyzPosYawBody = 14        # XYZ位置 带偏航角 机体坐标系
uint8 XyzVelYawBody = 15        # XYZ速度 带偏航角 机体坐标系
uint8 XyVelZPosYawBody = 16     # XY速度 Z位置 带偏航角 机体坐标系
uint8 GlobalPos = 17            # 全局坐标 经纬度海拔

uint8 Takeoff = 100             # 起飞
uint8 Land = 101                # 降落
uint8 Hover = 102               # 悬停
uint8 Waypoint = 103            # 航点
uint8 Return = 104              # 返航


## 控制命令期望值
float32[3] desired_pos          ## [m]
float32[3] desired_vel          ## [m/s]
float32[3] desired_acc          ## [m/s^2]
float32[3] desired_att          ## [rad]
float32 desired_thrust          ## [0-1]
float32 desired_yaw             ## [rad]
float32 desired_yaw_rate        ## [rad/s]
float32 latitude                ## 无人机经度、纬度、高度                 
float32 longitude               ## 无人机经度、纬度、高度
float32 altitude                ## 无人机经度、纬度、高度