## 自定义消息：无人机控制指令

## 消息header
std_msgs/Header header

## 控制命令
uint8 cmd

## 注意：以下控制类型随PX4版本同、机型不同等，可能存在差异，请根据实际使用情况调整
## 控制命令枚举
uint8 XyzPos = 1                # 惯性系定点控制（不带偏航角），需要配合desired_pos使用
uint8 XyzVel = 2                # XYZ速度 不带偏航角, 需要配合desired_vel使用
uint8 XyVelZPos = 3             # XY速度 Z位置 不带偏航角, 需要配合desired_vel、desired_pos[2]使用
uint8 XyzPosYaw = 4             # XYZ位置 带偏航角, 需要配合desired_pos和desired_yaw使用
uint8 XyzPosYawrate = 5         # XYZ位置 带偏航角速率, 需要配合desired_pos和desired_yaw_rate使用
uint8 XyzVelYaw = 6             # XYZ速度 带偏航角, 需要配合desired_vel和desired_yaw使用
uint8 XyzVelYawrate = 7         # XYZ速度 带偏航角速率, 需要配合desired_vel和desired_yaw_rate使用
uint8 XyVelZPosYaw = 8          # XY速度 Z位置 带偏航角，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw使用
uint8 XyVelZPosYawrate = 9      # XY速度 Z位置 带偏航角速率，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw_rate使用
uint8 XyzPosVelYaw = 10         # XYZ位置速度 带偏航角，需要配合desired_pos、desired_vel和desired_yaw使用
uint8 XyzPosVelYawrate = 11     # XYZ位置速度 带偏航角速率，需要配合desired_pos、desired_vel和desired_yaw_rate使用
uint8 PosVelAccYaw = 12         # XYZ位置速度加速度 带偏航角，需要配合desired_pos、desired_vel、desired_acc和desired_yaw使用
uint8 PosVelAccYawrate = 13     # XYZ位置速度加速度 带偏航角速率，需要配合desired_pos、desired_vel、desired_acc和desired_yaw_rate使用
uint8 XyzPosYawBody = 14        # XYZ位置 带偏航角 机体坐标系，需要配合desired_pos和desired_yaw使用
uint8 XyzVelYawBody = 15        # XYZ速度 带偏航角 机体坐标系，需要配合desired_vel和desired_yaw使用
uint8 XyVelZPosYawBody = 16     # XY速度 Z位置 带偏航角 机体坐标系，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw使用
uint8 GlobalPos = 17            # 全局坐标 经纬度海拔，需要配合latitude、longitude、altitude使用

uint8 Point = 30                # 规划点

uint8 Takeoff = 100             # 起飞，home点上方悬停
uint8 Land = 101                # 原地降落
uint8 Hover = 102               # 当前位置悬停
uint8 Waypoint = 103            # 航点
uint8 Return = 104              # 返航

## 控制命令期望值
float32[3] desired_pos          ## 期望位置，单位：[m]
float32[3] desired_vel          ## 期望速度，单位：[m/s]
float32[3] desired_acc          ## 期望加速度，单位：[m/s^2]
float32[3] desired_att          ## 期望姿态，单位：[rad]
float32 desired_thrust          ## 期望推力，[0-1]
float32 desired_yaw             ## 期望偏航角，单位：[rad]
float32 desired_yaw_rate        ## 期望偏航角角速率，单位：[rad/s]
float32 latitude                ## 无人机期望经度                 
float32 longitude               ## 无人机期望纬度
float32 altitude                ## 无人机期望高度