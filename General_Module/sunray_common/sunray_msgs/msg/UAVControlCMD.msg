## 自定义消息：无人机控制指令

## 消息header
std_msgs/Header header

## 控制命令
uint8 cmd

## 注意：以下控制类型随PX4版本同、机型不同等，可能存在差异，请根据实际使用情况调整
## 控制命令枚举

## 控制命令枚举 - 基础移动模式
uint8 XyzPos = 1                # 移动指令：惯性系定点控制 保持当前偏航角，需要配合desired_pos使用
uint8 XyzPosYaw = 4             # 移动指令：惯性系定点控制 带偏航角, 需要配合desired_pos和desired_yaw使用
uint8 XyzPosYawrate = 5         # 移动指令：惯性系定点控制 带偏航角速率, 需要配合desired_pos和desired_yaw_rate使用

uint8 XyzVel = 2                # 移动指令：惯性系XYZ速度控制 保持当前偏航角, 需要配合desired_vel使用
uint8 XyzVelYaw = 6             # 移动指令：惯性系XYZ速度控制 带偏航角, 需要配合desired_vel和desired_yaw使用
uint8 XyzVelYawrate = 7         # 移动指令：惯性系XYZ速度控制 带偏航角速率, 需要配合desired_vel和desired_yaw_rate使用

uint8 XyVelZPos = 3             # 移动指令：惯性系XY速度 Z位置 保持当前偏航角, 需要配合desired_vel、desired_pos[2]使用
uint8 XyVelZPosYaw = 8          # 移动指令：惯性系XY速度 Z位置 带偏航角，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw使用
uint8 XyVelZPosYawrate = 9      # 移动指令：惯性系XY速度 Z位置 带偏航角速率，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw_rate使用

uint8 XyzPosVelYaw = 10         # 移动指令：XYZ位置速度 带偏航角，需要配合desired_pos、desired_vel和desired_yaw使用
uint8 XyzPosVelYawrate = 11     # 移动指令：XYZ位置速度 带偏航角速率，需要配合desired_pos、desired_vel和desired_yaw_rate使用

uint8 PosVelAccYaw = 12         # 移动指令：XYZ位置速度加速度 带偏航角，需要配合desired_pos、desired_vel、desired_acc和desired_yaw使用
uint8 PosVelAccYawrate = 13     # 移动指令：XYZ位置速度加速度 带偏航角速率，需要配合desired_pos、desired_vel、desired_acc和desired_yaw_rate使用

uint8 XyzPosYawBody = 14        # 移动指令：机体坐标系XYZ位置 带偏航角，需要配合desired_pos和desired_yaw使用
uint8 XyzVelYawBody = 15        # 移动指令：机体坐标系XYZ速度 带偏航角，需要配合desired_vel和desired_yaw使用
uint8 XyVelZPosYawBody = 16     # 移动指令：机体坐标系XY速度 Z位置 带偏航角，需要配合desired_vel[0]、desired_vel[1]、desired_pos[2]和desired_yaw使用
uint8 GlobalPos = 17            # 移动指令：全局坐标 经纬度海拔，需要配合latitude、longitude、altitude使用

## 控制命令枚举 - 使用自定义位置控制器进行控制（默认发送期望姿态至PX4飞控）
uint8 CTRL_XyzPos = 50          # 移动指令：惯性系定点控制 带偏航角, 需要配合desired_pos和desired_yaw使用
uint8 CTRL_Traj = 51            # 移动指令：轨迹追踪 需要配合desired_pos、desired_vel、desired_acc和desired_yaw使用

## 控制命令枚举 - 特殊指令（高级模式）
uint8 Point = 30                # 特殊指令：路径规划的目标点
uint8 Takeoff = 100             # 特殊指令：起飞，home点上方悬停
uint8 Land = 101                # 特殊指令：原地降落
uint8 Hover = 102               # 特殊指令：当前位置悬停
uint8 Waypoint = 103            # 特殊指令：航点
uint8 Return = 104              # 特殊指令：返航

## 控制命令期望值
float32[3] desired_pos          ## 期望位置，单位：[m]
float32[3] desired_vel          ## 期望速度，单位：[m/s]
float32[3] desired_acc          ## 期望加速度，单位：[m/s^2]
float32[3] desired_jerk         ## 期望加加速度，单位：[m/s^2]
float32[3] desired_att          ## 期望姿态，单位：[rad]
float32 desired_thrust          ## 期望推力，[0-1]
float32 desired_yaw             ## 期望偏航角，单位：[rad]
float32 desired_yaw_rate        ## 期望偏航角角速率，单位：[rad/s]
float32 latitude                ## 无人机期望经度                 
float32 longitude               ## 无人机期望纬度
float32 altitude                ## 无人机期望高度