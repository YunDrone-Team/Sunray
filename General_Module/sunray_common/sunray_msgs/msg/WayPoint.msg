## 自定义消息：无人机航点任务

## 消息header
std_msgs/Header header

## 开始执行指令
bool start

## 航点数
uint8 wp_num

## 航点结束执行: 1: 悬停 2: 降落 3: 返航
uint8 wp_end_type

# 偏航角类型 1: 固定值 2: 朝向下一个航点
uint8 wp_yaw_type

# 航点间移动速度
float32 wp_move_vel

# 航点 xyzyaw
sunray_msgs/Point[] wp_points


