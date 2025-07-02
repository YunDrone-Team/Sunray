std_msgs/Header header
# 航点数
uint8 wp_num
# 航点类型 NED 或 经纬
uint8 wp_type
# 结束事件 1: 悬停 2: 降落 3: 返航
uint8 wp_end_type
# 任务是否包含起飞过程
bool wp_takeoff
# 偏航角类型 1: 固定值 2: 朝向下一个航点 3: 指向环绕点
uint8 wp_yaw_type
# 航点间移动速度
float32 wp_move_vel
# 速度计算比例
float32 wp_vel_p
# 起飞或返航高度
float32 z_height
# 航点 xyzyaw 或 经纬高yaw
float64[4] wp_point_1
float64[4] wp_point_2
float64[4] wp_point_3
float64[4] wp_point_4
float64[4] wp_point_5
float64[4] wp_point_6
float64[4] wp_point_7
float64[4] wp_point_8
float64[4] wp_point_9
float64[4] wp_point_10
# 环绕点 xy 或 经纬
float64[2] wp_circle_point