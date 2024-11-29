std_msgs/Header header

float32[9] channel

# 模式下 0为无效 默认都分三档也就1 2 3 三个值
# arm_state = 2 表示要解锁 =1 表示要上锁
# mode_state = 1 表示切换到INIT模式 =2表示要切换到RC_CONTROL模式 =3表示要切换到CMD_CONTROL模式
# land_state = 1 表示要降落
# kill_state = 1 表示要紧急停止

uint8 arm_state
uint8 mode_state
uint8 land_state
uint8 kill_state