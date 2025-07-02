## 自定义消息：无人机设置指令

## 消息header
std_msgs/Header header

## 通道数值
float32[9] channel

# 模式下 0为无效 默认都分三档也就1 2 3 三个值

## 上/解锁指令
uint8 arm_state
## arm_state类型枚举
uint8 DISARM = 1                    ## 要上锁
uint8 ARM = 2                       ## 要解锁

## 切换无人机控制模式指令
uint8 mode_state
## mode_state类型枚举
uint8 INIT = 1                      ## 切换到 INIT
uint8 RC_CONTROL = 2                ## 切换到 RC_CONTROL
uint8 CMD_CONTROL = 3               ## 切换到 CMD_CONTROL

# land_state = 1 表示要降落
uint8 land_state

# kill_state = 1 表示要紧急停止
uint8 kill_state