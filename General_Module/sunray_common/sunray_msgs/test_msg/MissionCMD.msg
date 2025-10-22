## 自定义消息：

## 消息header
std_msgs/Header header

## 任务命令
uint8 mission

## 任务命令枚举
uint8 TAKEOFF = 1       # 起飞
uint8 LAND = 2          # 原地降落
uint8 RETURN = 3        # 返回原点
uint8 MOVE = 4          # 移动

## 阵型
uint8 mission_formation

## 阵型枚举
uint8 FORMATION1 = 1      
uint8 FORMATION2 = 2          
uint8 FORMATION3 = 3        

float64 leader_lat_ref                ## 领队无人机期望经度                 
float64 leader_lon_ref                ## 领队无人机期望纬度
float64 leader_alt_ref                ## 领队无人机期望高度

float64 origin_lat                ## 原点经度                 
float64 origin_lon                ## 原点纬度
float64 origin_alt                ## 原点高度