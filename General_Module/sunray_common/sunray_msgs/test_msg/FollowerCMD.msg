## 自定义消息：

## 消息header
std_msgs/Header header

## 阵型
uint8 mission_formation

## 阵型枚举
uint8 FORMATION1 = 1      
uint8 FORMATION2 = 2          
uint8 FORMATION3 = 3        

float64 leader_lat                ## 领队无人机期望经度                 
float64 leader_lon                ## 领队无人机期望纬度
float64 leader_alt                ## 领队无人机期望高度