std_msgs/Header header

## 以下参数单位统一 单位：m

float32 central_a           ## 中心方块大小（边长）
float32[2] delivery         ## 投送区中心点坐标 

## 障碍物的中心点坐标和半径
float32[2] obstacle_1       ## 第一个障碍物 
float32[2] obstacle_2       ## 第二个障碍物 
float32[2] obstacle_3       ## 第三个障碍物 
float32 obstacle_r            ## 障碍物半径

## 框的位置
float32[2] large_box        ## 大框中心点位置 
float32[2] small_box        ## 小框中心点位置 