std_msgs/Header header

## 启动区和中心区的大小
float32 obstacle_r              ## 障碍物半径 单位：m
float32 central_a               ## 中心方块大小（边长）单位：m

## 障碍物的中心坐标
float32[2] obstacle_1       ## 第一个障碍物 单位：m
float32[2] obstacle_2       ## 第二个障碍物 单位：m
float32[2] obstacle_3       ## 第三个障碍物 单位：m
float32[2] delivery         ## 投送区 单位：m

## 框的位置
float32[2] large_box        ## 大框中心点位置 单位：m
float32[2] small_box        ## 小框中心点位置 单位：m