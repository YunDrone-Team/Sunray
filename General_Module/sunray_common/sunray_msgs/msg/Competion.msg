std_msgs/Header header

## 启动区和中心区的大小
uint8 obstacle_r                  ## 启动区半径大小
uint8 central_a               ## 中心方块大小

## 障碍物的中心坐标
float32[2] obstacle_1       ## 第一个障碍物
float32[2] obstacle_2       ## 第二个障碍物
float32[2] obstacle_3       ## 第三个障碍物
float32[2] delivery         ## 投送区

## 框的位置
float32[2] large_box        ## 大框中心点位置
float32[2] small_box        ## 小框中心点位置