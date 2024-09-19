# sunray_tutorial

#### 介绍
思锐无人机仿真与实践开发平台----demo与教程模块

#### 运行环境
- ROS Noetic
- sunray_uav_control

#### 运行
```shell
# 运行
## 以下选择一个运行就行
## demo 内有相关的控制半径、速度等参数，请根据实际情况调整确保其在安全范围内 建议先在仿真环境中测试
## 这些demo需要运行在sunray_uav_control之后 也就是需要仿真或者真机先运行起来

1. 各种控制模式的demo 通过demo_id来选择 速度、位置控制等 具体参阅对应代码
roslaunch sunray_tutorial run_demo.launch demo_id:=1

2. 跟随模式demo
roslaunch sunray_tutorial follow_a_car.launch

3. 跟随标签demo
roslaunch sunray_tutorial follow_a_tag.launch
```

#### 参数
```shell
run_demo.launch中的每个demo需要无人机定位 位置环控制环的k_p等

keyboard_tagCar.launch 用于键盘控制随车模式下的小车移动

在有yaw控制的demo中（特别是body系下） 由于内部循环的频率较高很容易超调 导致无人机在来回摆动 所以需要将相关的k_p值调小

```
