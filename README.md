# Sunray

#### 介绍
思锐无人机仿真与实践开发平台

#### 软件架构
```shell
General_Module  --> 总控制模块
    sunray_common --> 自定义消息 依赖库等
    sunray_uav_control --> px4控制模块
    vrpn_client_ros --> mocap vrpn

Simulation --> 总仿真模块
    sunray_simulator --> sunray px4仿真
```
#### 安装教程
```shell
## 使用前提 安装px4 mavros 本px4仿真适用与px4 1.14及以上版本
## 以noetic版本为例
1.  sudo apt-get install ros-noetic-vrpn
2.  git clone https://gitee.com/yundrone_sunray2023/Sunray.git
3.  git checkout uestc
4.  编译
    # 编译sunray_common模块
    catkin_make --source General_Module/sunray_common --build build/sunray_common
    # 编译sunray_uav_control模块
    catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
    # 编译sunray_simulator模块
    catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
    ## 编译vrpn_client_ros功能包
    catkin_make --source General_Module/vrpn_client_ros --build build/vrpn_client_ros
5.  添加环境变量  path -->替换为实际路径
    echo "source path/Sunray/devel/setup.bash" >> ~/.bashrc
```
#### 使用说明

##### 仿真使用
```shell
仿真时启动QGC 不然可能会无法解锁

## 启动ros
roscore
## 启动仿真
roslaunch sunray_simulator sunray_sim_1uav.launch
## 提供外部定位 (指定外部定位输入来源 external_source)
roslaunch sunray_uav_control vision_pose.launch
## 运行控制模块
roslaunch sunray_uav_control sunray_control.launch
## 运行终端控制 (可简单测试offboard下的起飞降落 移动等)
roslaunch sunray_uav_control terminal_control.launch 

## 控制接口demo
General_Module/sunray_tutorial/src 下有多个控制方式可运行的demo 

```
##### 真机使用
```shell
## 启动ros
roscore
## 启动mavros
roslaunch sunray_uav_control sunray_mavros_exp.launch
## 提供外部定位 (指定外部定位输入来源 external_source)
roslaunch sunray_uav_control vision_pose.launch
## 运行控制模块
roslaunch sunray_uav_control sunray_control.launch
## 运行终端控制 (可简单测试offboard下的起飞降落 移动等)
roslaunch sunray_uav_control terminal_control.launch 

```
##### tutorial模块
```shell
## 该模块为demo模块
roslaunch sunray_tutorial run_demo.launch demo_id:=
## demo_id需要指定代表测试的脚本 具体功能见run_demo.launch
## 改模块为一键启动脚本 会自己解锁起飞降落等请注意安全 
# 前提运行
## 提供外部定位 (指定外部定位输入来源 external_source)
roslaunch sunray_uav_control vision_pose.launch
## 运行控制模块
roslaunch sunray_uav_control sunray_control.launch
```

##### 进阶用法
```shell
需要搭配遥控器使用
更多控制参阅 uav_control.cpp

```
#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


