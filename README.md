# Sunray

#### 介绍
思锐无人机仿真与实践开发平台

#### 软件架构
```shell
General_Module  --> 总控制模块
    sunray_common --> 自定义消息 依赖库等
    sunray_uav_control --> offboard控制模块
    vrpn_client_ros --> mocap vrpn动捕相关依赖
    sunray_tutorial --> demo教程相关功能
    sunray_ugv_planning --> 径规划相关功能
    sunray_ground --> 地面站相关功能
    sunray_wheeltec_robot --> 轮式机器人相关功能

Simulation --> 总仿真模块
    sunray_simulator --> sunray px4仿真
        launch_basic --> 启动仿真相关的launch文件
        launch --> 相关功能的launch文件

```
#### README.md


#### 安装教程
```shell
## 使用前提 安装px4 mavros 本px4仿真适用与px4 1.14及以上版本
## 以noetic版本为例
1.  安装依赖
    sudo apt-get install ros-noetic-vrpn
    sudo apt install ros-noetic-gazebo-plugins
    sudo apt-get install ros-noetic-velodyne-simulator

2.  git clone https://gitee.com/yundrone_sunray2023/Sunray.git

3.  编译
    # 编译sunray_common模块
    catkin_make --source General_Module/sunray_common --build build/sunray_common
    # 编译sunray_uav_control模块
    catkin_make --source General_Module/sunray_uav_control --build build/sunray_uav_control
    # 编译vrpn_client_ros功能包
    catkin_make --source General_Module/vrpn_client_ros --build build/vrpn_client_ros
    # 编译sunray_simulator模块
    catkin_make --source Simulation/sunray_simulator --build build/sunray_simulator
    # 编译sunray_planner模块
    catkin_make --source General_Module/sunray_planner --build build/sunray_planner
    # 编译sunray_ground模块
    catkin_make --source General_Module/sunray_ground --build build/sunray_ground
    # 编译sunray_tutorial模块
    catkin_make --source General_Module/sunray_tutorial --build build/sunray_tutorial
    # 编译ego-planner-swarm模块
    catkin_make --source ego-planner-swarm --build build/ego-planner
    # 编译FUEL模块
    catkin_make --source FUEL --build build/FUEL
    # 编译simulator_utils模块
    catkin_make --source Simulation/simulator_utils --build build/simulator_utils

4.  添加环境变量  path -->替换为实际路径
    source ~/Sunray/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Sunray/Simulation/sunray_simulator/models
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Sunray/Simulation/sunray_simulator/world_models
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/Sunray/Simulation/sunray_simulator/texture
```
#### 使用说明

##### 仿真使用示例
```shell
仿真时启动QGC 没有遥控器接入的情况下还需要把虚拟遥控打开 不然可能会无法解锁

## 启动ros
roscore
## 启动仿真
roslaunch sunray_simulator sunray_sim_1uav.launch
## 提供外部定位 (指定外部定位输入来源 external_source)
roslaunch sunray_uav_control vision_pose_node.launch external_source:=2
## 运行控制模块
roslaunch sunray_uav_control sunray_control.launch
## 运行终端控制 (可简单测试offboard下的起飞降落 移动等)
roslaunch sunray_uav_control terminal_control.launch 

```
##### 真机使用
```shell
## 启动ros
roscore
## 启动mavros
roslaunch sunray_uav_control sunray_mavros_exp.launch
## 提供外部定位 (指定外部定位输入来源 external_source)
roslaunch sunray_uav_control vision_pose_node.launch external_source:=
## 运行控制模块
roslaunch sunray_uav_control sunray_control.launch
## 运行终端控制 (可简单测试offboard下的起飞降落 移动等)
roslaunch sunray_uav_control terminal_control.launch 

```

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


