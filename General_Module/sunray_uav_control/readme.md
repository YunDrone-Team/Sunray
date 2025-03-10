# sunray_uav_control

#### 介绍
思锐无人机仿真与实践开发平台----控制模块

#### 运行环境
- ROS Noetic
- sunray_uav_control

#### launch文件说明
```shell
joynode.launch              --> 启动一个节点，用于接收来自joystick的输入 在仿真中接入遥控器使用
rc_control.launch           --> 启动一个节点，用于接收来自遥控器的输入
sunray_control_node.launch  --> 启动offboard的控制节点 所有控制过程都在这个节点中实现
sunray_mavros_exp.launch    --> 启动mavros节点，用于与真机飞控通信
external_fusion.launch     --> 外部定位节点，用于接收外部定位数据发送到飞控作为定位数据
terminal_control.launch     --> 终端控制 可以根据键盘输入控制无人机
waypoint.launch             --> 航点发布节点，用于发布航点
```

#### sunray_uav_control控制模式说明
```shell
INIT：          初始模式 会进入定点模式
RC_CONTROL:     遥控器控制 通过定阅遥控器输入做位置控制
CMD_CONTROL:    指令控制模式 通过定阅话题获取对应的控制方式和值
    Takeoff：           起飞
    Hover：             悬停
    Land：              降落
    Waypoint：          航点
    XyzPos              XYZ位置控制
    XyzVel              XYZ速度控制
    XyVelZPos           XY速度Z位置控制
    XyzPosYawBody       XYZ位置控制（机体坐标系）
    XyzVelYawBody       XYZ速度控制（机体坐标系）
    XyVelZPosYawBody    XY速度Z位置控制（机体坐标系）
    ....
LAND_CONTROL：  降落模式
```