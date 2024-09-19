# sunray_uav_control

#### 介绍
思锐无人机仿真与实践开发平台----控制模块

#### 运行环境
- ROS Noetic
- sunray_uav_control

#### launch文件说明
```shell
joynode.launch              --> 启动一个节点，用于接收来自joystick的输入 在仿真中接入遥控器使用
sunray_control_node.launch  --> 启动offboard的控制节点 所有控制过程都在这个节点中实现
sunray_mavros_exp.launch    --> 启动mavros节点，用于与真机飞控通信
vision_pose_node.launch     --> 外部定位节点，用于接收外部定位数据发送到飞控作为定位数据
run_all.launch              --> 内部嵌套sunray_control_node、vision_pose_node、show_detail等节点，可以在一个终端启动多个节点
terminal_control.launch     --> 终端控制 可以根据键盘输入控制无人机
```

#### sunray_uav_control控制模式说明
```shell
INIT：          初始模式 会进入定点模式
RC_CONTROL:     遥控器控制 通过定阅遥控器输入做位置控制
CMD_CONTROL:    指令控制模式 通过定阅话题获取对应的控制方式和值
    Takeoff：           起飞
    Hover：             悬停
    Land：              降落
    XYZ_POS：           XYZ位置控制
    XYZ_VEL：           XYZ速度控制
    XY_VEL_Z_POS：      XY速度Z位置控制
    XYZ_POS_BODY：      XYZ位置控制（机体坐标系）
    XYZ_VEL_BODY：      XYZ速度控制（机体坐标系）
    XY_VEL_Z_POS_BODY： XY速度Z位置控制（机体坐标系）
    TRAJECTORY：        轨迹控制（点控制）
LAND_CONTROL：  降落模式单独执行会中断其控制子模式
```


#### 重要参数
```shell
uav_id: 无人机编号
uav_name: 无人机名称
geo_fence: 地理围栏
Land_speed: 降落速度
Disarm_height: 上锁高度
Takeoff_height： 当前高度+Takeoff_height = 起飞高度
sim_mode: 是否是仿真模式
use_rc: 是否使用遥控器  sunray_control内置了offboard下的RC_CONTROL模式，如果use_rc=true 则需要修改通道对齐 否则可能会导致控制异常
external_source: 外部定位的来源
enable_rangeSensor： 是否使用激光定高等作为vins_pose的高度来源 不同的传感器要区修改对应的话题名
```