<!-- title: ROS 运行链路与消息接口 -->

<section id="ros-graph">

## 运行链路

Sunray 的上层开发链路可以简化成下面这张图。你自己的算法大多数时候应该放在“任务节点/算法节点”位置。

<div class="flow">
<span>任务节点</span>
<span>`UAVControlCMD` / `UGVControlCMD`</span>
<span>Sunray 控制节点</span>
<span>MAVROS 或底盘驱动</span>
<span>PX4 或 UGV</span>
</div>

状态反方向回来：

<div class="flow">
<span>PX4/MAVROS/定位源</span>
<span>`external_fusion_node`</span>
<span>`PX4State`</span>
<span>`uav_control_node`</span>
<span>`UAVState`</span>
<span>任务节点</span>
</div>

### 核心节点

| 节点 | 文件 | 作用 |
| --- | --- | --- |
| `external_fusion_node` | `sunray_uav_control/externalFusion` | 订阅 MAVROS 和外部定位，发布 `//sunray/px4_state`、`uav_odom`、RViz 轨迹和 mesh。 |
| `uav_control_node` | `sunray_uav_control/uav_control` | 订阅 `UAVControlCMD`/`UAVSetup`，管理 INIT、RC_CONTROL、CMD_CONTROL、LAND_CONTROL 状态机，向 MAVROS 发布 setpoint 或调用服务。 |
| `ugv_control_node` | `sunray_ugv_control/ugv_control` | 订阅 `UGVControlCMD`，读取 odom/动捕/viobot/UWB，输出 `cmd_vel`，发布 `UGVState`。 |
| `positionCmd2sunray` | `sunray_planner_utils/src/positionCmd2sunray.cpp` | 把规划器输出的 `sunray_msgs/PositionCommand` 转成 Sunray UAV 或 UGV 控制话题。 |

</section>

<section id="messages">

## 消息接口

Sunray 的二次开发核心在 `General_Module/sunray_common/sunray_msgs/msg`。你不需要一开始全背下来，但要熟悉下面几个。

### UAV 控制：`UAVControlCMD`

话题通常是 `//sunray/uav_control_cmd`，例如 `/uav1/sunray/uav_control_cmd`。

| 字段/枚举 | 含义 | 常见用法 |
| --- | --- | --- |
| `cmd = Takeoff` | 起飞到控制节点参数中的起飞高度。 | 任务开始。 |
| `cmd = Land` | 进入降落控制。 | 任务结束、异常处理。 |
| `cmd = Hover` | 当前位置悬停。 | 等待、暂停、超时保护。 |
| `cmd = XyzPos` | 惯性系位置控制，保持当前 yaw。 | 方形、航点、定点飞行。 |
| `cmd = XyzPosYaw` | 惯性系位置 + yaw。 | 要求朝向目标或固定朝向。 |
| `cmd = XyzVel` | 惯性系速度控制。 | 连续运动、搜索、避障。 |
| `cmd = XyVelZPosYawBody` | 机体系 XY 速度 + Z 高度 + yaw。 | 跟车、跟二维码、相对目标控制。 |
| `cmd = GlobalPos` | 经纬度/高度控制。 | GPS/RTK 场景。 |
| `cmd = Point` | 发布规划目标点，不直接控制 PX4。 | 给规划器/RViz goal bridge 用。 |
| `cmd = CTRL_Traj` | 使用自定义姿态控制器追踪位置、速度、加速度、yaw。 | 轨迹跟踪实验。 |

### UAV 设置：`UAVSetup`

话题通常是 `/uav1/sunray/setup`。它负责解锁、上锁、切 PX4 模式、切 Sunray 控制状态机。

```python
setup.cmd = UAVSetup.SET_CONTROL_MODE
setup.control_mode = "CMD_CONTROL"
setup_pub.publish(setup)

setup.cmd = UAVSetup.ARM
setup_pub.publish(setup)
```

### UAV 状态：`UAVState`

任务节点一般订阅 `/uav1/sunray/uav_state`，至少关注：

- `connected`：是否和飞控/MAVROS 链路连接。
- `armed`：是否解锁。
- `mode`：PX4 当前飞行模式，如 `OFFBOARD`、`POSCTL`。
- `control_mode`：Sunray 控制状态机，`2` 通常表示 `CMD_CONTROL`。
- `odom_valid`：定位是否有效。
- `position`、`velocity`、`attitude`：当前位置、速度、姿态。
- `home_pos`、`hover_pos`、`takeoff_height`：起飞点、悬停点和起飞高度。

### UGV 控制和状态

无人车使用 `UGVControlCMD` 和 `UGVState`，典型话题是 `/ugv1/sunray_ugv/ugv_control_cmd` 和 `/ugv1/sunray_ugv/ugv_state`。

| UGVControlCMD | 含义 |
| --- | --- |
| `INIT` | 初始化，不控制。 |
| `HOLD` | 原地停止，发布零速度。 |
| `POS_CONTROL_ENU` | 惯性系位置控制，内部 P 控制转 `cmd_vel`。 |
| `VEL_CONTROL_ENU` | 惯性系速度控制，控制节点转车体系速度。 |
| `VEL_CONTROL_BODY` | 车体系速度控制。 |
| `Point_Control_with_Astar` | 使用控制节点内置 A* 走向目标点。 |
| `POS_VEL_CONTROL_ENU` | 位置加速度前馈式控制。 |

</section>

<section id="uav">

## 无人机二次开发

无人机开发的关键包是 `General_Module/sunray_uav_control` 和 `General_Module/sunray_tutorial`。建议新手把 `uav_control_node` 当成稳定服务：它负责安全检查、模式切换、MAVROS setpoint、降落逻辑；你的任务节点只负责发目标和读状态。

### 最小控制链路

```bash
# 仿真常见链路，具体仿真包需确保已安装
roscore
roslaunch sunray_simulator sunray_sim_1uav.launch
roslaunch sunray_uav_control external_fusion.launch external_source:=2
roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1
roslaunch sunray_tutorial run_demo.launch demo_id:=1 uav_id:=1
```

### `external_source` 怎么选

| 值 | 来源 | 典型场景 |
| --- | --- | --- |
| `0` | `nav_msgs/Odometry` | VINS、FAST-LIO 或自定义 odom。 |
| `1` | `geometry_msgs/PoseStamped` | 只提供 pose 的定位源。 |
| `2` | Gazebo 仿真真值 | 仿真。 |
| `3` | 动捕 VRPN | 室内动捕场地。 |
| `4` | VIOBOT | VIOBOT 视觉惯导定位。 |
| `5` | GPS | 室外 GPS 模式，不走外部定位融合。 |
| `6` | RTK | RTK，经纬高转换 ENU。 |
| `7` | VINS | 代码枚举中存在，当前 launch 注释提示也可用 ODOM 入口。 |

### UAV 控制节点内部状态

`uav_control_node` 的 `UAVControl::mainLoop()` 按状态机运行：

- `INIT`：初始模式，未交给外部任务控制。
- `RC_CONTROL`：遥控器控制，由 Sunray 转成 OFFBOARD 控制。
- `CMD_CONTROL`：任务节点控制，二次开发主要使用这个模式。
- `LAND_CONTROL`：降落控制。
- `WITHOUT_CONTROL`：不控制。

进入 `CMD_CONTROL` 后，控制节点会根据 `UAVControlCMD.cmd` 把指令转成 MAVROS 的 `setpoint_raw/local`、`setpoint_raw/global` 或 `setpoint_raw/attitude`。

<div class="danger">
<strong>不要绕开控制节点直接打 MAVROS 指令</strong>
            除非你明确知道自己在做底层控制实验。直接发布 MAVROS setpoint 会绕过 Sunray 的围栏、定位检查、降落状态机和统一状态发布，新手实机风险很高。
          </div>

### 最推荐读的示例

| 示例 | 展示内容 | 适合改成 |
| --- | --- | --- |
| `uav_basic/takeoff_hover_land.cpp` | 起飞、悬停、降落完整流程。 | 任何 UAV 任务模板。 |
| `uav_basic/block_xyzpos.cpp` | `XyzPos` 依次飞多个点。 | 航点、巡逻、投放点移动。 |
| `uav_basic/circle_xyvelzpos.cpp` | 速度控制 + 高度保持。 | 环绕、搜索。 |
| `uav_basic/hexagon_xyzposyawbody.cpp` | 机体系目标/偏航控制。 | 相对运动。 |
| `uav_python/takeoff_hover_land.py` | Python 版最小流程。 | ROS Python 学生项目。 |
| `advanced/auto_land_by_pose.cpp` | 视觉目标引导降落。 | 二维码/ArUco 降落。 |

### Python 最小控制模板

```python
#!/usr/bin/env python3
import rospy
from sunray_msgs.msg import UAVState, UAVSetup, UAVControlCMD

uav_state = UAVState()

def state_cb(msg):
    global uav_state
    uav_state = msg

rospy.init_node("my_uav_task")
uav_id = rospy.get_param("~uav_id", 1)
prefix = "/uav{}".format(uav_id)

state_sub = rospy.Subscriber(prefix + "/sunray/uav_state", UAVState, state_cb)
cmd_pub = rospy.Publisher(prefix + "/sunray/uav_control_cmd", UAVControlCMD, queue_size=1)
setup_pub = rospy.Publisher(prefix + "/sunray/setup", UAVSetup, queue_size=1)

rate = rospy.Rate(20)

# 等连接
while not rospy.is_shutdown() and not uav_state.connected:
    rate.sleep()

# 进入 CMD_CONTROL
setup = UAVSetup()
setup.cmd = UAVSetup.SET_CONTROL_MODE
setup.control_mode = "CMD_CONTROL"
setup_pub.publish(setup)

# 发布一个位置点
cmd = UAVControlCMD()
cmd.cmd = UAVControlCMD.XyzPos
cmd.desired_pos = [0.0, 0.0, 1.0]

while not rospy.is_shutdown():
    cmd.header.stamp = rospy.Time.now()
    cmd_pub.publish(cmd)
    rate.sleep()
```

<div class="note">
<strong>控制频率建议</strong>
            对位置点类指令，可以低频反复发布或每到一个点发布一次；对速度类、跟踪类指令，建议 10-30 Hz 持续发布，并打开或自行实现超时保护。
          </div>

</section>

<section id="ugv">

## 无人车二次开发

无人车控制链路比 UAV 简单一些：`ugv_control_node` 读 odom，订阅 `UGVControlCMD`，输出底盘 `cmd_vel`。你的任务节点只要发布 `/ugv1/sunray_ugv/ugv_control_cmd`。

### 最小启动链路

```bash
# 仿真
roslaunch sunray_ugv_control ugv_control_sim.launch
roslaunch sunray_tutorial ugv_test_demo.launch demo_id:=1 ugv_id:=1

# 实车常见链路
roslaunch sunray_ugv_control wheeltec_robot.launch ugv_id:=1
roslaunch sunray_ugv_control ugv_control_exp.launch ugv_id:=1 location_source:=1
roslaunch sunray_tutorial ugv_test_demo.launch demo_id:=1 ugv_id:=1
```

### 定位源

| `location_source` | 来源 | 说明 |
| --- | --- | --- |
| `0` | 仿真 odom | 订阅 `//odom`。 |
| `1` | 动捕 VRPN | 订阅 `/vrpn_client_node_x/ugvX/pose` 和 twist。 |
| `2` | 自定义 odom | 由 `odom_topic` 参数指定。 |
| `3` | VIOBOT odom | 由 `odom_topic` 参数指定并走 viobot 回调。 |
| `4` | UWB LinkTrack | 订阅 `/nlink_linktrack_nodeframe2`。 |

### 新手先改哪里

优先复制 `sunray_tutorial/ugv_basic/ugv_block_pos.cpp` 或 `ugv_python/ugv_block_pos.py`。里面的关键代码是：

```python
cmd = UGVControlCMD()
cmd.cmd = UGVControlCMD.POS_CONTROL_ENU
cmd.desired_pos[0] = target_x
cmd.desired_pos[1] = target_y
cmd.desired_yaw = 0.0
cmd_pub.publish(cmd)
```

### UGV 控制参数

常用参数在 launch 中配置：

- `ugv_control_param/Kp_xy`：位置误差到速度的比例。
- `ugv_control_param/Kp_yaw`：偏航误差到角速度的比例。
- `ugv_control_param/max_vel_xy`：最大线速度限幅。
- `ugv_control_param/max_vel_yaw`：最大角速度限幅。
- `ugv_geo_fence/*`：地理围栏，超出后停止控制。
- `vel_topic`：底盘速度话题。空值时通常拼成 `/ugvX/cmd_vel`。
- `odom_topic`：自定义 odom 来源。

</section>

<section id="external-position">

## 外部定位

`external_fusion_node` 做两件事：第一，把外部定位送进 PX4；第二，把 MAVROS/PX4 的状态汇总成 Sunray 自定义状态。

### 为什么外部定位重要

PX4 在 OFFBOARD 下需要可靠定位。Sunray 的 `uav_control_node` 也会检查 `odom_valid` 和地理围栏。如果定位失效，控制节点会进入降落保护。

### 接入一个新的 odom 定位源

如果你的算法能发布 `nav_msgs/Odometry`，最简单：

```bash
roslaunch sunray_uav_control external_fusion.launch \
  external_source:=0 \
  position_topic:=/my_slam/odometry \
  use_vision_pose:=true
```

如果你的算法只发布 `geometry_msgs/PoseStamped`：

```bash
roslaunch sunray_uav_control external_fusion.launch \
  external_source:=1 \
  position_topic:=/my_pose
```

### RTK 原点

RTK 模式使用 `GeographicLib::LocalCartesian` 把经纬高转成本地 ENU。原点可以从 launch 参数配置，也可以等待地面站通过 `/sunray/global_rtk_origin` 发布 `RTKOrigin`。

<div class="warn">
<strong>坐标系要统一</strong>
            Sunray 上层控制基本按 ENU 理解：x 前/东向、y 左/北向、z 上。MAVROS 内部有 ENU/NED 转换，普通二次开发不要混用 PX4 原生 NED 概念，否则目标点方向容易反。
          </div>

</section>

<section id="planner">

## 规划接入

`sunray_planner_utils` 是把外部规划器接到 Sunray 控制接口的胶水层。新手做规划时，重点看这些节点，而不是一开始读完整 EGO/FUEL 源码。

| 节点/文件 | 作用 | 常见场景 |
| --- | --- | --- |
| `positionCmd2sunray.cpp` | 订阅 `PositionCommand`，发布 `UAVControlCMD` 或 `UGVControlCMD`。 | EGO/FUEL 轨迹接 Sunray。 |
| `positionCmd2sunrayugv.cpp` | UGV 专用的 `PositionCommand` 转换。 | 无人车 EGO。 |
| `goal2swarm.cpp` | 把 RViz 的 `/move_base_simple/goal` 分发成多个目标点。 | 多机/多车目标分配。 |
| `scan2Point.cpp` | 把 `LaserScan` 转 `PointCloud2`，并用 TF 投到 world。 | 2D 雷达给地图/规划使用。 |
| `point_cloud_transform.cpp` | 根据 odom 把点云转换到目标坐标系。 | 雷达点云坐标转换。 |
| `livox2Point.cpp` | 把 Livox 自定义点云转成 `sensor_msgs/PointCloud2`。 | Mid360/Livox 数据接入。 |

### EGO/FUEL 的正确打开方式

它们是第三方规划器，重点是输入/输出话题：

- 输入：无人机/无人车 odom、点云、目标点。
- 输出：规划轨迹或 `PositionCommand`。
- 桥接：通过 `positionCmd2sunray` 转成 `UAVControlCMD`。

```bash
# 脚本中常见的规划链路
roslaunch sunray_uav_control external_fusion.launch external_source:=2
roslaunch sunray_uav_control sunray_control_node.launch
roslaunch sunray_planner_utils sunray_ego_single.launch
roslaunch sunray_planner_utils positionCmd2sunray.launch
```

</section>
