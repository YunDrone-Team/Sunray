<!-- title: 重点包深入：common 与 UAV 控制 -->

<section id="deep-common">

## 深入：sunray_common

`General_Module/sunray_common` 是所有 Sunray 包的公共基础。它不是一个复杂算法包，而是两类内容：公共头文件 `common_lib` 和自定义消息包 `sunray_msgs`。二次开发时，你会频繁依赖它，但通常不需要改它。

### 目录职责

| 路径 | 职责 | 建议 |
| --- | --- | --- |
| `common_lib/ros_msg_utils.h` | 集中包含 ROS、MAVROS、Sunray 消息和常用工具头文件。 | C++ 示例大量使用，适合快速开发。 |
| `common_lib/control_utils.h` | UAV 起飞、降落、返航等辅助流程封装。 | 新手任务节点优先复用。 |
| `common_lib/geometry_utils.h` | 四元数、yaw、姿态转换等几何工具。 | 处理坐标和姿态时使用。 |
| `common_lib/math_utils.h` | 数学辅助函数。 | 按需使用。 |
| `common_lib/WGS84.h` | 经纬高、局部坐标相关类型与转换支持。 | RTK/GPS 场景会用到。 |
| `common_lib/sunray_logger.h` | 统一日志打印。 | C++ 节点建议使用，便于输出颜色和级别。 |
| `sunray_msgs/msg` | UAV、UGV、规划、视觉、编队、通信等自定义消息。 | 二次开发必须熟悉。 |
| `sunray_msgs/test_msg` | leader-follower/任务测试消息。 | 编队或测试时再看。 |
| `sunray_msgs/viobot_msg` | VIOBOT 算法开关和状态消息。 | 使用 VIOBOT 时看。 |

### `control_utils.h` 的意义

`Control_Utils` 是给任务节点用的“小工具类”。它会订阅 `/uavX/sunray/uav_state`，发布 `/uavX/sunray/uav_control_cmd` 和 `/uavX/sunray/setup`，并提供 `auto_takeoff()`、`auto_land()`、`auto_return()`。

```cpp
Control_Utils uav_control_utils;
uav_control_utils.init(nh, uav_id, node_name);

uav_control_utils.auto_takeoff();
// 自己的任务逻辑
uav_control_utils.auto_land();
```

这也是 `sunray_tutorial/uav_basic` 示例推荐的写法。它适合新手，因为起飞流程里已经包含等待连接、切 `CMD_CONTROL`、解锁和发送起飞命令。

### 消息分组理解

| 分组 | 代表消息 | 用途 |
| --- | --- | --- |
| UAV 控制 | `UAVControlCMD`、`UAVSetup`、`UAVState`、`PX4State`、`ExternalOdom` | 无人机任务开发主接口。 |
| UGV 控制 | `UGVControlCMD`、`UGVState` | 无人车任务开发主接口。 |
| 航点 | `WayPoint`、`WayPointState`、`UAVWayPoint`、`Point` | 地面站或航点任务。 |
| 规划/轨迹 | `PositionCommand`、`Target` | EGO/FUEL 和 Sunray 控制之间的桥接。 |
| 视觉 | `TargetMsg`、`TargetsInFrameMsg` | 检测/跟踪目标输出。 |
| 编队/避障 | `Formation`、`OrcaCmd`、`OrcaSetup` | 队形切换、ORCA 状态和指令。 |
| 通信/比赛 | `TextInfo`、`RTKOrigin`、`Competion` | 地面站信息、RTK 原点、比赛地图。 |
| 设备 | `RcState`、`LinktrackNodeframe2`、`algo_ctrl`、`algo_status` | 遥控器、UWB、VIOBOT。 |

<div class="tip">
<strong>什么时候改 sunray_common</strong>
            只有在你需要新增跨包共享的消息或通用工具函数时才改。普通任务开发不要改已有消息字段，否则所有依赖该消息的包都要重新编译，地面站协议也可能要同步。
          </div>

</section>

<section id="deep-uav-control">

## 深入：sunray_uav_control

`General_Module/sunray_uav_control` 是无人机系统最重要的核心包。它把 PX4/MAVROS、外部定位、遥控器、航点、终端控制和自定义姿态控制器包装成 Sunray 统一接口。

### 包内模块

| 目录 | 核心文件 | 作用 |
| --- | --- | --- |
| `uav_control` | `UAVControl.cpp/.h`、`uav_control_node.cpp` | 主控制状态机，订阅任务指令，发布 MAVROS setpoint。 |
| `externalFusion` | `externalFusion.cpp`、`ExternalPosition.h` | 汇总 PX4 状态，接入外部定位，发布 `PX4State`。 |
| `launch` | `sunray_control_node.launch`、`external_fusion.launch`、`sunray_mavros_exp.launch` | UAV 控制、定位、MAVROS 的主要启动入口。 |
| `launch_utils` | `terminal_control.launch`、`rviz_goal_bridge.launch`、`positioning_accuracy.launch` | 终端控制、RViz 目标点桥接、定位精度测试。 |
| `launch_att` | `sunray_control_test.launch`、`control_evaluation_node.launch` | 姿态控制和控制性能评估。 |
| `uav_control/pos_controller` | `pos_controller_pid.h`、`control_evaluation.h` | `CTRL_XyzPos`、`CTRL_Traj` 使用的自定义位置控制器。 |
| `waypoint` | `Waypoint.h`、`waypoint_mission_node.cpp`、`waypoint_pub.cpp` | 航点任务执行和航点发布。 |
| `rc_input` | `rc_input_node.cpp`、`joy_node.cpp` | 遥控器通道解析和虚拟摇杆。 |
| `utils` | `PX4ParamManager`、`RvizGoalBridge`、`terminal_control.cpp` | 参数管理、RViz goal 转控制指令、终端交互控制。 |
| `mavlink` | `mavlink_control.cpp`、`serial_port.cpp` | 不走 `vision_pose` 时直接用 MAVLink 发送外部定位。 |

### 主控制节点：`uav_control_node`

主循环频率是 200 Hz。它的核心逻辑在 `UAVControl::mainLoop()`：

1. `check_state()`：检查定位、围栏、RC 超时，并发布 `UAVState`。
2. `check_flip()`：检测姿态翻转，触发 emergency kill。
3. 根据 `control_mode` 进入 `INIT`、`RC_CONTROL`、`CMD_CONTROL`、`LAND_CONTROL` 或 `WITHOUT_CONTROL`。
4. `CMD_CONTROL` 中根据 `UAVControlCMD.cmd` 区分特殊指令、全局位置、自定义控制器和基础 MAVROS setpoint。

### `UAVControlCMD` 到 MAVROS 的转换

| Sunray 指令 | 控制节点行为 | 发布目标 |
| --- | --- | --- |
| `XyzPos` / `XyzVel` / `XyVelZPos` 等 | 设置 `type_mask`，填充 `mavros_msgs::PositionTarget`。 | `/uavX/mavros/setpoint_raw/local` |
| `XyzPosYawBody` / `XyzVelYawBody` 等 | 将机体系目标按当前 yaw 旋转到 ENU。 | `setpoint_raw/local` |
| `GlobalPos` | 填充经纬度、高度、yaw。 | `/uavX/mavros/setpoint_raw/global` |
| `CTRL_XyzPos` / `CTRL_Traj` | 调用 `pos_controller_pid` 计算姿态和推力。 | `/uavX/mavros/setpoint_raw/attitude` |
| `Takeoff` / `Hover` / `Return` / `Land` | 特殊函数处理，修改目标点或控制状态机。 | local setpoint 或降落状态机 |
| `Point` | 只发布 `geometry_msgs/PoseStamped` 目标点。 | `/uavX/sunray/goal` |

### 外部定位节点：`external_fusion_node`

`external_fusion_node` 不只是“定位转发”，它还订阅 MAVROS 的 `state`、`extended_state`、`battery`、`local_position`、`imu`、GPS、目标 setpoint，并打包成 `PX4State`。`uav_control_node` 只需要订阅 `/uavX/sunray/px4_state` 就能拿到完整状态。

### 关键启动文件

| launch | 用途 | 常用参数 |
| --- | --- | --- |
| `sunray_mavros_exp.launch` | 连接真实 PX4，启动 MAVROS。 | `uav_id`、`ip`、`fcu_url`、`gcs_url`。 |
| `external_fusion.launch` | 启动外部定位汇总节点。 | `external_source`、`position_topic`、`use_vision_pose`、`rtk_origin_mode`。 |
| `sunray_control_node.launch` | 启动主控制节点。 | `Takeoff_height`、`geo_fence/*`、`use_rc_control`、`check_flip`、`check_cmd_timeout`。 |
| `external_fusion_swarm.launch` | 递归启动多机 external fusion。 | `uav_num`、`external_source`。 |
| `sunray_control_node_swarm.launch` | 递归启动多机控制节点。 | `uav_num`、围栏和起降参数。 |
| `rc_control.launch` | 解析遥控器通道控制模式、解锁、降落、kill。 | `channel_arm`、`channel_mode`、`channel_land`、`channel_kill`。 |
| `terminal_control.launch` | 终端控制多个 UAV。 | `uav_num`。 |
| `rviz_goal_bridge.launch` | RViz 2D Nav Goal 转 UAV 控制目标。 | `default_height`、围栏、`goal_timeout`。 |

<div class="warn">
<strong>开发边界</strong>
            普通任务开发只发布 `/uavX/sunray/uav_control_cmd` 和 `/uavX/sunray/setup`。只有做控制器、定位源或 MAVROS/PX4 适配时，才进入 `UAVControl.cpp`、`ExternalPosition.h` 或 `mavlink` 目录修改。
          </div>

</section>
