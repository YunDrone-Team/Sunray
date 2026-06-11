<!-- title: 仿真、调试与安全速查 -->

<section id="simulation">

## 仿真到实机

从仿真迁移到实机时，任务节点通常不用大改，主要变化在启动链路和定位源。

| 环节 | 仿真 | 实机 |
| --- | --- | --- |
| 仿真/飞控连接 | `sunray_simulator` 相关 launch。 | `sunray_mavros_exp.launch` 连接真实 PX4。 |
| 定位源 | `external_source:=2` Gazebo 真值。 | 动捕 `3`、VIOBOT `4`、GPS `5`、RTK `6` 或自定义 odom `0`。 |
| 控制节点 | `sunray_control_node.launch`。 | 同样使用，但围栏、高度、降落速度要按实机调小。 |
| 任务节点 | `sunray_tutorial` 示例。 | 同样使用，但速度、范围、超时、安全逻辑要保守。 |
| 可视化/通信 | RViz、仿真通信桥。 | 地面站通信桥、遥控器、急停。 |

### 实机前必须确认

- MAVROS 已连接，`/uav1/mavros/state` 正常。
- `/uav1/sunray/px4_state` 和 `/uav1/sunray/uav_state` 有数据。
- `uav_state.odom_valid == true`。
- 地理围栏范围比实验场地略小，而不是无限大。
- 遥控器或地面站可以切回人工控制/降落/急停。
- 首次实机速度、高度、距离都减半或更低。

</section>

<section id="debug">

## 调试方法

Sunray 是 ROS 系统，排错先看节点、话题、消息和坐标系。

### 常用命令

```bash
# 看节点是否启动
rosnode list

# 看话题是否存在
rostopic list | grep sunray

# 看无人机状态
rostopic echo /uav1/sunray/uav_state

# 看控制指令是否真的发出
rostopic echo /uav1/sunray/uav_control_cmd

# 看 MAVROS 连接和 PX4 模式
rostopic echo /uav1/mavros/state

# 看话题频率
rostopic hz /uav1/sunray/uav_state

# 看 TF
rosrun tf view_frames
```

### 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 任务节点一直等待连接 | `external_fusion_node` 是否启动，MAVROS 是否连接，话题前缀是否是 `/uav1`。 |
| 无法进入 `CMD_CONTROL` | `odom_valid` 是否为 true，是否未解锁且 `use_rc_control` 限制，PX4 是否允许 OFFBOARD。 |
| 无人机不动 | `uav_control_cmd` 是否发布，`uav_state.control_mode` 是否为 `CMD_CONTROL`，PX4 是否 `OFFBOARD`。 |
| 方向反了 | 检查 ENU/机体系、目标点坐标、动捕坐标轴、雷达/相机 TF。 |
| 突然降落 | 通常是定位失效或越过地理围栏，查看控制节点输出和 `uav_state.odom_valid`。 |
| UGV 不走 | 检查 odom 是否有效、`vel_topic` 是否对应底盘、`cmd_vel` 是否有输出。 |

</section>

<section id="safety">

## 安全清单

<div class="danger">
<strong>实机测试顺序</strong>
            先无桨或架高验证话题和模式，再低高度短距离测试，最后再做完整任务。任何新算法第一次实机都不要直接跑全速、全范围、全流程。
          </div>

- 确认遥控器接管、上锁、降落、急停通道有效。
- 确认 `geo_fence` 设置合理，尤其是 `z_max`。
- 确认 `Land_speed`、`land_end_speed` 不过大。
- 确认定位源稳定，静止时位置漂移不大。
- 确认任务节点有退出策略：目标丢失、超时、定位失效时悬停或降落。
- 速度控制任务必须持续发布，且要限制最大速度。
- 视觉任务要处理目标突然消失、误检、延迟。
- 多机任务要先单机，再双机，再多机。

</section>

<section id="appendix">

## 速查表

### 常用话题

| 话题 | 消息 | 方向 |
| --- | --- | --- |
| `/uav1/sunray/uav_control_cmd` | `sunray_msgs/UAVControlCMD` | 任务节点 -> UAV 控制节点 |
| `/uav1/sunray/setup` | `sunray_msgs/UAVSetup` | 任务节点/地面站 -> UAV 控制节点 |
| `/uav1/sunray/uav_state` | `sunray_msgs/UAVState` | UAV 控制节点 -> 任务节点/地面站 |
| `/uav1/sunray/px4_state` | `sunray_msgs/PX4State` | external fusion -> UAV 控制节点 |
| `/uav1/sunray/uav_odom` | `nav_msgs/Odometry` | external fusion -> 规划/RViz |
| `/ugv1/sunray_ugv/ugv_control_cmd` | `sunray_msgs/UGVControlCMD` | 任务节点 -> UGV 控制节点 |
| `/ugv1/sunray_ugv/ugv_state` | `sunray_msgs/UGVState` | UGV 控制节点 -> 任务节点/地面站 |
| `/sunray/global_rtk_origin` | `sunray_msgs/RTKOrigin` | 地面站/通信桥 -> external fusion |

### 常用文件入口

| 想做什么 | 先看哪里 |
| --- | --- |
| 写 UAV 基础任务 | `General_Module/sunray_tutorial/uav_basic` 或 `uav_python` |
| 写 UGV 基础任务 | `General_Module/sunray_tutorial/ugv_basic` 或 `ugv_python` |
| 看 UAV 控制状态机 | `General_Module/sunray_uav_control/uav_control/UAVControl.cpp` |
| 接新定位源 | `General_Module/sunray_uav_control/externalFusion/ExternalPosition.h` |
| 接 EGO/FUEL | `General_Module/sunray_planner_utils/src/positionCmd2sunray.cpp` |
| 看 UGV 控制 | `General_Module/sunray_ugv_control/ugv_control/UGVControl.cpp` |
| 看启动组合 | `scripts_sim`、`scripts_exp` |
| 看构建模块 | `buildscripts/modules.yaml` |

### 建议阅读顺序

1. `README.md`。
2. `buildscripts/modules.yaml`。
3. `sunray_msgs/msg/UAVControlCMD.msg`、`UAVState.msg`、`UAVSetup.msg`。
4. `sunray_tutorial/uav_python/takeoff_hover_land.py` 或 `uav_basic/takeoff_hover_land.cpp`。
5. `sunray_tutorial/uav_basic/block_xyzpos.cpp`。
6. `sunray_uav_control/launch/sunray_control_node.launch`。
7. `sunray_uav_control/uav_control/UAVControl.cpp` 的 `init()`、`mainLoop()`、`handle_cmd_control()`。
8. 按项目需要进入外部定位、UGV、规划、视觉或编队章节。

</section>

<div class="footer">

本手册根据当前仓库结构编写，文档文件位于 `docs/`。它只说明二次开发思路和接口，不替代飞行安全规范、PX4 官方文档或硬件厂商手册。

</div>
