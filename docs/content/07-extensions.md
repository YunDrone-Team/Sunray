<!-- title: 扩展模块 -->

<section id="vision">

## 视觉与目标

视觉相关代码分散在 `External_Module/sunray_detection`、`sunray_tutorial/advanced` 和若干 detection launch 中。新手先理解输出消息，不要先改检测算法。

### Sunray 目标消息

`TargetMsg` 描述单个目标，包含图像归一化中心 `cx/cy`、宽高 `w/h`、置信度、类别、跟踪 ID、3D 位置和视线角。`TargetsInFrameMsg` 是一帧图像中的目标数组。

### 典型任务

| 任务 | 文件/launch | 思路 |
| --- | --- | --- |
| 二维码/ArUco 降落 | `auto_land_by_pose.launch`、`landmark_detection*.launch` | 检测目标位置，计算相对误差，发布 UAV 速度/位置指令。 |
| 跟车 | `follow_car_xyvelzposyawbody.launch` | 使用机体系速度控制跟随移动目标。 |
| 搜索目标并发送给 UGV | `search_target_and_send_to_ugv.launch` | 无人机搜索目标，得到全局位置后发给无人车/规划模块。 |
| 简易避障 | `simple_obstacle_avoidance*.launch` | 读取深度/图像信息，按阈值调整速度。 |

</section>

<section id="formation">

## 编队与避障

编队相关在 `sunray_formation`，避碰核心 ORCA 在 `sunray_formation/sunray_orca`。新手阶段先用 launch 和配置文件，不建议先改 ORCA 内部数学实现。

| 路径 | 作用 |
| --- | --- |
| `formation_control/config/*.yaml` | 静态编队、圆、8 字、航点等队形配置。 |
| `formation_control/launch/formation_uav_sim.launch` | 无人机仿真编队控制。 |
| `formation_control/launch/formation_ugv_sim.launch` | 无人车仿真编队控制。 |
| `formation_control/launch/formation_switch.launch` | 编队模式切换。 |
| `sunray_orca/launch/orca_swarm_uav.launch` | 多 UAV ORCA 避障。 |
| `sunray_orca/launch/orca_swarm_ugv.launch` | 多 UGV ORCA 避障。 |

```bash
# 脚本示例
bash scripts_sim/formation_6uav.sh
bash scripts_sim/formation_3ugv.sh
```

</section>

<section id="gimbal-media">

## 云台与视频

`sunray_gimbal` 和 `sunray_media` 属于外设能力。除非你的任务需要相机控制、拍照、云台追踪或 RTSP 推流，否则可以先跳过。

| 模块 | 用途 | 入口 |
| --- | --- | --- |
| `sunray_gimbal/scripts/gimbal_control.py` | 云台控制节点。 | `gimbal_control.launch` |
| `sunray_gimbal/demo/*` | 拍照、扫描、目标锁定、二维码追踪示例。 | `scripts_exp/demo_gimbal_*.sh` |
| `sunray_gimbal/msg/GimbalParams.msg` | 云台参数消息。 | 任务节点可订阅/发布。 |
| `sunray_media/src/rtsp_push.cpp` | 视频推流。 | `sunray_media/launch/rtsp_push.launch` |

</section>

<section id="communication">

## 通信桥接

`sunray_communication_bridge` 负责地面站、机载电脑和多智能体之间的 TCP/UDP 通信。普通算法开发不需要改它，但需要知道它会转发哪些 ROS 话题。

### 它会做什么

- 订阅 UAV/UGV 状态，通过 UDP/TCP 发给地面站。
- 接收地面站控制指令，发布到 `uav_control_cmd`、`setup`、`ugv_control_cmd`。
- 转发航点、RTK 原点、PX4 参数、编队切换、Viobot 算法开关。
- 在真机多智能体中，转发其他 UAV/UGV 的状态。

### 常用 launch 参数

| 参数 | 说明 |
| --- | --- |
| `is_simulation` | 仿真/真机模式。仿真通常一个通信节点管理多个智能体。 |
| `uav_id` / `ugv_id` | 当前智能体 ID。 |
| `uav_experiment_num` / `ugv_experiment_num` | 真机数量。 |
| `uav_simulation_num` / `ugv_simulation_num` | 仿真数量。 |
| `tcp_port`、`udp_port`、`udp_ground_port` | 通信端口。 |
| `PX4StateTransmitEnabled`、`UAVStateTransmitEnabled` | 是否传输对应状态。 |

</section>

<section id="fmt">

## FMT 控制

`sunray_fmt_control` 是针对 FMT 飞控控制话题的功能包。它和普通 PX4/MAVROS 控制链路相似，但启动文件和控制节点不同。

| 入口 | 作用 |
| --- | --- |
| `sunray_fmt_control_node.launch` | 启动 FMT 控制节点。 |
| `fmt_external_fusion.launch` | FMT 外部定位融合。 |
| `sunray_mavros_exp.launch` | FMT 实机 MAVROS 链路。 |
| `scripts_exp/demo_*_fmt.sh` | FMT 平台示例任务脚本。 |

如果你使用的是普通 Sunray PX4 平台，优先看 `sunray_uav_control`；如果项目明确要求 FMT，再进入这个包。

</section>
