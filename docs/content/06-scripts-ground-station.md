<!-- title: 快速启动脚本与地面站 -->

<section id="quick-scripts">

## 快速启动脚本与地面站

仓库提供了多组 `scripts_xxx` 快速启动脚本。它们把一套任务需要的多个 `roslaunch` 串起来，既可以手动执行，也可以作为地面站快速启动的命令模板。理解这些脚本，比背单个 launch 更接近真实使用方式。

### 脚本目录

| 目录 | 用途 | 典型脚本 |
| --- | --- | --- |
| `scripts_sim` | 仿真快速启动。 | `demo_takeoff_hover_land.sh`、`sunray_uav_ego_sim.sh`、`formation_6uav.sh`、`sunray_ugv_sim.sh`。 |
| `scripts_exp` | 实机快速启动。 | `demo_takeoff_hover_land.sh`、`uav_control_viobot.sh`、`sunray_uav_ego_mid360.sh`、`ugv_formation.sh`。 |
| `scripts_swarm` | 集群/GPS/RTK/leader-follower 快速启动。 | `leader_fowllower_sim_3uav.sh`、`uav_control_sim_gps.sh`、`uav_control_sim_rtk.sh`。 |
| `server` | 机载端/地面站后台自启动配置。 | `server.sh`、`server.env`、`communication.sh`。 |
| `tests/production` | 生产/硬件测试场景。 | `sunray150_basic_sim_test.sh`、`sunray150_lidar_exp_test.sh`。 |

### 地面站快速启动的本质

地面站快速启动通常不是直接调用某个 C++ 函数，而是让机载电脑执行预置的 shell 启动命令。`sunray_communication_bridge` 负责地面站与机载端之间的通信，`server/server.sh` 则展示了机载端如何按配置启动通信桥、MAVROS、外部定位和控制节点。

<div class="flow">
<span>地面站选择任务</span>
<span>通信桥/机载服务收到命令</span>
<span>执行 scripts_xxx 或 server 配置</span>
<span>启动多个 ROS 节点</span>
<span>地面站接收状态</span>
</div>

### `server/server.env` 控制哪些自启动

| 变量 | 含义 |
| --- | --- |
| `RUN_SEVER` | 是否启动 `roscore`。变量名保留了当前脚本写法。 |
| `ID`、`NAME` | 当前智能体编号和类型，`NAME=uav` 时按无人机启动通信。 |
| `UAV_NUM`、`UGV_NUM` | 系统中的无人机/无人车数量。 |
| `START_GROUND_STATION` | 是否启动 `sunray_communication_bridge`。 |
| `START_MAVROS` | 是否启动 `sunray_mavros_exp.launch`。 |
| `EXTERNAL_SOURCE`、`START_EXTERNAL_POSITION` | 外部定位类型以及是否启动 `external_fusion.launch`。 |
| `START_CONTROL` | 是否启动 `sunray_control_node.launch`。 |

### 仿真脚本分类

| 类别 | 脚本 | 启动链路 |
| --- | --- | --- |
| 基础 UAV | `demo_takeoff_hover_land.sh`、`demo_block_pos.sh`、`demo_circle.sh`、`demo_hexagon.sh`、`demo_triangle_pos.sh`、`demo_waypoint.sh` | 仿真环境 + external fusion + control node + tutorial demo。 |
| 视觉任务 | `demo_auto_land.sh`、`land_on_a_moving_car.sh`、`demo_follow_car.sh`、`demo_follow_tag.sh`、`demo_search_target_sim.sh` | 仿真环境 + UAV 控制 + 检测 launch + advanced tutorial。 |
| 规划任务 | `sunray_uav_ego_sim.sh`、`sunray_uav_ego_global_map_sim.sh`、`sunray_uav_vins_ego_sim.sh`、`fuel_sim.sh` | UAV 控制 + EGO/FUEL + 点云/地图 + `positionCmd2sunray`。 |
| UGV | `sunray_ugv_sim.sh`、`sunray_ugv_Astar_test.sh`、`sunray_ugv_ego_sim.sh` | UGV 仿真 + UGV 控制 + A*/EGO。 |
| 编队 | `formation_6uav.sh`、`formation_3ugv.sh`、`uav_formation.sh`、`ugv_formation.sh`、`uav_ugv_formation.sh` | 多智能体仿真 + 控制节点 + ORCA + formation。 |
| GPS/RTK | `uav_control_sim_gps.sh`、`scripts_swarm/uav_control_sim_gps.sh`、`scripts_swarm/uav_control_sim_rtk.sh` | 仿真 + `external_source:=5/6` + 控制节点。 |

### 实机脚本分类

| 类别 | 脚本 | 说明 |
| --- | --- | --- |
| 基础 UAV 动作 | `demo_takeoff_hover_land.sh`、`demo_block_pos.sh`、`demo_circle.sh`、`demo_hexayon.sh`、`demo_waypoint.sh` | MAVROS + external fusion + control node + tutorial demo。 |
| 定位版本 | `*_mid360.sh`、`*_viobot.sh`、`*_rtk.sh` | 同一任务换不同定位源或雷达建图链路。 |
| FMT 平台 | `demo_*_fmt.sh`、`fmt_control.sh` | 使用 `sunray_fmt_control`。 |
| 规划 | `sunray_uav_ego_mid360.sh`、`sunray_ugv_ego.sh`、`ugv_ego_swarm.sh`、`sunray_ugv_ego_search.sh` | Mid360/FAST-LIO + EGO + Sunray 控制。 |
| UGV | `ugv_block.sh`、`ugv_circle.sh`、`ugv_py_block.sh`、`ugv_py_circle.sh`、`ugv_Astar_obstacle.sh` | 底盘驱动 + UGV 控制 + tutorial。 |
| 视觉/云台/视频 | `demo_auto_land.sh`、`demo_follow_car.sh`、`demo_detection_viobot.sh`、`rtsp_push.sh` | 相机/检测/推流配合任务节点。 |
| 编队 | `uav_formation.sh`、`ugv_formation.sh` | 实机多智能体 ORCA + formation。 |
| 清理 | `kill_node.sh`、`kill_node_viobot.sh` | 结束 tmux 会话、关闭 VIOBOT 算法、清理 ROS 节点。 |

### 脚本改造给地面站调用的原则

- 脚本开头显式 `source /opt/ros/noetic/setup.bash` 和 `source ~/Sunray/devel/setup.bash`，避免地面站远程启动时环境不完整。
- 把 `UAV_ID`、`UGV_ID`、`EXTERNAL_SOURCE`、`UAV_NUM` 等做成变量或参数，不要写死多个版本。
- 实机脚本要把 MAVROS、外部定位、控制节点、任务节点分阶段启动，并保留足够 `sleep`。
- 地面站快速启动更适合调用稳定脚本，不适合直接拼很长的临时命令。
- 任务结束要配套清理脚本，尤其是 VIOBOT、tmux 和多机任务。

```bash
# 手动执行快速启动脚本
source devel/setup.bash
bash scripts_sim/demo_takeoff_hover_land.sh

# 实机示例，先确认安全条件
bash scripts_exp/demo_takeoff_hover_land.sh

# 机载端按 server.env 自启动后台节点
bash server/server.sh
```

</section>
