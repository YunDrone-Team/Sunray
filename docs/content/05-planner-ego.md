<!-- title: 规划工具与 EGO 接入 -->

<section id="deep-planner-utils">

## 深入：sunray_planner_utils

`General_Module/sunray_planner_utils` 是 Sunray 与规划/点云/地图之间的适配层。这个包的价值是把 EGO、FUEL、FAST-LIO、Livox、2D 雷达等不同接口统一接到 Sunray 的控制话题。

### 节点职责

| 节点 | 文件 | 输入 | 输出 |
| --- | --- | --- | --- |
| `positionCmd2sunray` | `src/positionCmd2sunray.cpp` | `sunray_msgs/PositionCommand` | `UAVControlCMD` 或 `UGVControlCMD` |
| `positionCmd2sunrayugv` | `src/positionCmd2sunrayugv.cpp` | `PositionCommand` | `UGVControlCMD` |
| `goal2swarm` | `src/goal2swarm.cpp` | `/move_base_simple/goal` 或 waypoint yaml | `/goal_1`、`/goal_2` 等 |
| `point_cloud_transform` | `src/point_cloud_transform.cpp` | 局部点云 + odom | world 坐标点云 |
| `scan2point` | `src/scan2Point.cpp` | `LaserScan` + TF/pose/odom | `PointCloud2` |
| `livox2Point` | `src/livox2Point.cpp` | `livox_ros_driver2/CustomMsg` | `sensor_msgs/PointCloud2` |

### `positionCmd2sunray` 的 `control_type`

| `control_type` | UAV 输出模式 | 适合场景 |
| --- | --- | --- |
| `0` | `XyzPosYaw` | 只想让 Sunray 位置控制跟踪规划点。 |
| `1` | `XyzVelYaw` | 规划器输出速度，任务偏速度控制。 |
| `2` | `XyzPosVelYaw` | 位置 + 速度前馈，EGO 常用。 |
| `3` | `CTRL_Traj` | 使用 Sunray 自定义姿态控制器追踪位置、速度、加速度。 |

`enable_yaw=false` 时会把 yaw 固定为 0，适合先验证轨迹位置；需要机头朝向目标或跟随轨迹时再打开。

### 重要 launch 组合

| launch | 做什么 | 关键参数/话题 |
| --- | --- | --- |
| `positionCmd2sunray.launch` | 单独启动 EGO/FUEL 到 Sunray 的控制桥。 | `cmd_sub_topic`、`control_pub_topic`、`control_type`。 |
| `sunray_ego_single.launch` | 单 UAV EGO：桥接控制、点云转换、goal 分发、包含 EGO。 | `/uav1/pos_cmd` -> `/uav1/sunray/uav_control_cmd`。 |
| `sunray_ego_single_mid360.launch` | Mid360 点云输入的单 UAV EGO。 | 通常配合 `msg_MID360.launch`、`mapping_mid360.launch`。 |
| `sunray_ego_swarm.launch` | 多 UAV EGO 规划。 | `uav_num`、`goal2swarm`、多个 `/goal_i`。 |
| `sunray_ego_ugv.launch` | UGV 使用 EGO 规划。 | `/sunray/odometry`、`/sunray/pointCloud`、`/ugv1/pos_cmd`。 |
| `sunray_ego_ugv_swarm.launch` | 多 UGV EGO。 | 配合 `goal2swarm_ugv.launch`。 |
| `sunray_vins_ego.launch` | VINS 里程计 + EGO + Sunray 控制。 | `/vins_estimator/imu_propagate`、`control_type=3`。 |
| `sunray_fuel.launch` / `sunray_fuel_mid360.launch` | FUEL 接入 Sunray。 | 与 EGO 类似，重点是 pos_cmd 和点云。 |
| `goal2swarm.launch` / `goal2swarm_ugv.launch` | RViz 目标点转多个 agent 目标点。 | `offset`、`offset_direction`、`goal_topic`。 |
| `msg_MID360.launch` | 启动 Livox MID360 ROS2/ROS1 驱动节点。 | `frame_id`、`publish_freq`、`user_config_path`。 |
| `mapping_mid360.launch` | 启动 FAST-LIO Mid360 建图。 | `fast_lio`、`transform_odom_pointCloud`。 |

### EGO 接入的四条线

<div class="flow">
<span>odom</span>
<span>EGO grid map / FSM</span>
<span>traj_server</span>
<span>PositionCommand</span>
<span>positionCmd2sunray</span>
<span>UAVControlCMD</span>
</div>

<div class="flow">
<span>local point cloud</span>
<span>point_cloud_transform</span>
<span>world point cloud</span>
<span>EGO grid map</span>
</div>

<div class="flow">
<span>RViz 2D Nav Goal</span>
<span>goal2swarm</span>
<span>/goal_i</span>
<span>EGO target</span>
</div>

<div class="flow">
<span>EGO bspline</span>
<span>traj_server</span>
<span>pos_cmd</span>
<span>Sunray control</span>
</div>

</section>

<section id="deep-ego">

## 深入：ego-planner-swarm

`External_Module/ego-planner-swarm` 是引入的 EGO-Planner-Swarm。它是第三方规划算法，但仓库里已经有 Sunray 专用 launch 和适配。新手要关注“输入输出”和“Sunray 如何包一层”，不建议一开始改优化器。

### 包结构

| 子包 | 作用 | 阅读优先级 |
| --- | --- | --- |
| `planner/plan_manage` | EGO 主状态机、规划管理、`ego_planner_node`、`traj_server`。 | 最高，尤其 launch。 |
| `planner/plan_env` | 占据栅格地图、传感器融合、局部地图。 | 调地图和点云时看。 |
| `planner/path_searching` | A* / kinodynamic 搜索。 | 算法研究时看。 |
| `planner/bspline_opt` | B 样条轨迹优化。 | 算法研究时看。 |
| `planner/traj_utils` | 轨迹消息和工具。 | 桥接和消息问题时看。 |
| `planner/drone_detect` | 深度图检测其他无人机。 | 多机避障感知时看。 |
| `planner/rosmsg_tcp_bridge` | ROS 消息 TCP 桥。 | 特殊通信需求时看。 |
| `uav_simulator/*` | EGO 自带仿真、mock map、local sensing、SO3 控制等。 | Sunray 已有仿真链路时少改。 |

### Sunray 专用 launch

| 路径 | 说明 |
| --- | --- |
| `plan_manage/launch_sunray/run_in_sim.launch` | Sunray 风格的 EGO 仿真运行入口，启动 `ego_planner_node` 和 `traj_server`，并可包含 EGO 自带模拟器。 |
| `plan_manage/launch_sunray/sunray_ego_sim_with_rviz.launch` | 带 RViz 和随机地图的 EGO 仿真演示。 |
| `plan_manage/launch_sunray/sunray_ego_exp.launch` | 实机/外部里程计场景的 EGO 入口。 |
| `plan_manage/launch_sunray/sunray_map_generator.launch` | 调用 `sunray_map_generator` 生成全局/局部点云地图。 |
| `plan_manage/launch_sunray/rviz.launch` | EGO RViz 配置入口。 |

### EGO 的关键话题

| 话题 | 方向 | 含义 |
| --- | --- | --- |
| `~odom_world` / `~grid_map/odom` | 输入 | 规划器和地图使用的里程计。 |
| `~grid_map/cloud` | 输入 | 点云障碍物。 |
| `~grid_map/depth`、`~grid_map/pose` | 输入 | 深度图模式使用。 |
| `~move_base_simple/goal` | 输入 | 手动目标点。 |
| `~planning/bspline` | 输出 | EGO 规划出的 B 样条。 |
| `position_cmd` | 输出 | `traj_server` 采样 B 样条后输出的位置/速度/加速度/yaw。 |
| `/broadcast_bspline` | 输入/输出 | 多机共享轨迹，用于互相避让。 |

### 常调参数

| 参数组 | 参数 | 影响 |
| --- | --- | --- |
| `fsm` | `flight_type`、`planning_horizon`、`thresh_replan_time`、`fail_safe` | 目标模式、规划范围、重规划频率和安全策略。 |
| `grid_map` | `resolution`、`map_size_*`、`local_update_range_*`、`obstacles_inflation`、`ground_height` | 地图分辨率、大小、感知范围和障碍物膨胀。 |
| `manager` | `max_vel`、`max_acc`、`max_jerk`、`control_points_distance` | 轨迹速度、加速度、平滑性和控制点间距。 |
| `optimization` | `lambda_smooth`、`lambda_collision`、`lambda_feasibility`、`dist0`、`swarm_clearance` | 平滑、避障、可行性和多机间距权重。 |
| `bspline` | `limit_vel`、`limit_acc`、`limit_ratio` | B 样条动态约束。 |

<div class="note">
<strong>调 EGO 的顺序</strong>
            先确认 odom 和点云坐标系正确，再确认 RViz 中地图和目标点正确，最后才调 `max_vel`、`planning_horizon`、`obstacles_inflation` 和优化权重。坐标系错时，调参数只会掩盖问题。
          </div>

</section>
