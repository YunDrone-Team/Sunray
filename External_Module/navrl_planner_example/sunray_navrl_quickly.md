# Sunray NavRL 快速启动说明

本文说明如何在 Sunray 中启动 NavRL 部署/推理链路。这里的 NavRL 指的是
`planning/third_party_planner_examples/navrl_planner_example` 下的 ROS1 示例，
不是 Isaac Sim / Orbit 训练链路。

## 1. 启动链路组成

NavRL 在 Sunray 中不是一个单独 `roslaunch` 就能完整跑起来的节点。完整链路通常包含：

1. 数据源：`sunray_sim` 仿真，或真机定位/点云/控制链路。
2. NavRL 地图和适配节点：`sunray_navrl_adapter` 的 launch 会启动 `map_manager` 和 Sunray 适配节点。
3. Sunray 控制执行端：UAV 仿真需要 `sunray_uav_control`，UGV 仿真由 `NavRL2SunrayUGV_sim` 直接输出 `cmd_vel` 给 `sunray_sim`。
4. NavRL 策略推理节点：`rosrun navigation_runner navigation_node.py`。
5. 目标点输入：RViz 的 `2D Nav Goal`，或 `NavRLTerminalControl` 发布 `/move_base_simple/goal`。

注意：`NavRL2Sunray_sim.launch` 和 `NavRL2SunrayUGV_sim.launch` 只启动地图和适配器，不会启动 `navigation_node.py`。策略节点需要单独在 Conda 环境中启动。

## 2. 首次准备

在仓库根目录执行：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR

# CPU 环境
./planning/third_party_planner_examples/navrl_planner_example/setup_navrl_planner_env.sh --torch cpu --build

# 或 CUDA 11.8 环境
./planning/third_party_planner_examples/navrl_planner_example/setup_navrl_planner_env.sh --env NavRL-cu118 --torch cu118 --build
```

确认策略权重存在：

```bash
ls planning/third_party_planner_examples/navrl_planner_example/navigation_runner/scripts/ckpts/navrl_checkpoint.pt
```

如果该文件不存在，`navigation_node.py` 会在加载策略时失败。

每个 ROS 终端建议先执行：

```bash
cd /home/taolin/Documents/GitHub/Sunray_V2_PR
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

启动 `navigation_node.py` 的终端还需要激活 Conda：

```bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate NavRL
```

如果使用了自定义环境名，例如 `NavRL-cu118`，激活对应环境即可。

## 3. 单 UAV 仿真快速启动

这个流程对应启动面板中的“NavRL 单无人机避障验证”。

### 终端 1：启动 sunray_sim 单 UAV

```bash
roslaunch sunray_sim sunray_sim.launch
```

可选：使用随机地图。

```bash
roslaunch sunray_sim sunray_sim.launch map_source:=random_map
```

### 终端 2：启动 NavRL 到 Sunray 的仿真适配链路

```bash
roslaunch sunray_navrl_adapter NavRL2Sunray_sim.launch rviz:=true
```

该 launch 会启动：

- `map_manager/occupancy_map_node`
- `sunray_navrl_adapter/NavRL2Sunray_node`
- RViz，可通过 `rviz:=false` 关闭

默认话题关系：

- 订阅 odom：`/uav1/sunray_sim/odom`
- 订阅局部点云：`/uav1/sunray_sim/cloud_sensor_frame`
- 订阅 NavRL 原始速度：`/CERLAB/quadcopter/cmd_vel`
- 发布 Sunray 控制指令：`/uav1/sunray/uav_control/control_cmd`

### 终端 3：启动定位融合

```bash
roslaunch localization_fusion localization_fusion.launch \
  source_id:=5 \
  agent_name:=uav \
  agent_id:=1 \
  use_private_agent_key:=true
```

这里 `source_id:=5` 对接 `sunray_sim` 的 odom。

### 终端 4：启动 UAV 控制器

```bash
roslaunch sunray_uav_control uav_control.launch \
  airframe_type:=sunray_sim \
  controller_types:=0 \
  agent_name:=uav \
  agent_id:=1 \
  enable_uav_control_monitor:=true
```

`controller_types:=0` 是 PX4_OriginController，和启动面板里的 NavRL 单 UAV 验证流程一致。

### 终端 5：启动目标点终端工具，可选

```bash
roslaunch sunray_navrl_adapter NavRLTerminalControl.launch
```

该工具的菜单 1 会发布 NavRL 目标点到 `/move_base_simple/goal`。也可以不用它，直接在 RViz 中使用 `2D Nav Goal`。

### 终端 6：启动 NavRL 策略节点

```bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

如果使用 CPU 环境，建议显式覆盖设备：

```bash
rosrun navigation_runner navigation_node.py device=cpu sim.device=cpu sim.use_gpu=false
```

如果使用 CUDA 环境，默认配置是 `device: "cuda:0"`，可以直接运行上面的普通命令。

`navigation_node.py` 启动后会在终端中出现：

```text
[nav-ros]: Press Enter to STOP motion!
```

这是人工暂停开关。不要随手按 Enter；第一次 Enter 会进入暂停，第二次 Enter 会继续。

### 发布目标点

方式 1：在 RViz 中点击 `2D Nav Goal`。

方式 2：使用 `NavRLTerminalControl` 的菜单 1，输入目标 `x y`。

方式 3：直接发布一个目标点：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{
  header: {frame_id: "map"},
  pose: {
    position: {x: 3.0, y: 0.0, z: 1.0},
    orientation: {w: 1.0}
  }
}'
```

## 4. 单 UGV 仿真快速启动

这个流程对应启动面板中的“NavRL 单无人车避障验证”。UGV 链路不需要启动 `sunray_uav_control`，`NavRL2SunrayUGV_sim` 会把 NavRL 输出转换成 `/ugv1/sunray/ugv_control/cmd_vel`。

### 终端 1：启动 sunray_sim 单 UGV

```bash
roslaunch sunray_sim sunray_sim.launch uav_num:=0 ugv_num:=1 ugv1_init_z:=1.0
```

`ugv1_init_z:=1.0` 是为了兼容当前 NavRL 的三维射线/高度逻辑。

### 终端 2：启动 UGV 适配链路

```bash
roslaunch sunray_navrl_adapter NavRL2SunrayUGV_sim.launch rviz:=true
```

默认话题关系：

- 订阅 odom：`/ugv1/sunray_sim/odom`
- 订阅局部点云：`/ugv1/sunray_sim/cloud_sensor_frame`
- 订阅 NavRL 原始速度：`/CERLAB/quadcopter/cmd_vel`
- 发布 UGV 速度：`/ugv1/sunray/ugv_control/cmd_vel`

### 终端 3：启动 NavRL 策略节点

```bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

当前 `navigation.py` 已经直接订阅 `/ugv1/sunray_sim/odom`。如果使用旧版本脚本，只订阅了 UAV odom，可以使用兼容写法：

```bash
rosrun navigation_runner navigation_node.py /uav1/sunray_sim/odom:=/ugv1/sunray_sim/odom
```

CPU 环境同样可以显式覆盖：

```bash
rosrun navigation_runner navigation_node.py device=cpu sim.device=cpu sim.use_gpu=false
```

UGV 目标点建议直接用 RViz 的 `2D Nav Goal`，或用 `rostopic pub` 发布 `/move_base_simple/goal`。

## 5. 真机或外部数据接入

真机启动前应先确认外部链路已经提供以下数据：

- 里程计：默认 `/uav1/sunray/uav_odom`
- FAST-LIO body 系点云：默认 `/cloud_registered_body`
- 雷达 IMU：默认 `/livox/imu`
- Sunray 控制入口：默认 `/uav1/sunray/uav_control_cmd`
- UAV 控制器、定位、驱动和安全策略已经按真机流程准备好

如果 MID360 相对机体存在固定 roll/pitch 安装倾角，启动 NavRL 适配链路时启用
独立点云校平节点：

```bash
roslaunch sunray_navrl_adapter NavRL2Sunray.launch \
  enable_body_point_cloud_leveler:=true \
  point_cloud_topic:=/cloud_registered_body_aligned \
  rviz:=false
```

校平节点会在启动阶段用 `/livox/imu` 的前 200 帧估计固定 roll/pitch，并将
`/cloud_registered_body` 旋转后发布到 `/cloud_registered_body_aligned`。采样期间
必须让无人机静止且机体水平；标定完成前不会向 NavRL 地图发布点云。

该重力校平不能估计雷达相对机体的 yaw 安装误差，也不处理雷达原点到机体中心的
平移。存在这些误差时，仍需通过机械测量或外参标定补齐。

再启动策略节点：

```bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

如果真机不是 `uav1`，需要同步修改或 remap：

- `NavRL2Sunray.launch` 的 `agent_name`、`agent_id`、`odom_topic`、`point_cloud_topic`、`control_cmd_topic`
- `navigation.py` 中当前硬编码订阅的 odom 话题，或使用 ROS remap

真机联调前建议先把 `rviz:=true` 打开，确认 odom、点云、占据地图和目标点坐标系一致，再接入真实控制输出。

## 6. 常用检查命令

确认 ROS 包可见：

```bash
rospack find navigation_runner
rospack find map_manager
rospack find sunray_navrl_adapter
rospack find sunray_sim
```

确认仿真 UAV 数据：

```bash
rostopic hz /uav1/sunray_sim/odom
rostopic hz /uav1/sunray_sim/cloud_sensor_frame
```

确认仿真 UGV 数据：

```bash
rostopic hz /ugv1/sunray_sim/odom
rostopic hz /ugv1/sunray_sim/cloud_sensor_frame
```

确认 map_manager 服务：

```bash
rosservice list | grep occupancy_map
rosservice list | grep raycast
```

确认 NavRL 策略输出：

```bash
rostopic echo /CERLAB/quadcopter/cmd_vel
rostopic echo /CERLAB/quadcopter/setpoint_pose
```

确认 Sunray UAV 控制输入：

```bash
rostopic echo /uav1/sunray/uav_control/control_cmd
```

确认 Sunray UGV 控制输入：

```bash
rostopic echo /ugv1/sunray/ugv_control/cmd_vel
```

## 7. 常见问题

### 只启动了 NavRL2Sunray，但没有动作

`NavRL2Sunray*.launch` 不启动策略推理节点。需要单独执行：

```bash
conda activate NavRL
rosrun navigation_runner navigation_node.py
```

### navigation_node 一直等待 odom

检查对应 odom：

```bash
rostopic hz /uav1/sunray_sim/odom
rostopic hz /uav1/sunray/localization/local_odom
rostopic hz /ugv1/sunray_sim/odom
```

如果是自定义机号，检查 `navigation.py` 的硬编码订阅或使用 ROS remap。

### 有 odom 但不发布速度

先确认是否已经发布目标点：

```bash
rostopic echo /move_base_simple/goal
```

再确认 `occupancy_map/raycast` 服务是否存在。`navigation_node.py` 在没有目标点或 raycast 数据时会持续发布起飞/保持位姿，不会进入策略速度输出。

### CPU 环境报 CUDA 错误

当前配置文件默认可能是 `device: "cuda:0"`。CPU 环境启动时使用：

```bash
rosrun navigation_runner navigation_node.py device=cpu sim.device=cpu sim.use_gpu=false
```

也可以直接修改：

- `navigation_runner/scripts/cfg/train.yaml` 中的 `device`
- `navigation_runner/scripts/cfg/sim.yaml` 中的 `sim.device` 和 `sim.use_gpu`

### 重复启动 map_manager

不要同时启动 `navigation_runner/launch/safety_and_perception_*.launch` 和 `sunray_navrl_adapter/NavRL2Sunray*.launch`，它们都会启动地图相关节点，容易出现节点名或服务冲突。Sunray 适配流程优先使用 `sunray_navrl_adapter` 下的 launch。

### 误按 navigation_node 终端的 Enter

`navigation_node.py` 的终端 Enter 是人工暂停/继续开关。第一次 Enter 会停止运动，第二次 Enter 才会继续。

## 8. 使用启动面板

如果使用 Sunray 启动面板，`tools/sunray_launcher_panel/config/sunray_quick_launch_groups.yaml` 中已经有两个 NavRL 快速流程：

- `NavRL 单无人机避障验证`
- `NavRL 单无人车避障验证`

启动面板会启动大部分 ROS 链路，但 `navigation_node.py` 仍需要按提示在新终端手动启动，因为它需要 Conda 环境，并且终端 Enter 被用作人工暂停/继续开关。
