# sunray_test

`sunray_test` 是 Sunray 的自动化测试模块。当前只保留 Dashboard 这套入口：

- 人工入口：`tests/run_test.sh`
- ROS 入口：`rosrun sunray_test run_test_dashboard.py`

旧的 `run_suite.py`、`run_scenario.py`、`internal_run.py`、`show_config.py` 入口和
`config/suites`、`config/scenarios` 配置已废弃并移除。Dashboard 会根据用户选择生成运行期
suite，再通过 `run_test_dashboard.py --run-suite --suite-file ...` 内部模式启动执行器；
用户不需要直接调用内部模式。

测试产物默认输出到 `tests/output/<timestamp>/`，本次运行生成的 suite 写入同一目录下的 `suite.yaml`。

## 使用

```bash
source devel/setup.bash

tests/run_test.sh
tests/run_test.sh --sim
tests/run_test.sh --list
tests/run_test.sh --check-config
tests/run_test.sh --history
tests/run_test.sh --open-latest-report
tests/run_test.sh --items visual_landing --dry-run --no-bringup --no-prompt
tests/run_test.sh --items visual_landing --show-suite --no-prompt
tests/run_test.sh --sim --items ego_goal
tests/run_test.sh --sim --items hover --profile sunray150_lidar
```

首次运行 `tests/run_test.sh` 会先按 `config/runtime_deps.json` 检查运行依赖，缺少
`PyYAML`、`numpy`、ROS Python 模块或 `tcpdump` 时会尝试通过 apt/pip 安装；不会自动重新
编译工作空间。需要跳过依赖检查时：

```bash
SUNRAY_TEST_SKIP_DEP_CHECK=1 tests/run_test.sh
```

默认环境是 `exp`；只有带 `--sim` 时才拉起 Gazebo 仿真链路。Dashboard 固定使用 `uav_id=1`。

不带 `--items` 时会进入全屏 TUI。界面上方依次是功能测试、硬件测试和工具，下方实时预览
测试步骤和启动链路。功能测试会自动补齐依赖硬件，例如
`visual_landing` 补 `battery + down_camera`，`ego_goal` 补 `battery + lidar`。
用户也可以额外选择某个硬件测试。停在某个测试项目上按 `Tab` 可以进入该项目的参数设置页；
参数修改只影响本次生成的 suite，不会写回配置文件。
顶部的 `external_source` 会传给 `external_fusion.launch external_source:=...`，实机基础款默认 `MOCAP(3)`，雷达款默认 `ODOM(0)`；如果手动修改过定位源，则以本次手动选择为准。
短按 `e` 在 `ODOM(0)` 和 `MOCAP(3)` 间切换；长按 `e` 打开完整列表：
`ODOM(0)`、`POSE(1)`、`GAZEBO(2)`、`MOCAP(3)`、`VIOBOT(4)`、`GPS(5)`、`RTK(6)`、
`Mini viobot(7)`。
机型默认按测试项目自动推导；选择雷达或 EGO-Planner 时会自动使用 `sunray150_lidar`，
其他情况默认 `sunray150_basic`。如果实际机型和测试项目不一致，例如雷达款只测试悬停，
可在 TUI 中按 `m` 手动切换为基础款或雷达款。机型只影响平台默认参数、评分和仿真模型；
是否拉起雷达 driver、mapping、EGO 仍然由测试项目是否需要雷达链路决定。
工具区内嵌雷达IP自动配置和 VRPN 服务器检查，不参与 suite 生成；两个工具的
`rosrun` 独立启动方式仍然保留。

TUI 快捷键：

- `←` / `→`：在功能测试、硬件测试、工具三个区域之间切换
- `q`：退出

测试项目区域快捷键：

- `↑` / `↓`：循环移动当前测试项目光标
- `Space`：勾选或取消当前测试项
- `Tab`：进入当前测试项参数设置
- `m`：在自动推导、基础款、雷达款之间切换机型
- `e`：短按切换定位源，长按打开完整列表
- `a` / `c`：当前测试分组全选或清空
- `Enter`：生成 suite 并启动测试

工具区域快捷键：

- `↑` / `↓`：循环选择工具
- `Enter`：进入当前工具页

工具页快捷键：

- `Tab` / `Esc`：返回工具列表；VRPN 输入或确认中会先取消当前编辑
- `VRPN`：`↑` / `↓` 选择目标，`Space` 输入新 IP，`Enter` 确认或写入
- `MID360`：`Enter` 启动，`Space` 停止，日志默认跟随最新输出，`↑` / `↓` 滚动日志

参数页快捷键：

- `↑` / `↓`：选择参数
- `←` / `→`：按参数步进值调整数值，或切换布尔/枚举值
- `Enter`：手动输入当前参数值
- `Backspace`：恢复当前参数默认值
- `Tab` / `Esc`：返回测试项目列表

每个参数的中文名、类型、单位、默认值、步进范围和详细含义都写在
`config/dashboard/dashboard.yaml` 的 `param_schema` 中。Dashboard 生成 suite 时会把本次修改
merge 到对应测试项的 `step.params`。

如果当前终端不支持全屏 TUI，请使用 `--items` 指定测试项目。带 `--items` 时跳过交互，直接按命令行指定的项目生成计划。

## 参数说明

Dashboard 参数页只编辑本次运行生成的 suite，不会回写配置文件。参数页内带 `*` 的值表示已经偏离默认值。

相机测试参数，`front_camera` 和 `down_camera` 共用：

- `topic_key`：平台和环境配置里的相机 topic 键名，前视默认 `front_camera`，下视默认 `down_camera`。
- `timeout_s`：等待首帧图像的最长时间，节点启动慢时调大。
- `sample_duration_s`：收到首帧后继续采样图像流质量的时间窗口。
- `min_messages`：采样窗口内至少需要收到的图像帧数。
- `min_rate_hz`：图像平均发布频率的最低通过阈值。
- `max_gap_s`：相邻两帧允许的最大间隔，用于发现卡顿。
- `require_non_uniform_frame`：要求图像不是纯色或几乎无纹理，默认开启。
- `max_identical_frame_ratio`：允许重复帧占比上限，用于冻结画面判断。
- `require_timestamp_progress`：要求图像 header 时间戳递增，默认开启。
- `require_frame_content_change`：要求采样期间画面内容变化，静态场景容易误判，默认关闭。
- `black_mean_threshold`：黑屏判断的平均亮度阈值。
- `black_dynamic_range_threshold`：黑屏判断的像素动态范围阈值。
- `max_black_frame_ratio`：允许黑屏候选帧占比上限。
- `device_path`：可选的本机设备路径，例如 `/dev/video0`，填写后会先检查设备是否存在。

电池测试参数：

- `topic_key`：平台和环境配置里的电池 topic 键名，默认 `battery`。
- `timeout_s`：等待电池消息的最长时间。
- `pass_threshold_v`：电池电压通过阈值，低于该值判定失败。

激光雷达测试参数：

- `imu_topic_pattern`：用于匹配 MID360 IMU topic 的关键字或模式。
- `lidar_topic_pattern`：用于匹配 MID360 点云 topic 的关键字或模式。
- `timeout_s`：等待 IMU 和点云 topic 出现的最长时间。
- `sample_duration_s`：统计消息频率、间隔和点云数量的采样窗口。
- `min_messages`：采样窗口内要求收到的最少 IMU 或点云消息数。
- `min_rate_hz`：雷达消息平均频率的最低通过阈值。
- `max_gap_s`：相邻消息允许的最大间隔。
- `min_points_per_cloud`：单帧点云需要达到的最低点数。
- `min_valid_clouds`：采样窗口内需要达到点数要求的点云帧数量。

悬停测试参数：

- `duration_s`：起飞后保持当前位置的测试时长。

EGO-Planner 自主规划参数：

- `goal_topic`：发布 EGO 目标点的 ROS topic。
- `frame_id`：目标点 `PoseStamped` 使用的坐标系。
- `mission_key`：从 `config/missions` 选择的目标点列表。
- `z_m`：目标点没有有效高度时使用的默认高度。
- `goal_source`：目标点来源，`list` 读取任务列表，`input` 从 rviz 输入。
- `timeout_s`：单个目标点允许飞行的最长时间。
- `stable_time_s`：进入到点半径后必须连续保持的判稳时间。
- `hold_time_s`：目标点判稳通过后的额外保持时间。
- `reach_radius_m`：判断到达目标点的距离阈值。
- `publish_burst_count`：每个目标点开始时连续发布目标消息的次数。
- `publish_burst_interval_s`：连续发布目标消息之间的间隔。
- `use_xy_only`：只按水平距离判断到点，不把高度误差计入距离。
- `keepalive_enabled`：持续把 EGO `pos_cmd` 转成控制指令。
- `keepalive_rate_hz`：keepalive 控制指令发送频率。
- `keepalive_stale_timeout_s`：`pos_cmd` 超时未更新后停止转发旧指令的时间。
- `keepalive_zero_velocity_epsilon`：判断规划输出速度接近 0 的阈值。
- `post_transition_enabled`：EGO 测试结束后是否执行姿态和高度过渡。
- `post_transition_target_yaw_rad`：结束过渡的目标偏航角，单位 rad。
- `post_transition_yaw_rate_rad_s`：结束过渡的偏航角速度。
- `post_transition_hold_after_s`：结束过渡后继续保持的时间。
- `post_transition_target_z_m`：结束过渡的目标高度。
- `pos_cmd_topic`：keepalive 订阅的 EGO 位置控制指令 topic。
- `control_cmd_topic`：keepalive 转发到 Sunray 控制链路的控制 topic。

指点飞行参数：

- `mission_key`：从 `config/missions` 选择的航点列表。
- `waypoint_source`：航点来源，`list` 读取任务列表，`input` 运行时逐个输入。
- `timeout_s`：单个航点允许飞行的最长时间。
- `stable_time_s`：进入航点半径后必须连续保持的判稳时间。
- `hold_time_s`：航点判稳通过后的额外保持时间。
- `reach_radius_m`：判断到达航点的距离阈值。

视觉降落参数：

- `launch_file`：视觉降落 case 内部启动的 launch 文件。
- `auto_takeoff`：传给视觉降落 launch 的自动起飞开关，dashboard 已统一起飞，通常保持 `false`。
- `height_m`：视觉降落开始前无人机保持或上升到的高度。
- `launch_args.error_xy`：允许进入最终下降的水平误差阈值。
- `launch_args.error_z`：视觉降落控制允许的高度误差阈值。
- `launch_args.land_vel`：识别目标后最终下降的速度。
- `launch_args.last_land_time`：最终降落阶段持续发送降落控制的时间。
- `launch_args.drop_height`：视觉跟踪阶段每次下发的相对下降高度，数值越大下降越快。

## 入口参数

- `--sim`：使用仿真环境并拉起 sim bringup
- `--items visual_landing,ego_goal`：跳过交互，直接指定测试项目
- `--profile sunray150_basic|sunray150_lidar`：手动指定机型；不填时按测试项目自动推导
- `--check-config`：校验 dashboard 配置并输出摘要
- `--history --history-limit 5`：查看最近测试结果
- `--open-latest-report`：打开最近一次 HTML 报告
- `--show-suite`：只打印生成的运行期 suite，不写入文件，不启动测试
- `--dry-run --no-bringup`：只预览 suite、bringup tabs 和 runner 命令
- `--no-prompt --sn ... --tester ...`：跳过 SN/测试人员交互
`--dry-run`、`--show-suite` 是互斥动作，并且不会要求 `gnome-terminal`。

## 架构

Dashboard 负责三件事：

- 根据 `config/dashboard/dashboard.yaml` 的测试项、依赖和运行规则生成运行期 suite
- 按 `sim/exp` 和测试需求生成 bringup tabs
- 写入 suite 后通过内部 runner 模式执行 `TestRunner`

执行器逻辑仍然在 `src/sunray_test/core/runner.py`，配置加载和校验在
`src/sunray_test/core/suite_loader.py`。这样入口收敛为一套，但硬件测试、功能测试、phase、
报告生成仍然保持解耦。

## 目录职责

- `config/dashboard/`
  Dashboard 测试项、依赖关系、runtime profile、bringup tabs、runner tabs
- `config/platforms/`
  平台默认参数、topic、录包 topic、分析参数
- `config/environments/`
  `sim` / `exp` 环境差异和 topic 覆盖
- `config/missions/`
  航点和 EGO 目标点等飞行任务数据
- `config/scoring/`
  飞行评分权重、门槛和等级阈值
- `src/sunray_test/cases/`
  硬件测试和功能测试 case
- `src/sunray_test/phases/`
  起飞、降落等复用阶段
- `src/sunray_test/core/`
  runner、上下文、suite 加载和结果模型
- `src/sunray_test/dashboard/`
  Dashboard CLI、全屏 TUI、模型、终端启动、运行期 suite 和历史结果
- `src/sunray_test/reports/`
  飞行分析、评分和 HTML 报告

## 扩展测试项

新增硬件或功能测试时按这个边界改：

1. 在 `src/sunray_test/cases/` 增加 case，并在 registry 中注册。
2. 如果需要起飞/降落等阶段动作，放在 `src/sunray_test/phases/`。
3. 在 `config/dashboard/dashboard.yaml` 增加测试项、显示名、分组、依赖、suite step。
4. 如果功能测试依赖额外 bringup，在同一个 dashboard 配置里给对应 tab 加 `when` 条件。
5. 如果需要任务点或目标点，把数据放进 `config/missions/`。
6. 如果需要评分，扩展 `config/scoring/scoring.yaml` 和 `src/sunray_test/reports/`。

硬件测试可以单独选择；功能测试必须声明所需硬件依赖，由 Dashboard 自动补齐。

## 安装和编译

从仓库根目录执行：

```bash
./build.sh sunray_test
source devel/setup.bash
```

运行依赖由 `tests/run_test.sh` 首次启动时按 `config/runtime_deps.json` 自动检查。其中
`tcpdump` 用于雷达 IP 自动配置工具的抓包检测；如果不想自动检查依赖，可设置
`SUNRAY_TEST_SKIP_DEP_CHECK=1`。

## VRPN 检查

实机运行前如需确认 VRPN 服务器 IP，可以在 Dashboard 工具区操作，也可以直接运行：

```bash
source devel/setup.bash
rosrun sunray_test vrpn_server_check.py
```

如需直接写入：

```bash
rosrun sunray_test vrpn_server_check.py --server 192.168.xx.xx --yes
rosrun sunray_test vrpn_server_check.py --target vrpn --server 192.168.xx.xx --yes
```

## sim / exp 话题约定

- `sim`
  - 前视相机：`/uav{uav_id}/monocular_front/image_raw`
  - 下视相机：`/uav{uav_id}/monocular_down/image_raw`
- `exp`
  - 前视相机：`/web_cam_front/image_raw`
  - 下视相机：`/web_cam/image_raw`

具体话题以 `config/environments/*.yaml` 为准。
