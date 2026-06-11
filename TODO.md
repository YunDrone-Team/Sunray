# Sunray 面向新手用户的代码与工程改进 TODO

本文从“有一点 ROS 基础、但不熟悉 PX4/Sunray 的学生”视角整理。目标不是重写架构，而是降低第一次编译、第一次启动、第一次读示例、第一次接入自己算法时的踩坑率。

优先级说明：

- P0：强烈建议优先做。会直接影响新手能否跑通或是否容易误操作。
- P1：建议做。主要提升可理解性、可维护性和二次开发效率。
- P2：有时间再做。偏工程规范、长期维护和体验优化。

## P0：先解决新手最容易卡住的问题

### 1. 建立真正的“一键冒烟测试”

现状：仓库有大量脚本和 launch，但新手很难判断“我现在环境是否正确、编译是否完整、核心话题是否正常”。  
建议：

- 新增 `scripts/check_sunray_env.sh` 或类似入口，只做检查，不启动危险任务。
- 检查 ROS 版本、`source devel/setup.bash`、核心包是否可 `rospack find`、关键消息是否生成。
- 检查 `Simulation` 子模块/目录是否存在并提示如何初始化。
- 检查 `gnome-terminal`、`tmux`、MAVROS、Gazebo、RViz、yaml-cpp、octomap、Livox 驱动等可选依赖。
- 输出明确结论：`可运行基础 UAV 仿真`、`缺少仿真模块`、`只能编译核心接口`、`缺少实机依赖`。

验收标准：

- 新手执行一个命令就能知道当前机器适合跑哪个 demo。
- 失败信息必须给出下一步命令或文档链接。

### 2. 给快速启动脚本增加参数化与无 GUI 模式

现状：`scripts_sim`、`scripts_exp`、`scripts_swarm` 大多硬编码 `gnome-terminal`、固定 `uav_id:=1`、固定 sleep 时间。地面站调用、远程 SSH、无桌面环境下容易失败。  
建议：

- 抽出公共启动函数，例如 `scripts/lib/sunray_launch.sh`。
- 所有脚本统一支持 `--uav-id`、`--ugv-id`、`--external-source`、`--no-gui`、`--dry-run`。
- `--dry-run` 只打印将要执行的 `roslaunch` 链路，方便新手理解。
- `--no-gui` 使用后台进程或 tmux，避免强依赖 `gnome-terminal`。
- 保留现有脚本名，内部转调公共函数，避免破坏地面站已有配置。

验收标准：

- `bash scripts_sim/demo_takeoff_hover_land.sh --dry-run` 能打印完整启动链路。
- SSH 无图形环境下能用 `--no-gui` 启动基础链路。

### 3. 清理硬编码绝对路径

现状：多个文件包含固定用户路径或旧路径，换机器后会直接误导或编译失败。已观察到的高风险点：

- `General_Module/sunray_ugv_control/CMakeLists.txt` 中写死 `/home/yundrone/app/octomap_msgs/...`。
- `server/communication.sh` 中写死 `/home/PRR/Sunray/devel/setup.sh`。
- `General_Module/sunray_planner_utils/launch/goal2swarm.launch` 和 `goal2swarm_ugv.launch` 默认路径指向不存在/旧包名的绝对路径。
- `General_Module/sunray_planner_utils/src/goal2swarm.cpp` 默认 waypoint 路径写死。
- `General_Module/sunray_gimbal/demo/image_saver.cpp` 默认保存到 `/home/yundrone/saved_images/`。

建议：

- ROS 内路径统一用 `$(find package_name)`。
- C++ 默认路径通过 ROS 参数传入，不写死用户 home。
- CMake 依赖通过 `find_package` 和 `package.xml` 声明，不指向本机 build 目录。
- server 脚本通过 `SUNRAY_HOME=${SUNRAY_HOME:-$HOME/Sunray}` 组织路径。

验收标准：

- 将仓库放到任意 home 路径下，基础编译和基础脚本不需要手动改路径。

### 4. 修正会误导新手的命名和注释

现状：存在拼写、旧命名和注释不一致，新手会直接照抄。建议先修正“入口层”问题：

- `RUN_SEVER` 建议兼容保留，同时新增正确变量名 `RUN_SERVER`。
- `rivz_config` 改为 `rviz_config`，旧参数可短期兼容。
- `scripts_exp/demo_hexayon*.sh` 统一为 `demo_hexagon*.sh`，旧脚本保留转发。
- `scripts_swarm/leader_fowllower_sim_3uav.sh` 统一为 `leader_follower_sim_3uav.sh`。
- `General_Module/sunray_tutorial/scripts/squre.py` 统一为 `square.py`。
- `positionCmd2sunray.cpp` 文件头注释中的 `egp-planner` 改为 `ego-planner`。
- Python UAV 示例里“等待 FMT 连接”改为“等待 UAV/PX4 连接”。
- `UAVControlCMD.msg` 中 `latitude`/`longitude` 注释疑似写反，应核对后修正。

验收标准：

- 新手看到的脚本名、参数名、注释和实际行为一致。
- 旧名字在一个过渡版本内仍可运行，并打印 deprecation 提示。

### 5. 为实机启动增加安全前置检查

现状：实机脚本可以直接启动任务节点，新手可能在定位未稳定、围栏不合理、遥控器接管未确认时运行。  
建议：

- 实机脚本默认只启动 MAVROS、external fusion、control node，不自动执行任务 demo，除非显式 `--run-demo`。
- 增加 `scripts_exp/preflight_check.sh`：
  - MAVROS connected。
  - `uav_state.odom_valid == true`。
  - `uav_state.armed == false` 初始安全。
  - 地理围栏参数不是过大默认值。
  - 遥控器/地面站急停通道已确认。
- 起飞类示例加二次确认，或要求传入 `--i-understand-real-flight-risk`。

验收标准：

- 实机脚本在安全条件不满足时拒绝执行任务节点。
- 错误输出能指出具体缺哪一项。

## P1：提升二次开发可读性和可维护性

### 6. 给新任务开发提供模板包/模板节点

现状：学生通常复制 `sunray_tutorial` 示例，但示例既承担教学又承担功能演示，容易把无关代码一起复制。  
建议：

- 新增 `General_Module/sunray_tutorial/template/`：
  - `uav_task_template.cpp`
  - `uav_task_template.py`
  - `ugv_task_template.cpp`
  - `ugv_task_template.py`
- 模板只保留：参数读取、状态订阅、控制发布、安全退出、超时保护、Hover/Land fallback。
- 在模板顶部写清楚“应该改哪里”和“不建议改哪里”。
- 提供 `roslaunch sunray_tutorial task_template.launch`，可直接运行空任务。

验收标准：

- 学生从模板复制后，只需改一个 `runTask()` 或状态机函数。

### 7. 抽象 Python 版控制工具类

现状：C++ 有 `Control_Utils`，Python 示例里起飞、切模式、解锁、降落逻辑重复，且注释存在旧 FMT 表述。  
建议：

- 新增 `sunray_tutorial/uav_python/sunray_control_utils.py`。
- 封装 `wait_connected()`、`set_cmd_control()`、`arm()`、`takeoff()`、`hover()`、`land()`。
- 所有 Python UAV 基础示例复用这个工具类。
- 给每个等待函数加 timeout 和失败返回，不要无限等待。

验收标准：

- Python 示例代码缩短，任务逻辑更突出。
- 连接失败、定位无效、无法解锁时能退出并打印原因。

### 8. 给核心消息增加“使用场景表”和字段单位校验

现状：`sunray_msgs/msg` 注释较多，但新手仍难判断哪个字段必须填、哪个字段会被忽略。  
建议：

- 对 `UAVControlCMD`、`UAVSetup`、`UAVState`、`UGVControlCMD` 增加更结构化注释。
- 在文档或代码中建立“控制模式 -> 必填字段 -> 坐标系 -> 单位”的表。
- 可新增轻量运行时检查工具，例如任务节点发布前调用 `validate_uav_cmd(cmd)`，发现 NaN、模式字段缺失、速度过大时报警。

验收标准：

- 新手能明确知道 `XyzPosYaw`、`XyzVelYaw`、`XyVelZPosYawBody` 各该填哪些字段。

### 9. 将 launch 参数集中成 YAML 配置

现状：`sunray_control_node.launch`、`external_fusion.launch` 等参数很多，且默认值直接散在 launch 文件里。  
建议：

- 为 UAV 控制、外部定位、UGV 控制各提供默认 YAML。
- launch 只负责加载 YAML 并允许少量参数覆盖。
- 增加 `config/safe_defaults.yaml` 和 `config/sim_defaults.yaml`，区分仿真和实机。
- 对关键参数加注释：围栏、起飞高度、降落速度、定位超时、cmd timeout。

验收标准：

- 新手调参时主要改 YAML，不需要读长 launch。
- 仿真和实机默认参数明确分开。

### 10. 拆分并标注“自研代码”和“外部代码”

现状：`External_Module` 中混合了 EGO、FUEL、VINS、雷达、底盘、检测等外部工程，新手不知道该读哪里、该不该改。  
建议：

- 在每个外部模块根目录放一个 `SUNRAY_NOTES.md`。
- 写清楚：
  - 上游项目来源。
  - Sunray 改动点。
  - 推荐只看的 launch/topic/interface。
  - 不建议新手修改的内部算法目录。
- 在根 README 中给出“二次开发优先读 General_Module，外部模块按需接入”的明确说明。

验收标准：

- 学生不会一上来钻进 EGO/FUEL/VINS 内部实现。

### 11. 统一日志与错误提示风格

现状：部分节点使用 `Logger::print_color`，部分使用 `ROS_INFO`，脚本中输出格式也不统一。  
建议：

- 统一错误级别：INFO/WARN/ERROR/FATAL。
- 等待类日志使用 throttle，避免刷屏。
- 关键失败必须输出“原因 + 建议检查命令”。
- 脚本输出统一前缀，例如 `[sunray][preflight]`、`[sunray][launch]`。

验收标准：

- 新手看终端能快速定位是哪个节点、哪个话题、哪个参数出了问题。

### 12. 规划器桥接逻辑补充边界处理

现状：`positionCmd2sunray` 对 `control_type`、`vehicle_type`、重复发布等逻辑可读性较弱，UGV 分支存在先 publish 再判断重复的结构。  
建议：

- 明确 `control_type` 枚举和非法值处理。
- `vehicle_type` 使用小写局部变量命名风格，避免 `Vehicle_type`。
- 对输入 `PositionCommand` 做 NaN/inf 检查。
- 对规划器长时间无输出增加 Hover/Stop fallback。
- 对 UAV/UGV 分支分别拆成函数，降低新手阅读成本。

验收标准：

- 非法参数不会静默 return，必须输出错误。
- 规划器断流时系统行为可预测。

## P2：长期工程质量和发布体验

### 13. package.xml 和许可证元信息补齐

现状：多个包的 `package.xml` 仍有 `<license>TODO</license>`。  
建议：

- 自研包统一填写实际许可证。
- 外部包保留上游许可证，不随意改。
- 根 README 明确“自研代码”和“第三方代码”的许可边界。

验收标准：

- 所有自研包 `package.xml` 无 `TODO` 元信息。

### 14. 建立 CI/本地检查脚本

建议新增 `scripts/dev_check.sh`，至少做：

- `bash -n` 检查所有 shell 脚本。
- 检查 launch XML 是否可解析。
- 检查 `package.xml` 是否存在 TODO。
- 检查是否有高风险绝对路径。
- 检查文档中的脚本名是否真实存在。

注意：用 `bash -n` 检查 shell，而不是 `sh -n`，因为当前脚本大量使用 bash 语法。

验收标准：

- 维护者提交前能用一个命令发现拼写、路径和脚本语法问题。

### 15. 快速启动脚本生成化

现状：大量脚本只是不同 launch 组合，重复多、命名容易错。  
建议：

- 使用 YAML 描述启动场景：节点名、延迟、launch、参数、是否仿真/实机。
- 自动生成 `scripts_sim/*.sh` 和 `scripts_exp/*.sh`，或由一个统一 runner 执行 YAML。
- 地面站快速启动也读取同一份场景配置，避免脚本和地面站菜单不一致。

验收标准：

- 新增一个 demo 不需要手写多个 shell 文件。

### 16. 明确支持矩阵

建议在根 README 或 `docs/` 中维护一张支持矩阵：

- Ubuntu/ROS 版本。
- PX4/MAVROS 版本。
- 仿真是否需要 Simulation 子模块。
- 支持的定位源：Gazebo、动捕、VIOBOT、GPS、RTK、自定义 odom。
- 支持的硬件：Sunray UAV、FMT、UGV、Mid360、Viobot、云台。

验收标准：

- 新手能在安装前判断自己的环境是否匹配。

### 17. 控制接口增加最小示例测试

建议为核心接口增加不依赖真实飞控的测试：

- 发布一条 `UAVControlCMD`，验证控制节点是否收到并更新内部状态。
- 发布模拟 `PX4State`，验证任务节点等待逻辑能通过。
- 对 `Control_Utils` 的 timeout、安全退出做测试。
- 对 `positionCmd2sunray` 输入 `PositionCommand`，验证输出模式与字段。

验收标准：

- 改消息、改控制桥、改示例时能快速发现破坏兼容性的改动。

### 18. 统一命名约定和目录约定

建议形成 `CONTRIBUTING.md`：

- 脚本命名：`demo_<task>_<source>.sh`。
- launch 参数命名：统一小写下划线。
- 话题命名：统一 `/uavX/sunray/...`、`/ugvX/sunray_ugv/...`。
- 坐标系：明确 ENU、Body、GPS/RTK 的命名和转换边界。
- 示例文件：C++/Python 同名同义。

验收标准：

- 后续新示例不会继续扩大命名差异。

## 建议实施顺序

1. 先做 P0-1、P0-3、P0-4：让新手能诊断环境，减少路径/命名坑。
2. 再做 P0-2、P0-5：让启动脚本更适合地面站、SSH 和实机安全流程。
3. 然后做 P1-6、P1-7、P1-8：让学生写新任务更容易。
4. 最后做 P1-9 到 P2：提高长期维护质量。

## 当前不建议优先做的事

- 不建议马上大规模重构 `sunray_uav_control` 状态机。它是核心稳定层，先通过文档、模板、校验和参数整理降低使用门槛。
- 不建议让新手直接修改 EGO/FUEL/VINS 内部算法。更应先稳定 Sunray 与这些外部算法之间的话题边界。
- 不建议先追求全量自动化测试。先建立冒烟检查和关键接口测试，收益更直接。
