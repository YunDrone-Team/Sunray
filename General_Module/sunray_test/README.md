# sunray_test

`sunray_test` 是 Sunray 的自动化测试模块。当前只保留 Dashboard 这套入口：

- 人工入口：`tests/run_test.sh`
- ROS 入口：`rosrun sunray_test run_test_dashboard.py`

旧的 `run_suite.py`、`run_scenario.py`、`internal_run.py`、`show_config.py` 入口和 `config/suites`、`config/scenarios` 配置已废弃并移除。Dashboard 会根据用户选择生成运行期 suite，再通过 `run_test_dashboard.py --run-suite --suite-file ...` 内部模式启动执行器；用户不需要直接调用内部模式。

测试产物默认输出到 `tests/output/<timestamp>/`，运行期 suite 写入 `tests/output/generated_suites/`。

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
tests/run_test.sh --items visual_landing --write-suite-only --no-prompt
tests/run_test.sh --sim --items ego_goal
```

默认环境是 `exp`；只有带 `--sim` 时才拉起 Gazebo 仿真链路。Dashboard 固定使用 `uav_id=1`。

交互流程先选择硬件测试，再选择功能测试。功能测试会自动补齐依赖硬件，例如 `visual_landing` 补 `battery + down_camera`，`ego_goal` 补 `battery + lidar`。用户也可以额外选择某个硬件测试。

## 入口参数

- `--sim`：使用仿真环境并拉起 sim bringup
- `--items visual_landing,ego_goal`：跳过交互，直接指定测试项目
- `--check-config`：校验 dashboard 配置并输出摘要
- `--history --history-limit 5`：查看最近测试结果
- `--open-latest-report`：打开最近一次 HTML 报告
- `--show-suite`：只打印生成的运行期 suite，不写入文件，不启动测试
- `--write-suite-only`：只写入运行期 suite，不启动测试，并打印内部 runner 命令
- `--dry-run --no-bringup`：只预览 suite、bringup tabs 和 runner 命令
- `--no-prompt --sn ... --tester ...`：跳过 SN/测试人员交互
- `--yes`：跳过最终启动确认

`--dry-run`、`--show-suite`、`--write-suite-only` 是互斥动作，并且不会要求 `gnome-terminal`。

## 架构

Dashboard 负责三件事：

- 根据 `config/dashboard/dashboard.yaml` 的测试项、依赖和运行规则生成运行期 suite
- 按 `sim/exp` 和测试需求生成 bringup tabs
- 写入 suite 后通过内部 runner 模式执行 `TestRunner`

执行器逻辑仍然在 `src/sunray_test/core/runner.py`，配置加载和校验在 `src/sunray_test/core/suite_loader.py`。这样入口收敛为一套，但硬件测试、功能测试、phase、报告生成仍然保持解耦。

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
  Dashboard CLI、模型、交互、终端启动、运行期 suite 和历史结果
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
bash General_Module/sunray_test/setup_sunray_test.sh
source devel/setup.bash
```

该脚本会检查并安装 Python 依赖，然后编译 `sunray_test`。

## VRPN 检查

实机运行前如需确认 VRPN 服务器 IP：

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
