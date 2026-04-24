# sunray_test

`sunray_test` 是 Sunray 的自动化测试模块，负责：

- 按 `platform + environment + suite` 组合生成生效配置
- 执行硬件检查、起飞/降落 phase、飞行 case
- 产出 `test_result.json`、`event_log.jsonl`、`report.html`
- 对飞行数据做分析、评分和 HTML 报告渲染

当前整体分两层：

- `tests/`
  面向人工入口和兼容包装
- `General_Module/sunray_test/`
  面向框架实现，负责场景编排、配置加载、执行、报告和评分

## 入口

1. 手工入口：`tests/run_test.sh`
2. 场景启动器：`rosrun sunray_test run_scenario.py --scenario ...`
3. 真正执行器：`rosrun sunray_test run_suite.py --platform ... --environment sim|exp --suite ...`
4. 内部直连入口：`rosrun sunray_test internal_run.py`
5. 生效配置查看：`rosrun sunray_test show_config.py --platform ... --environment ... --suite ...`

测试产物默认输出到 `workspace_root/tests/output/<timestamp>/`。  
如果需要覆盖输出目录，只通过 `run_suite.py --output-dir ...` 传参，不再在 suite 配置里定义。

### 入口说明

- `run_scenario.py`
  负责按 `config/scenarios/*.yaml` 拉起仿真/实机场景窗口。
  在真正开终端前会先校验 `platform/environment/suite` 组合是否合法，避免配置错了还先拉一堆窗口。
- `run_suite.py`
  底层执行器，直接调用 `TestRunner` 跑一套测试。
- `internal_run.py`
  适合环境已经准备好、只想反复切换 suite 重跑的场景。
  它会在运行时列出 `config/suites/*.yaml`，只要求选择 suite，不再交互输入 `SN` 和测试人员。
- `show_config.py`
  输出 merge 后并且通过校验的最终配置，适合调参时确认真实生效值。

### 常用命令

```bash
source devel/setup.bash

rosrun sunray_test internal_run.py
rosrun sunray_test run_suite.py --platform sunray150_basic --environment sim --suite basic_acceptance
rosrun sunray_test run_scenario.py --list
rosrun sunray_test run_scenario.py --scenario sunray150_basic_sim
rosrun sunray_test show_config.py --platform sunray150_basic --environment sim --suite flight_regression
rosrun sunray_test show_config.py --platform sunray150_basic --environment sim --suite flight_regression --section topics --format yaml
```

## 安装和编译

从仓库根目录执行：

```bash
bash General_Module/sunray_test/setup_sunray_test.sh
source devel/setup.bash
```

该脚本会检查并安装 `sunray_test` 需要的 Python 库，然后执行：

```bash
catkin_make --source General_Module/sunray_test --build build/sunray_test
```

## 配置模型

当前配置按职责拆成几层：

- `platforms`
  机型默认参数，例如悬停时长、航点阈值、电池阈值、录包 topic 模板
- `environments`
  环境差异，例如 `sim / exp` 的 topic 覆盖和额外录包 topic
- `suites`
  测试项顺序和编排，只描述这次测什么、按什么顺序测
- `scenarios`
  场景启动链，描述要拉起哪些 `roslaunch/tab`，以及默认 runner 参数
- `missions`
  可复用飞行任务，例如航点列表
- `scoring`
  飞行分析评分规则和等级阈值

### 当前约定

- `platforms` 保存机型默认测试参数
- `suites` 只保存测试顺序和编排必需参数，例如 `mission_key`
- 如果某个测试项没有在 `suite.steps[].params` 里显式传参，就自动使用 `platform.defaults`
- `environment` 只覆盖环境差异，不重复写整套默认参数

## 配置校验与生效配置查看

`src/sunray_test/core/suite_loader.py` 现在负责两件事：

- `load_config_triplet(...)`
  负责 merge `platform/environment/suite`
- `validate_config_triplet(...)`
  负责在加载期校验结构、类型、topic 引用、mission 引用、case/phase 是否已注册

这意味着以下问题会在“加载配置时”直接报错，而不是跑到一半才炸：

- YAML 顶层缺 key 或写错 key
- `steps` 结构错误
- `case type / phase` 未注册
- `topic_key` 指向不存在的 topic
- `mission_key` 指向不存在的 mission
- 数值字段、布尔字段类型不对

调参时建议优先用：

```bash
rosrun sunray_test show_config.py --platform sunray150_basic --environment sim --suite basic_acceptance
```

它会输出 merge 后并且通过校验的最终配置，减少在 `platform/environment/suite` 之间来回跳。

## 目录职责

- `config/platforms/`
  机型配置
- `config/environments/`
  环境配置，例如 `sim` / `exp`
- `config/scenarios/`
  场景启动配置，例如拉起哪些 launch/tab/延迟和默认 runner 参数
- `config/missions/`
  复用飞行任务配置，例如航点序列
- `config/cameras/<platform>/`
  实机前视/下视相机驱动配置，例如 `video_device`
- `config/suites/`
  测试项顺序和编排
- `config/scoring/`
  飞行评分权重、门槛和等级阈值
- `src/sunray_test/cases/`
  硬件测试和飞行任务
- `src/sunray_test/phases/`
  起飞、降落等可复用阶段动作
- `src/sunray_test/capabilities/`
  rosbag、事件日志、硬件探测等公共能力
- `src/sunray_test/core/`
  配置加载、上下文、执行主流程、结果模型
- `src/sunray_test/reports/`
  分析、评分和 HTML 报告生成

## 报告模块结构

报告相关代码已经拆成“评分”和“渲染”两层，避免所有逻辑堆在单文件里：

- `reports/flight_metrics.py`
  负责从日志和 rosbag 中提取飞行分析结果
- `reports/scoring.py`
  负责综合评分、分项评分、权重和等级
- `reports/html_renderer.py`
  页面总编排
- `reports/renderers/common.py`
  公共格式化、状态 badge、键值描述渲染
- `reports/renderers/summary.py`
  水印、综合评分、阶段轨迹、基础信息
- `reports/renderers/cases.py`
  用例表格和展开详情
- `reports/renderers/flight.py`
  飞行指标内容、配置快照、产物信息
- `reports/renderers/styles.py`
  报告 CSS

## 参数修改位置

- 改机型默认 topic / 阈值：`config/platforms/*.yaml`
- 改 sim / exp 环境差异：`config/environments/*.yaml`
- 改场景拉起链：`config/scenarios/*.yaml`
- 改航点任务：`config/missions/*.yaml`
- 改测试顺序：`config/suites/*.yaml`
- 改评分规则：`config/scoring/scoring.yaml`
- 改实机相机设备号：`config/cameras/<platform>/*.yml`

相机在线检测默认不只检查“有没有图像消息”，还会检查图像数据是否不是整帧全一致。  
这条规则当前为代码内默认开启，不再放到配置文件中修改。

切换到 `CMD_CONTROL` 后，如果需要等待几秒再解锁，可在 `config/platforms/*.yaml` 中调整。  
当前默认固定等待 3 秒。

## sim / exp 话题对照

### 相机话题

- `platforms`
  - `front_camera` / `down_camera` 默认留空，由环境显式提供
- `sim`
  - `front_camera`: `/uav{uav_id}/monocular_front/image_raw`
  - `down_camera`: `/uav{uav_id}/monocular_down/image_raw`
- `exp`
  - `front_camera`: `/web_cam_front/image_raw`
  - `down_camera`: `/web_cam/image_raw`

### 录包 topic

- 平台公共 topic
  - `/uav{uav_id}/mavros/local_position/pose`
  - `/uav{uav_id}/sunray/uav_state`
  - `/uav{uav_id}/sunray/uav_control_cmd`
  - `/uav{uav_id}/sunray/setup`
  - `/uav{uav_id}/sunray_detect/landmark_detection_ros`
- `sim` 额外追加
  - `/uav{uav_id}/sunray/gazebo_pose`
- `exp` 额外追加
  - `/vrpn_client_node_1/uav1/pose`
  - `/vrpn_client_node_1/uav1/twist`
