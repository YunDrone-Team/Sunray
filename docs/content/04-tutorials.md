<!-- title: 重点包深入：sunray_tutorial -->

<section id="deep-tutorial">

## 深入：sunray_tutorial

`General_Module/sunray_tutorial` 是最适合学生二次开发的包。它不是“演示用完就丢”的目录，而是 Sunray 给新任务提供的参考模板库。

### 目录分层

| 目录 | 内容 | 适合人群 |
| --- | --- | --- |
| `uav_basic` | C++ UAV 基础动作：起飞降落、方形、圆、三角形、六边形、跟车/跟标签。 | 做 UAV 任务的主入口。 |
| `uav_python` | Python 版 UAV 基础示例。 | ROS Python 用户。 |
| `ugv_basic` | C++ UGV 位置、速度、车体系速度示例。 | 无人车任务。 |
| `ugv_python` | Python 版 UGV 示例。 | 快速验证算法。 |
| `advanced` | 自动降落、基于 pose 降落、移动小车降落、搜索跟随降落、目标搜索发送给 UGV、简易避障。 | 复合任务。 |
| `launch` | 把各示例包装成可启动节点。 | 脚本和地面站调用。 |
| `config` | 相机参数、二维码/ArUco/椭圆检测参数、日志配置。 | 视觉任务调参。 |
| `scripts` | 键盘控制小车、随机目标、扫描等小工具。 | 辅助测试。 |

### `run_demo.launch` 的 demo_id

`run_demo.launch` 是 UAV 示例的统一入口，快速启动脚本和地面站可以通过 `demo_id` 选择任务。

| `demo_id` | 节点 | 含义 |
| --- | --- | --- |
| `1` | `takeoff_hover_land` | C++ 起飞、悬停、降落。 |
| `2` | `block_xyzpos` | C++ 方形位置轨迹。 |
| `3` | `circle_xyzvel` | C++ XYZ 速度圆轨迹。 |
| `4` | `circle_xyvelzpos` | C++ XY 速度、Z 定高圆轨迹。 |
| `5` | `hexagon_xyzposyawbody` | C++ 机体系六边形。 |
| `11` | `takeoff_hover_land.py` | Python 起飞、悬停、降落。 |
| `12` | `block_pos.py` | Python 方形位置轨迹。 |
| `13` | `circle_vel.py` | Python 圆形速度轨迹。 |
| `14` | `circle_z_pos.py` | Python 圆形 + 高度位置。 |
| `15` | `pos_body_hexagon.py` | Python 机体系六边形。 |
| `16` | `triangle_xyzvel` | C++ 三角形速度轨迹。 |
| `17` | `triangle_xyzpos` | C++ 三角形位置轨迹。 |

### `ugv_test_demo.launch` 的 demo_id

| `demo_id` | 节点 | 含义 |
| --- | --- | --- |
| `1` | `ugv_block_pos` | C++ UGV 方形位置轨迹。 |
| `2` | `ugv_circle_vel` | C++ 惯性系速度圆。 |
| `3` | `ugv_circle_vel_body` | C++ 车体系速度圆。 |
| `11` | `ugv_block_pos.py` | Python UGV 方形位置轨迹。 |
| `12` | `ugv_circle_vel.py` | Python 惯性系速度圆。 |
| `13` | `ugv_circle_vel_body.py` | Python 车体系速度圆。 |

### 改任务时重点改哪里

基础示例里通常都有“轨迹控制关键代码段”。新手应优先改这些内容：

- 目标点数组，例如方形顶点、搜索中心、搜索半径。
- 控制模式，例如 `XyzPos`、`XyzVel`、`XyVelZPosYawBody`。
- 到达判断阈值，例如 `fabs(position-target) < 0.15`。
- 速度、最大速度、P 控制增益等 launch 参数。
- 目标丢失、超时、任务结束后的 Hover/Land/HOLD 行为。

### 复合任务怎么读

| 任务 | 推荐读法 |
| --- | --- |
| `auto_land_by_pose.cpp` | 看目标 pose 如何转换成降落速度/高度变化。 |
| `search_follow_and_land.cpp` | 看搜索、跟随、降落三个状态如何切换。 |
| `search_target_and_send_to_ugv.cpp` | 看 UAV 检测目标后如何把目标发给 UGV 或规划模块。 |
| `simple_obstacle_avoidance*.cpp` | 看感知阈值如何影响速度指令。 |
| `follow_car_xyvelzposyawbody.cpp` | 看机体系速度如何适合相对跟随。 |

</section>

<section id="new-task">

## 写一个新任务

推荐流程：复制 `sunray_tutorial` 中最接近的示例，新建一个节点，然后只改任务逻辑。

### 任务节点基本结构

1. 读取参数：`uav_id` 或 `ugv_id`。
2. 拼接话题前缀：`/uav1` 或 `/ugv1`。
3. 订阅状态：`UAVState` 或 `UGVState`。
4. 发布控制：`UAVControlCMD` 或 `UGVControlCMD`。
5. 等待连接和定位有效。
6. 进入控制模式，执行任务状态机。
7. 结束时悬停、降落或 HOLD。

### 新任务应该放哪里

| 情况 | 建议位置 |
| --- | --- |
| 课程实验、简单示例 | `General_Module/sunray_tutorial/uav_basic`、`uav_python`、`ugv_basic`、`ugv_python`。 |
| 视觉/降落/搜索等复合任务 | `General_Module/sunray_tutorial/advanced`。 |
| 独立研究算法 | 新建自己的 catkin package，依赖 `sunray_msgs`，不要把实验逻辑塞进控制节点。 |
| 底层控制器实验 | 单独分支或新节点，谨慎修改 `sunray_uav_control/uav_control`。 |

### 何时需要改 CMakeLists

如果你新增 C++ 节点，需要在对应包的 `CMakeLists.txt` 添加 `add_executable` 和 `target_link_libraries`。如果只是改 Python 脚本，通常需要确保脚本可执行，并加入 `catkin_install_python` 才便于安装运行。

<div class="note">
<strong>本手册没有修改源码</strong>
            由于本次要求只能在 `docs/` 中操作，这里只说明做法，不直接改 `sunray_tutorial/CMakeLists.txt`。
          </div>

</section>
