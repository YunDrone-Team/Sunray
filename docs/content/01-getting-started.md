<!-- title: 入门与仓库地图 -->

<section id="intro">

## 手册定位

Sunray 是一个 ROS/catkin 风格的无人机、无人车仿真与实践平台。它已经把很多复杂内容封装好了：PX4/MAVROS 状态、外部定位融合、控制指令转换、示例任务、规划器桥接、云台和通信等。

对新手来说，最重要的不是马上修改 PX4 或 MAVROS，而是先学会用 Sunray 的上层接口发指令。多数二次开发任务可以只写一个 ROS 节点，订阅状态，发布控制命令。

<div class="note">
<strong>阅读顺序建议</strong>
            先看仓库地图、运行链路和消息接口；然后选 UAV 或 UGV 的任务示例复制修改；最后再按需要进入定位、规划、视觉、编队、通信等扩展模块。
          </div>

<div class="toc-cards">
<div class="toc-card">
<h3>讲得多的部分</h3>
<p>这些是 Sunray 二次开发主接口，学生会经常复制、调用或调参。</p>
<ul>
<li>`General_Module/sunray_common/sunray_msgs`</li>
<li>`General_Module/sunray_uav_control`</li>
<li>`General_Module/sunray_tutorial`</li>
<li>`General_Module/sunray_planner_utils`</li>
<li>`External_Module/ego-planner-swarm` 的 Sunray 接入方式</li>
<li>`scripts_sim`、`scripts_exp`、`scripts_swarm`、`server` 中的快速启动链路</li>
</ul>
</div>
<div class="toc-card">
<h3>讲得少的部分</h3>
<p>这些多为第三方算法、驱动或专用功能，手册主要讲“怎么接”和“什么时候看”。</p>
<ul>
<li>`External_Module/FUEL`</li>
<li>`External_Module/VINS-Fusion`、`External_Module/oradar_lidar`</li>
<li>`External_Module/turn_on_wheeltec_robot`、`vrpn_client_ros`</li>
<li>`sunray_formation/sunray_orca` 的底层 ORCA 实现</li>
<li>通信协议内部编码和地面站私有协议细节</li>
</ul>
</div>
</div>

</section>

<section id="quick-map">

## 仓库地图

当前仓库可以按“核心模块、外部模块、启动脚本、构建系统、编队模块”理解。

| 路径 | 作用 | 新手优先级 |
| --- | --- | --- |
| `General_Module/sunray_common` | 公共头文件、自定义 ROS 消息 `sunray_msgs`。 | 高。先看消息定义。 |
| `General_Module/sunray_uav_control` | 无人机控制核心：MAVROS 接口、外部定位汇总、UAV 控制状态机、航点、遥控器输入。 | 高。理解接口，谨慎修改内部。 |
| `General_Module/sunray_ugv_control` | 无人车控制核心：订阅 UGV 控制指令，输出 `cmd_vel`，带简单 A* 和 RViz 显示。 | 高。UGV 项目必须看。 |
| `General_Module/sunray_tutorial` | 二次开发示例：起飞降落、方形/圆形/六边形轨迹、跟车、识别降落、UGV 轨迹等。 | 最高。新任务从这里复制。 |
| `General_Module/sunray_planner_utils` | 规划器适配工具：点云转换、LaserScan 转点云、EGO/FUEL `PositionCommand` 转 Sunray 控制指令、目标点分发。 | 中高。做规划时看。 |
| `Comunication_Module/sunray_communication_bridge` | 地面站和智能体通信桥，负责状态、控制、航点、RTK 原点、编队命令等转发。 | 中。需要地面站或多机通信时看。 |
| `External_Module` | 第三方/外部功能：EGO、FUEL、VINS-Fusion、雷达驱动、轮趣底盘、VRPN、视觉检测等。 | 按需。优先看 launch 和接入话题。 |
| `sunray_formation` | 多机编队、RViz 编队仿真、ORCA 避障。 | 按需。做集群再深入。 |
| `scripts_sim` / `scripts_exp` | 仿真/实机一键启动组合，展示一个任务需要哪些节点同时运行。 | 高。运行流程从这里学。 |
| `buildscripts` / `build.sh` | 模块化构建系统，支持 TUI/CLI 选择模块组。 | 中。会用即可。 |

<div class="warn">
<strong>当前仓库状态提醒</strong>
            `Simulation` 目录在当前工作区是空目录，并且 `git submodule status` 显示它像是未初始化的子模块。因此文档中涉及仿真世界和 `sunray_simulator` 的内容主要依据现有启动脚本和构建配置说明。实际运行仿真前，请确认仿真子模块/包已经拉取完整并编译。
          </div>

</section>

<section id="learn-path">

## 学习路线

<div class="path-grid">
<div class="path-card">
<span class="tag">阶段 1</span>
<h3>先跑通现成示例</h3>
<p>目标是知道一个任务要启动哪些节点，不急着改代码。</p>
<pre><code class="language-bash">source devel/setup.bash
bash scripts_sim/demo_takeoff_hover_land.sh</code></pre>
<p>如果是实机，先看 `scripts_exp/demo_takeoff_hover_land.sh`，但不要直接上桨测试，先确认定位、遥控器、急停和场地。</p>
</div>
<div class="path-card">
<span class="tag">阶段 2</span>
<h3>读懂任务层示例</h3>
<p>优先读 `sunray_tutorial/uav_basic/block_xyzpos.cpp` 或 `uav_python/takeoff_hover_land.py`。它们展示了二次开发最常见写法：订阅状态、发布控制指令。</p>
</div>
<div class="path-card">
<span class="tag">阶段 3</span>
<h3>写自己的任务节点</h3>
<p>复制一个 tutorial 示例，改目标点、速度、状态机和判断条件。先仿真，再低速实机。</p>
</div>
<div class="path-card">
<span class="tag">阶段 4</span>
<h3>接入自己的算法</h3>
<p>算法节点只需要输出 Sunray 控制话题，或者输出规划器通用消息再用 `sunray_planner_utils` 桥接。</p>
</div>
</div>

</section>

<section id="build-run">

## 编译与启动

仓库根目录有 `build.sh`，它封装了模块化构建。没有参数时进入 TUI，带参数时走 CLI。

```bash
# 查看可用模块
./build.sh --list

# 查看模块组
./build.sh --groups

# 构建常用 UAV 组
./build.sh uav

# 构建 UGV 组
./build.sh ugv

# 重复上次选择
./build.sh -s

# 构建完成后
source devel/setup.bash
```

### 模块组怎么选

| 组名 | 适合场景 | 包含重点 |
| --- | --- | --- |
| `uav` | 普通无人机控制、仿真、规划、云台、通信。 | UAV 控制、tutorial、planner utils、EGO、formation。 |
| `fmt` | FMT 飞控平台。 | FMT 控制、通信、tutorial。 |
| `viobot` | VIOBOT 定位/视觉方案。 | UAV 控制、planner utils、media、detection、EGO。 |
| `ugv` | 无人车控制。 | UGV 控制、轮趣底盘、2D 雷达、tutorial。 |
| `swarm_uav` | 无人机集群。 | UAV 控制、通信、ORCA、编队。 |

### 启动脚本的阅读方法

`scripts_sim` 和 `scripts_exp` 不是普通单节点脚本，而是把一个任务需要的多个 `roslaunch` 放到多个终端标签里。比如 `scripts_sim/demo_takeoff_hover_land.sh` 的链路是：

<div class="flow">
<span>roscore</span>
<span>仿真环境</span>
<span>external_fusion</span>
<span>uav_control_node</span>
<span>tutorial demo</span>
</div>

<div class="tip">
<strong>读脚本的小技巧</strong>
            先找 `roslaunch sunray_uav_control external_fusion.launch` 和 `roslaunch sunray_uav_control sunray_control_node.launch`。前者给控制节点提供状态，后者真正执行 Sunray 控制指令。最后启动的 `sunray_tutorial` 节点通常就是任务逻辑。
          </div>

</section>
