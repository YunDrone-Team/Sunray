# sunray_fmt_control

专门用于 FMT 飞控控制话题的 ROS 功能包。

## 主要功能
- fmt_control节点发布/mavros/setpoint_raw/local等控制话题，实现无人机定点、轨迹、航点任务等控制
- 支持多种控制模式：位置控制、速度控制、姿态控制
- 支持多种飞行模式：定点悬停、轨迹飞行等
- fmt_externalFusion外部点位节点发布/sunray/px4_state，该话题包括无人机状态、无人机的实时位置、速度等信息
- 提供多种示例脚本，演示无人机起飞、定点

## 快速使用

1. 编译功能包：
   ```bash
    在Sunray目录下，运行./build.sh，选择编译sunray_fmt_control功能包
   ```
2. 在scripts_exp目录下运行示例脚本：
   ```bash
   #cpp脚本
   ./demo_takeoff_hover_land_fmt.sh # 起飞、悬停、降落
   ./demo_block_pos_fmt.sh # 方形轨迹
   ./demo_circle_fmt.sh # 圆周轨迹
   ./demo_hexayon_fmt.sh # 六边形轨迹
   #python脚本
   ./demo_py_takeoff_hover_land_fmt.sh # 起飞、悬停、降落
   ./demo_py_block_pos_fmt.sh # 方形轨迹
   ./demo_py_circle_fmt.sh # 圆周轨迹
   ./demo_py_hexayon_fmt.sh # 六边形轨迹
   ```
3. 查看控制话题：
   ```bash
   rostopic list
   ```
## 目录结构
- src/  fmt_control控制节点，fmt_externalFusion外部点位节点
- test/  测试脚本
- README.md 说明文档
- package.xml/CMakeLists.txt ROS包配置
- launch/  启动文件

## 注意事项
- 确保无人机已连接到 ROS 系统，且已启动 mavros 节点
- 示例脚本中默认使用无人机 ID 1，若需修改，请在脚本中修改uav_id参数