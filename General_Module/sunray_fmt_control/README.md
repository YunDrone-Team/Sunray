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
    在sunray目录下，运行./build_fmt_control.sh
   ```
2. 在scripts_fmt_exp运行示例脚本：
   ```bash
   ./demo_takeoff_hover
   ./demo_block_pos.sh
   ./demo_circle.sh
   ```
3. 查看控制话题：
   ```bash
   rostopic list

## 目录结构
- src/  cpp脚本
- README.md 说明文档
- package.xml/CMakeLists.txt ROS包配置
- launch/  启动文件