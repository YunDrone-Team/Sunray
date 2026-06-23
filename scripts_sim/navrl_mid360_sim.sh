#!/bin/bash
# 脚本：Gazebo 仿真 NavRL，地图输入来自 Sunray MID360 仿真点云

# 请注意,由于gnome-terminal语法的限制,连续子页之间无法使用注释,因此在这里对每个launch起到的作用进行简单地说明

# roscore：提供 ROS master。
# sunray_sim_uav_planning.launch：启动 Gazebo、PX4 SITL、MAVROS 和带 MID360 的单无人机仿真。
# external_fusion.launch：使用 Gazebo 真值里程计生成 Sunray 控制框架需要的无人机状态。
# sunray_control_node.launch：启动 Sunray 无人机控制节点，订阅 /uav1/sunray/uav_control_cmd。
# NavRL2Sunray.launch：启动 NavRL 地图管理与 Sunray 控制指令适配节点，并打开 RViz。
# NavRLTerminalControl.launch：提供终端菜单，用于起飞、降落、悬停和发布 NavRL 目标点。
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_simulator sunray_sim_uav_planning.launch vehicle:=sunray150_with_mid360 world:=$(rospack find sunray_simulator)/worlds/planning_test.world gui:=true; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=2 position_topic:=/uav1/sunray/gazebo_pose; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_navrl_adapter NavRL2Sunray.launch point_cloud_topic:=/uav1/livox/lidar rviz:=true; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_navrl_adapter NavRLTerminalControl.launch agent_id:=1 frame_id:=map; exec bash"'

# navigation_node.py：进入 NavRL Conda 环境并加载 ROS 工作空间后，启动 NavRL 策略推理节点。
gnome-terminal --window -e 'bash -c "
# 等待前面的 ROS、Gazebo 和适配节点启动完成。
sleep 10.0

# 加载 Conda 命令支持。
source /home/taolin/miniconda3/etc/profile.d/conda.sh

# 进入 NavRL 推理节点依赖的 Conda 环境。
conda activate NavRL

# 加载 ROS Noetic 环境。
source /opt/ros/noetic/setup.bash

# 加载 Sunray 工作空间，使 navigation_runner 包可被 rosrun 找到。
source /home/taolin/Documents/YunDrone/Sunray/devel/setup.bash

# 当前 NavRL 环境的 PyTorch 是 CPU 版本，显式覆盖默认 CUDA 配置后启动策略推理节点。
rosrun navigation_runner navigation_node.py device=cpu sim.device=cpu sim.use_gpu=false hydra.run.dir=/home/taolin/Documents/YunDrone/Sunray/navrl_logs/\${now:%Y-%m-%d}/\${now:%H-%M-%S}

# 如果是GPU带有CUDA加速,可以使用下面的命令
# rosrun navigation_runner navigation_node.py hydra.run.dir=/home/taolin/Documents/YunDrone/Sunray/navrl_logs/\${now:%Y-%m-%d}/\${now:%H-%M-%S}

# 保持终端窗口不自动关闭，便于查看日志。
exec bash
"'
