#!/bin/bash
# 脚本：实机 NavRL，地图输入来自 MID360 点云，定位来自 Sunray external_fusion 输出

# 请注意,由于gnome-terminal语法的限制,连续子页之间无法使用注释,因此在这里对每个launch起到的作用进行简单地说明

# roscore：提供 ROS master。
# sunray_mavros_exp.launch：启动实机 MAVROS 链路。
# external_fusion.launch：接入实机外部定位并发布 /uav1/sunray/uav_odom。
# sunray_control_node.launch：启动 Sunray 无人机控制节点，订阅 /uav1/sunray/uav_control_cmd。
# NavRL2Sunray.launch：校平 FAST-LIO body 系点云后，再接入 NavRL 地图与 Sunray 控制适配链路。
# NavRLTerminalControl.launch：提供终端菜单，用于起飞、降落、悬停和发布 NavRL 目标点。
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_uav_control sunray_mavros_exp.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control external_fusion.launch external_source:=0 uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 4.0; roslaunch sunray_uav_control sunray_control_node.launch uav_id:=1; exec bash"' \
--tab -e 'bash -c "sleep 8.0; roslaunch sunray_navrl_adapter NavRL2Sunray.launch agent_id:=1 enable_body_point_cloud_leveler:=true point_cloud_topic:=/cloud_registered_body_aligned rviz:=false; exec bash"' \
--tab -e 'bash -c "sleep 10.0; roslaunch sunray_navrl_adapter NavRLTerminalControl.launch agent_id:=1 frame_id:=map; exec bash"'

# msg_MID360.launch：启动 MID360 驱动，发布雷达点云和 IMU 数据。
# mapping_mid360.launch：启动 FAST-LIO 建图定位链路，为 external_fusion 提供实机定位来源。
gnome-terminal --window -e 'bash -c "sleep 4.0; roslaunch sunray_planner_utils msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 6.0; roslaunch sunray_planner_utils mapping_mid360.launch rviz:=false; exec bash"'

# navigation_node.py：进入 NavRL Conda 环境并加载 ROS 工作空间后，启动 NavRL 策略推理节点。
gnome-terminal --window -e 'bash -c "
# 等待前面的 ROS、实机驱动、建图定位和适配节点启动完成。
sleep 12.0

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

# 保持终端窗口不自动关闭，便于查看日志。
exec bash
"'
