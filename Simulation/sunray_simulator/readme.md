# sunray_uav_control

#### 介绍
思锐无人机仿真与实践开发平台----仿真模块
#### 运行环境
- ROS Noetic
- gazebo
- mavros
- PX4

#### launch文件说明
```shell
sunray_sim_1uav.launch          --> 启动单个无人机的仿真环境
sunray_sim_3uav.launch          --> 启动3个无人机的仿真环境
sunray_sim_cam.launch           --> 带有相机的无人机
sunray_sim_indoor_1.launch      --> 室内环境
sunray_sim_planner.launch       --> 用于仿真规划器的环境
sunray_sim_with_2dlidar.launch  --> 带有2d激光雷达的仿真环境
sunray_sim_with_3dlidar.launch  --> 带有3d激光雷达的仿真环境

transform_pointcloud.launch     --> 点云坐标转换
scan2Point.launch               --> 2d激光雷达转点云
ego_sunray.launch               --> ego规划结果给sunray控制
```



#### 重要参数
```shell
vehicle: 机型
world: 仿真环境
uav_id： 无人机编号

```