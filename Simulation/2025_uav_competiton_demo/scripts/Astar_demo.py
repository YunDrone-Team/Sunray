#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sunray_msgs.msg import UAVState, UAVControlCMD, UAVSetup 
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped as PathPose
from astar import AStar
import signal 
import sys 
import math
import numpy as np
from external import Obs, Servo

class CircleVelController:
    def __init__(self):
        self.node_name = "competiton_demo" 
        self.uav_state = UAVState() 
        self.uav_cmd = UAVControlCMD() 
        self.uav_setup = UAVSetup() 
        
        "获取参数，构建无人机名称"
        self.uav_id = rospy.get_param("~uav_id", 1) #
        self.uav_name = rospy.get_param("~uav_name", "uav")
        self.uav_name = f"/{self.uav_name}{self.uav_id}"

        "订阅无人机状态消息和障碍物位置消息，发布无人机控制指令和设置指令"
        self.uav_state_sub = rospy.Subscriber(
            self.uav_name + "/sunray/uav_state", UAVState, self.uav_state_cb)
        self.control_cmd_pub = rospy.Publisher(
            self.uav_name + "/sunray/uav_control_cmd", UAVControlCMD, queue_size=1)
        self.uav_setup_pub = rospy.Publisher(
            self.uav_name + "/sunray/setup", UAVSetup, queue_size=1)
        
        # rviz可视化发布者
        self.grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.sim = rospy.get_param("/sim", False)
        self.obs = Obs(sim=self.sim)  # 是否为仿真模式
        self.obs.open()  # 打开障碍物订阅
        self.servo = Servo(self.uav_name)

        self.start_x, self.start_y = -2.5, -2.5
        self.goal_x, self.goal_y = 2.02, -0.56
        # self.goal_x, self.goal_y = 0.88, -0.42
        self.obstacle_coords = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]
        self.astar = AStar()
    
        "初始化控制指令和圆形轨迹参数"
        '''
        # 控制命令期望值
        float32[3] desired_pos          ## 期望位置，单位：[m]
        float32[3] desired_vel          ## 期望速度，单位：[m/s]
        float32[3] desired_acc          ## 期望加速度，单位：[m/s^2]
        float32[3] desired_att          ## 期望姿态，单位：[rad]
        float32 desired_yaw             ## 期望偏航角，单位：[rad]
        float32 desired_yaw_rate        ## 期望偏航角角速率，单位：[rad/s]
        '''
        self.uav_cmd.cmd = 0
        self.uav_cmd.desired_pos = [0.0, 0.0, 0.0]
        self.uav_cmd.desired_vel = [0.0, 0.0, 0.0]
        self.uav_cmd.desired_acc = [0.0, 0.0, 0.0]
        self.uav_cmd.desired_att = [0.0, 0.0, 0.0]
        self.uav_cmd.desired_yaw = 0.0
        self.uav_cmd.desired_yaw_rate = 0.0

        "设置参数"
        self.center_x, self.center_y = 0.0, 0.0
        self.radius = 1.0 
        self.num_points = 50 
        self.k_p, self.z_k_p = 1.5, 0.5 
        self.max_vel = 1.0 
        self.height = 1.1 
        self.flip_duration = 2.0  
        self.flip_angle = 90.0  
        self.max_roll_speed = 2.0  
        self.max_pitch_speed = 2.0  
        self.pose = PoseStamped()
        self.points = [
            [2.02, -0.56, self.height], # 随机避障区终点
            [2.02, 0.0, self.height],   # 小框中心位置
            [2.13, 1.14, self.height],  # 大框中心位置
            [2.17, 1.36, self.height],  # 穿框后终点
            [0.11, 1.67, self.height],  # 风扰区
            [0.0, 0.0, self.height]     # 投放区
        ] 

        "退出信号处理"
        signal.signal(signal.SIGINT, self.my_sigint_handler)
        
    "处理CTRL+C信号"
    def my_sigint_handler(self, sig, frame):
        rospy.loginfo(f"{self.node_name}: exit...")
        rospy.signal_shutdown("User interrupted")
        sys.exit(0)

    "无人机状态回调函数"
    def uav_state_cb(self, msg):
        self.uav_state = msg
    
    def publish_occupancy_grid(self, grid, origin, resolution):
        """发布栅格地图到rviz"""
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        
        msg.info.resolution = resolution
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # 转换格式：0=free(白色), 100=occupied(黑色), -1=unknown(灰色)
        flat_grid = grid.flatten()
        occupancy_data = []
        for cell in flat_grid:
            if cell == 0:
                occupancy_data.append(0)    # free
            else:
                occupancy_data.append(100) # occupied
        
        msg.data = occupancy_data
        self.grid_pub.publish(msg)
        rospy.loginfo("栅格地图已发布到rviz")
    
    def publish_path(self, waypoints):
        """发布路径到rviz"""
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        
        for wx, wy in waypoints:
            pose = PathPose()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
        rospy.loginfo(f"路径已发布到rviz，包含{len(waypoints)}个点")
    
    def find_nearest_free_cell(self, grid, center, max_radius=20):
        """在给定中心点周围寻找最近的可通行栅格"""
        center_row, center_col = center
        rows, cols = grid.shape
        
        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) == radius or abs(dc) == radius:  # 只检查边界点
                        new_row = center_row + dr
                        new_col = center_col + dc
                        
                        if (0 <= new_row < rows and 0 <= new_col < cols and 
                            grid[new_row, new_col] == 0):
                            rospy.loginfo(f"找到可通行点: ({new_row}, {new_col}), 距离: {radius}")
                            return (new_row, new_col)
        return None

    "等待无人机连接"
    def wait_for_connection(self):
        rate = rospy.Rate(1.0) #1 Hz
        times = 0
        while not rospy.is_shutdown() and not self.uav_state.connected:
            times += 1
            if times > 5:
                rospy.loginfo(f"{self.node_name}:Wait for UAV connect...")
            rate.sleep()
        rospy.loginfo(f"{self.node_name}:UAV connected!")

    "切换到cmd控制模式"
    def set_control_mode(self):
        rate = rospy.Rate(1.0) #1 Hz
        while not rospy.is_shutdown() and self.uav_state.control_mode != UAVSetup.CMD_CONTROL:
            self.uav_setup.cmd = UAVSetup.SET_CONTROL_MODE
            self.uav_setup.control_mode = "CMD_CONTROL"
            self.uav_setup_pub.publish(self.uav_setup)
            rospy.loginfo(f"{self.node_name}:SET_CONTROL_MODE -> [{self.uav_setup.control_mode}]")
            rate.sleep()
        rospy.loginfo(f"{self.node_name}:UAV control_mode set to [{self.uav_setup.control_mode}] successfully!")

    "解锁无人机"
    def arm_uav(self):
        for i in range(2, 0, -1): 
            rospy.loginfo(f"{self.node_name}: Arm UAV in {i} sec...")
            rospy.sleep(1.0)
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown() and not self.uav_state.armed:
            self.uav_setup.cmd = UAVSetup.ARM 
            self.uav_setup_pub.publish(self.uav_setup)
            rospy.loginfo(f"{self.node_name}: ARM UAV now.")
            rate.sleep()
        rospy.loginfo(f"{self.node_name}: ARM UAV successfully!")

    "起飞无人机"
    def takeoff(self):
        rate = rospy.Rate(0.25)
        while not rospy.is_shutdown() and abs(self.uav_state.position[2] - self.uav_state.home_pos[2] - self.uav_state.takeoff_height) > 0.2: 
            self.uav_cmd.cmd = UAVControlCMD.Takeoff
            self.uav_cmd.header.stamp = rospy.Time.now()
            self.control_cmd_pub.publish(self.uav_cmd)
            rospy.loginfo(f"{self.node_name}: Takeoff UAV now.")
            rate.sleep()
        rospy.loginfo(f"{self.node_name}: Takeoff UAV successfully!")

    "悬停无人机"
    def hover(self, time=1):
        rospy.sleep(1.0)
        rospy.loginfo(f"{self.node_name}: Send UAV Hover cmd.")
        self.uav_cmd.cmd = UAVControlCMD.Hover
        self.uav_cmd.header.stamp = rospy.Time.now()
        self.control_cmd_pub.publish(self.uav_cmd)
        rospy.sleep(time)

    "单点位置控制接口"
    def pose_ctrl(self, x, y, z):
        rate = rospy.Rate(20.0)  # 20 Hz
        count = 0
        while not rospy.is_shutdown():
            self.uav_cmd.header.stamp = rospy.Time.now()
            self.uav_cmd.cmd = UAVControlCMD.XyzPos  # 位置模式
            self.uav_cmd.desired_pos = [x, y, z]
            self.control_cmd_pub.publish(self.uav_cmd)
            dx = x - self.uav_state.position[0]
            dy = y - self.uav_state.position[1]
            dz = z - self.uav_state.position[2]
            if abs(dx) < 0.1 and abs(dy) < 0.1 and abs(dz) < 0.2:
                count += 1
            else:
                count = 0

            if count > 40:
                rospy.loginfo(f"{self.node_name}: Reached target position ({x}, {y}, {z})")
                break
            rate.sleep()

    "单点速度控制接口"
    def vel_ctrl(self, vx, vy, vz):
        self.uav_cmd.header.stamp = rospy.Time.now()
        self.uav_cmd.cmd = UAVControlCMD.XyzVel
        self.uav_cmd.desired_vel = [vx, vy, vz]
        self.control_cmd_pub.publish(self.uav_cmd)

    "自定义飞行,使用速度控制接口"
    def custom_fly_vel(self):
        rate = rospy.Rate(20.0)  # 20 Hz

        for idx, point in enumerate(self.points):
            rospy.loginfo(f"{self.node_name}: Flying to point {idx+1}: {point}")
            target_x, target_y, target_z = point

            while not rospy.is_shutdown():
                dx = target_x - self.uav_state.position[0]
                dy = target_y - self.uav_state.position[1]
                dz = target_z - self.uav_state.position[2]

                vx = self.k_p * dx
                vy = self.k_p * dy
                vz = self.z_k_p * dz

                vx = min(max(vx, -self.max_vel), self.max_vel)
                vy = min(max(vy, -self.max_vel), self.max_vel)
                vz = min(max(vz, -self.max_vel), self.max_vel)

                self.uav_cmd.header.stamp = rospy.Time.now()
                self.uav_cmd.cmd = UAVControlCMD.XyzVel
                self.uav_cmd.desired_vel = [vx, vy, vz]
                self.control_cmd_pub.publish(self.uav_cmd)

                if abs(dx) < 0.15 and abs(dy) < 0.15 and abs(dz) < 0.2:
                    rospy.loginfo(f"{self.node_name}: Reached point {idx+1}")
                    if idx == 6:
                        self.hover()
                    break

                rate.sleep()

    "自定义飞行：使用位置控制接口"
    def custom_fly_pose(self):
        rate = rospy.Rate(20.0)  # 20 Hz

        for idx, point in enumerate(self.points):
            rospy.loginfo(f"{self.node_name}: Flying to point {idx+1}: {point}")
            target_x, target_y, target_z = point

            drop_count = 0  # 投放计数
            while not rospy.is_shutdown():
                self.uav_cmd.header.stamp = rospy.Time.now()
                self.uav_cmd.cmd = UAVControlCMD.XyzPos  # 位置模式
                self.uav_cmd.desired_pos = [target_x, target_y, target_z]
                self.control_cmd_pub.publish(self.uav_cmd)

                dx = target_x - self.uav_state.position[0]
                dy = target_y - self.uav_state.position[1]
                dz = target_z - self.uav_state.position[2]

                if abs(dx) < 0.15 and abs(dy) < 0.15 and abs(dz) < 0.2:
                    rospy.loginfo(f"{self.node_name}: Reached point {idx+1}")
                    
                    if idx == 4:
                        self.hover(2)
                        break

                    if idx == 5:
                        self.hover()
                        while True:
                            self.uav_cmd.header.stamp = rospy.Time.now()
                            self.uav_cmd.cmd = UAVControlCMD.XyzPos  # 位置模式
                            self.uav_cmd.desired_pos = [target_x, target_y, target_z]
                            self.control_cmd_pub.publish(self.uav_cmd)

                            dx = target_x - self.uav_state.position[0]
                            dy = target_y - self.uav_state.position[1]
                            # dz = target_z - 0.07 - self.uav_state.position[2]
                            dz = target_z - self.uav_state.position[2]

                            if abs(dx) < 0.1 and abs(dy) < 0.1 and abs(dz) < 0.1:
                                drop_count += 1
                            else:
                                drop_count = 0
                            
                            if drop_count > 20:
                                self.servo.servo_drop()  # 投放
                                self.hover()
                                break

                            rate.sleep()

                    break

                rate.sleep()

    "返回原点(速度控制)"
    def return_to_origin_vel(self):
        rate = rospy.Rate(20.0)
        count = 0
        self.pose.pose.position.x = -2.0
        self.pose.pose.position.y = -2.0
        self.pose.pose.position.z = self.height

        while not rospy.is_shutdown():
            dx = self.pose.pose.position.x - self.uav_state.position[0]
            dy = self.pose.pose.position.y - self.uav_state.position[1]
            dz = self.pose.pose.position.z - self.uav_state.position[2]

            vx = self.k_p * dx
            vy = self.k_p * dy
            vz = self.z_k_p * dz

            vx = min(max(vx, -self.max_vel), self.max_vel)
            vy = min(max(vy, -self.max_vel), self.max_vel)
            vz = min(max(vz, -self.max_vel), self.max_vel)

            self.uav_cmd.header.stamp = rospy.Time.now()
            self.uav_cmd.cmd = UAVControlCMD.XyzVel
            self.uav_cmd.desired_vel = [vx, vy, vz]
            self.control_cmd_pub.publish(self.uav_cmd)

            if abs(self.uav_state.position[0] - self.pose.pose.position.x) < 0.1 and \
               abs(self.uav_state.position[1] - self.pose.pose.position.y) < 0.1:
                count += 1
            else:
                count = 0

            if count > 40:
                rospy.loginfo(f"{self.node_name}: Arrived at origin (velocity control).")
                break

            rate.sleep()

    "返回原点（位置控制）"
    def return_to_origin_pose(self):
        rate = rospy.Rate(20.0)
        count = 0
        target_x = self.uav_state.home_pos[0]
        target_y = self.uav_state.home_pos[1]
        print(self.uav_state.home_pos)
        target_z = self.height

        while not rospy.is_shutdown():
            self.uav_cmd.header.stamp = rospy.Time.now()
            self.uav_cmd.cmd = UAVControlCMD.XyzPos   
            self.uav_cmd.desired_pos = [target_x, target_y, target_z]
            self.control_cmd_pub.publish(self.uav_cmd)

            dx = target_x - self.uav_state.position[0]
            dy = target_y - self.uav_state.position[1]

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                count += 1
            else:
                count = 0

            if count > 20:
                rospy.loginfo(f"{self.node_name}: Arrived at origin (position control).")
                break 

            rate.sleep()

    "降落"
    def land(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and (self.uav_state.control_mode != UAVSetup.LAND_CONTROL or self.uav_state.landed_state != 1):
            self.uav_cmd.header.stamp = rospy.Time.now()
            self.uav_cmd.cmd = UAVControlCMD.Land
            self.control_cmd_pub.publish(self.uav_cmd)
            rospy.loginfo(f"{self.node_name}: Land UAV now.")
            rate.sleep()

        rate = rospy.Rate(2.0)  # 1 Hz
        while not rospy.is_shutdown() and self.uav_state.landed_state != 1:
            rospy.loginfo(f"{self.node_name}: Landing")
            rate.sleep()

        rospy.loginfo(f"{self.node_name}: Land UAV successfully!")
        rospy.loginfo(f"{self.node_name}: Demo finished, quit!")

    "A*规划"
    def astar_plan_and_fly(self):
        # A*参数
        origin = (self.start_x, self.start_y)
        resolution = 0.05 # 分辨率
        rows, cols = 120, 120 # 栅格地图大小
        inflation_radius = 0.54 # 膨胀半径
        self.points[-1] = [self.obs.get_delivery()[0], self.obs.get_delivery()[1], self.height]  # 更新投放点位置
        self.obstacle_coords = self.obs.get_obstacles()
        # print(self.obstacle_coords)
        
        # 生成grid（带膨胀）
        grid = self.astar.create_grid_from_obstacles(
            self.obstacle_coords, rows, cols, resolution, origin, inflation_radius)
        
        # 发布栅格地图到rviz
        self.publish_occupancy_grid(grid, origin, resolution)
        
        # 获取起点和终点的栅格坐标
        start_x, start_y = self.uav_state.position[0], self.uav_state.position[1]
        start_grid = self.astar.world_to_grid(start_x, start_y, origin, resolution)
        goal_grid = self.astar.world_to_grid(self.goal_x, self.goal_y, origin, resolution)
        
        # 调试信息：打印坐标转换结果
        rospy.loginfo(f"起点世界坐标: ({start_x:.2f}, {start_y:.2f})")
        rospy.loginfo(f"终点世界坐标: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        rospy.loginfo(f"起点栅格坐标: {start_grid}")
        rospy.loginfo(f"终点栅格坐标: {goal_grid}")
        
        # 检查起点和终点是否在地图范围内
        if not (0 <= start_grid[0] < rows and 0 <= start_grid[1] < cols):
            rospy.logerr(f"起点超出地图范围: {start_grid}, 地图大小: ({rows}, {cols})")
            self.land()
            return
        if not (0 <= goal_grid[0] < rows and 0 <= goal_grid[1] < cols):
            rospy.logerr(f"终点超出地图范围: {goal_grid}, 地图大小: ({rows}, {cols})")
            self.land()
            return
            
        # 检查起点和终点是否在障碍物内
        if grid[start_grid[0], start_grid[1]] == 1:
            rospy.logwarn(f"起点位于障碍物内: {start_grid}, 尝试寻找附近可通行点")
            start_grid = self.find_nearest_free_cell(grid, start_grid)
            if start_grid is None:
                rospy.logerr("无法找到可通行的起点")
                self.land()
                return
                
        if grid[goal_grid[0], goal_grid[1]] == 1:
            rospy.logwarn(f"终点位于障碍物内: {goal_grid}, 尝试寻找附近可通行点")
            goal_grid = self.find_nearest_free_cell(grid, goal_grid)
            if goal_grid is None:
                rospy.logerr("无法找到可通行的终点")
                self.land()
                return
        
        # 路径规划
        path = self.astar.search(start_grid, goal_grid, grid)
        if not path or len(path) < 2:
            rospy.logwarn("A*未找到有效路径，任务中止！")
            rospy.loginfo(f"栅格地图障碍物数量: {np.sum(grid == 1)}")
            rospy.loginfo(f"栅格地图自由空间数量: {np.sum(grid == 0)}")
            self.land()
            return
        # 路径点还原为世界坐标
        waypoints = [self.astar.grid_to_world(gy, gx, origin, resolution) for gy, gx in path]
        
        # 路径平滑化处理 - 减少路径点数量以提高速度
        rospy.loginfo(f"原始路径点数: {len(waypoints)}")
        waypoints = self.astar.smooth_path(waypoints, smoothing_factor=0.3, iterations=3)
        # 减少插值点数量，或跳过插值直接使用平滑路径
        # waypoints = self.astar.interpolate_path(waypoints, num_points=2)  # 注释掉减少路径点
        if not self.astar.is_path_safe(waypoints, grid, origin, resolution):
            rospy.logwarn("平滑路径不安全，回退到原始A*路径")
            waypoints = [self.astar.grid_to_world(gy, gx, origin, resolution) for gy, gx in path]
        
        # 进一步减少路径点 - 每隔几个点取一个
        if len(waypoints) > 10:
            step = max(1, len(waypoints) // 8)  # 最多保留8个关键点
            key_waypoints = waypoints[::step]
            if waypoints[-1] not in key_waypoints:
                key_waypoints.append(waypoints[-1])  # 确保包含终点
            waypoints = key_waypoints
        
        rospy.loginfo(f"优化后路径点数: {len(waypoints)}")
        
        # 发布路径到rviz
        self.publish_path(waypoints)
        
        # 控制无人机依次飞向waypoints（位置控制）
        rate = rospy.Rate(30.0)
        for idx, (wx, wy) in enumerate(waypoints):
            rospy.loginfo(f"A*航点 {idx+1}/{len(waypoints)}: ({wx:.2f}, {wy:.2f}, {self.height:.2f})")
            while not rospy.is_shutdown():
                self.uav_cmd.header.stamp = rospy.Time.now()
                self.uav_cmd.cmd = UAVControlCMD.XyzPos
                self.uav_cmd.desired_pos = [wx, wy, self.height]
                self.control_cmd_pub.publish(self.uav_cmd)
                dx = wx - self.uav_state.position[0]
                dy = wy - self.uav_state.position[1]
                dz = self.height - self.uav_state.position[2]
                if abs(dx) < 0.15 and abs(dy) < 0.15 and abs(dz) < 0.2:
                    break
                rate.sleep()
        rospy.loginfo("A*路径已完成!")


    "主程序"
    def run(self):
        rospy.init_node("Astar_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        self.set_control_mode()
        self.arm_uav()
        self.takeoff()
        self.hover()
        self.astar_plan_and_fly()
        self.custom_fly_pose()
        self.return_to_origin_pose()
        self.land()
        self.servo.servo_init()

    "舵机开闭测试"
    def drop_test(self, state='close'):
        rospy.init_node("Astar_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        self.set_control_mode()

        if state=='close':
            self.servo.servo_init()     # 闭合
        else:
            self.servo.servo_drop()   # 开启

    "打印运行点位用于测试是否订阅成功"
    def points_print(self):
        rospy.init_node("Astar_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        rospy.sleep(3) # 等待数据更新
        self.points[-1] = [self.obs.get_delivery()[0], self.obs.get_delivery()[1], self.height]  # 更新投放点位置
        self.obstacle_coords = self.obs.get_obstacles()
        # 打印当前无人机位置
        print("当前无人机位置:")
        print(self.uav_state.position) 
        # 打印所有运行点位
        print("所有运行点位:")
        for point in self.points:
            print(point)
        # 打印所有障碍物点位
        print("所有障碍物点位:")
        for obs in self.obstacle_coords:
            print(obs)

if __name__ == "__main__":
    try:
        controller = CircleVelController()        
        # controller.run()                # 完整流程
        controller.points_print()     # 打印运行点位
        # controller.drop_test('close')   # 闭合抛投器
        # controller.drop_test('open')    # 开启抛投器
    except rospy.ROSInterruptException:
        pass
