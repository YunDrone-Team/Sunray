#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sunray_msgs.msg import UAVState, UAVControlCMD, UAVSetup 
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import signal 
import sys 
import math 
from external import Obs, Servo
from APF_Planner import APF_Planner

from visualization_msgs.msg import Marker
from nav_msgs.msg import Path

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

        "订阅无人机状态消息，发布无人机控制指令和设置指令"
        self.uav_state_sub = rospy.Subscriber(
            self.uav_name + "/sunray/uav_state", UAVState, self.uav_state_cb)
        self.control_cmd_pub = rospy.Publisher(
            self.uav_name + "/sunray/uav_control_cmd", UAVControlCMD, queue_size=1)
        self.uav_setup_pub = rospy.Publisher(
            self.uav_name + "/sunray/setup", UAVSetup, queue_size=1)
        
        # 发布器
        self.pose_pub = rospy.Publisher('/drone_pose', PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        
        # 存储历史轨迹
        self.path = Path()
        self.path.header.frame_id = "world"  # 与TF坐标系一致

        self.obs = Obs(sim=True)  # 是否为仿真模式
        self.obs.open()  # 打开障碍物订阅
        self.servo = Servo(self.uav_name)

        "初始化APF路径规划器"
        self.start = (-2.0, -2.0)  # 起点坐标
        # self.start = (-1.0, -1.0)  # 起点坐标
        self.goal = (1.27, -0.5)   # 目标
        # self.goal = (1.0, 2.0)   # 目标
        self.obstacles = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]  # 障碍物列表
        # self.obstacles = [(0.0, 0.0)]  # 障碍物列表
        self.safe_zone = (-1.0, 2.5, -2.5, -0.5)  # 安全区域边界 Xmin Xmax Ymin Ymax
        # self.safe_zone = (-3.0, 3.0, -3.0, 3.0)  # 安全区域边界 Xmin Xmax Ymin Ymax
        self.apf_planner = APF_Planner(
            self.start, self.goal, self.obstacles, self.safe_zone)

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
        self.height = 1.0 
        self.flip_duration = 2.0  
        self.flip_angle = 90.0  
        self.max_roll_speed = 2.0  
        self.max_pitch_speed = 2.0  
        self.pose = PoseStamped()
        self.points = [
            # [-0.559, -1.176, self.height],
            # [0.706, -0.761, self.height],
            [1.22, -0.572, self.height],
            [1.2678, 0.1259, self.height],
            [1.46, 1.27, self.height],
            [1.43, 2.0, self.height],
            [0.0, 2.0, self.height],
            [-1.5, 1.5, self.height]
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
        target_x = -2.0
        target_y = -2.0
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

            if count > 40:
                rospy.loginfo(f"{self.node_name}: Arrived at origin (position control).")
                break 

            rate.sleep()


    "降落"
    def land(self):
        rate = rospy.Rate(0.25)
        while not rospy.is_shutdown() and (self.uav_state.control_mode != UAVSetup.LAND_CONTROL or self.uav_state.landed_state != 1):
            self.uav_cmd.cmd = UAVControlCMD.Land
            self.uav_cmd.header.stamp = rospy.Time.now()
            self.control_cmd_pub.publish(self.uav_cmd)
            rospy.loginfo(f"{self.node_name}: Land UAV now.")
            rate.sleep()

        rate = rospy.Rate(1.0)  # 1 Hz
        while not rospy.is_shutdown() and self.uav_state.landed_state != 1:
            rospy.loginfo(f"{self.node_name}: Landing")
            rate.sleep()

        rospy.loginfo(f"{self.node_name}: Land UAV successfully!")
        rospy.loginfo(f"{self.node_name}: Demo finished, quit!")

    "位置控制接口"
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

    "速度控制接口"
    def velocity_ctrl(self, vx, vy, vz):
        self.uav_cmd.header.stamp = rospy.Time.now()
        self.uav_cmd.cmd = UAVControlCMD.XyzVel
        self.uav_cmd.desired_vel = [vx, vy, vz]
        self.control_cmd_pub.publish(self.uav_cmd)

    "rviz可视化速度矢量"
    def show_velocity(self, vx, vy, vz):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.ARROW
        marker.scale.x = 0.05; marker.scale.y = 0.1
        marker.color.r = 1.0; marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.1)
        
        start = Point(*self.uav_state.position)
        end = Point(
            self.uav_state.position[0] + vx,
            self.uav_state.position[1] + vy,
            self.uav_state.position[2] + vz
        )
        marker.points = [start, end]
        self.velocity_pub.publish(marker)

    "测试函数"
    def apf_plan_and_fly(self):
        self.points[-1] = [self.obs.get_delivery()[0], self.obs.get_delivery()[1], self.height]  # 更新投放点位置
        self.obstacles = self.obs.get_obstacles()  # 获取障碍物位置
        self.apf_planner.update_obstacles(self.obstacles)

        target_z = self.height
        rate = rospy.Rate(20.0)  # 20 Hz
        while not rospy.is_shutdown():
            now_pos = self.uav_state.position[:2]
            vel = self.apf_planner.calculate_velocity(now_pos)
            # rospy.loginfo(f"{self.node_name}: Current position: {now_pos}, Calculated velocity: {vel}")
            dz = target_z - self.uav_state.position[2]
            vz = self.z_k_p * dz
            vz = min(max(vz, -self.max_vel), self.max_vel)
            self.velocity_ctrl(vel[0], vel[1], vz)

            dx = self.goal[0] - self.uav_state.position[0]
            dy = self.goal[1] - self.uav_state.position[1]

            self.show_velocity(vel[0], vel[1], vz) # 向rviz发布速度矢量

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                break
            rate.sleep()
        rospy.loginfo("APF路径已完成！")

    "测试APF路径规划器"
    def plan_test(self):
        rospy.init_node("plan_test_demo", anonymous=True)
        rate = rospy.Rate(20.0)  # 20 Hz
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        self.set_control_mode()
        self.arm_uav()
        self.takeoff()
        self.hover()

        self.apf_plan_and_fly()

        self.hover()

        for idx, point in enumerate(self.points):
            rospy.loginfo(f"{self.node_name}: Flying to point {idx+1}: {point}")
            target_x, target_y, target_z = point

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
                    if idx == 0:
                        # 设置偏航到 90
                        rospy.sleep(1.0)
                        desired_yaw_deg = 90.0
                        self.uav_cmd.desired_yaw = math.radians(desired_yaw_deg)
                        self.uav_cmd.cmd = UAVControlCMD.XyzPosYaw
                        rospy.loginfo(f"{self.node_name}: Rotating to yaw {desired_yaw_deg} deg")
                        self.uav_cmd.header.stamp = rospy.Time.now()
                        self.control_cmd_pub.publish(self.uav_cmd)
                        rospy.sleep(2.0)  # 等旋转
                        break
                    
                    if idx == 4:
                        self.hover()
                        rospy.sleep(5.0)
                        break

                    break

                rate.sleep()

        self.return_to_origin_pose()

        while not rospy.is_shutdown():
            if (abs(self.uav_state.position[0] - (-2.0)) < 0.1 and
                abs(self.uav_state.position[1] - (-2.0)) < 0.1):
                self.land()
            else:
                self.uav_cmd.desired_pos = [-2.0, -2.0, 0.2]
                self.uav_cmd.cmd = UAVControlCMD.XyzPos
                self.uav_cmd.header.stamp = rospy.Time.now()
                self.control_cmd_pub.publish(self.uav_cmd)



    "主程序"
    def run(self):
        rospy.init_node("competiton_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()

        self.set_control_mode()
        self.arm_uav()
        self.takeoff()
        self.hover()

        # self.fly_cirle()
        # self.custom_fly_vel()
        # self.return_to_origin_vel()
        # self.apf_plan_and_fly()
        self.return_to_origin_pose()
        self.land()

if __name__ == "__main__":
    try:
        controller = CircleVelController()
        # controller.run()
        controller.plan_test()
    except rospy.ROSInterruptException:
        pass
