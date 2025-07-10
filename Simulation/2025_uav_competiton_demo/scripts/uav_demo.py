#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sunray_msgs.msg import UAVState, UAVControlCMD, UAVSetup 
from geometry_msgs.msg import PoseStamped 
import signal 
import sys 
import math 

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
            [-0.559, -1.176, self.height],
            [0.706, -0.761, self.height],
            [1.22, -0.572, self.height],
            [1.2678, 0.1259, self.height],
            [1.46, 1.27, self.height],
            [1.43, 2.0, self.height],
            [0.0, 1.5, self.height],
            [-1.5, 1.5, self.height],
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
    def hover(self):
        rospy.sleep(1.0)
        rospy.loginfo(f"{self.node_name}: Send UAV Hover cmd.")
        self.uav_cmd.cmd = UAVControlCMD.Hover
        self.uav_cmd.header.stamp = rospy.Time.now()
        self.control_cmd_pub.publish(self.uav_cmd)
        rospy.sleep(1.0)

    "执行圆形轨迹飞行"
    def fly_cirle(self):
        rate = rospy.Rate(10.0) #10 Hz
        rospy.loginfo(f"{self.node_name}: Start circle.")
        for i in range(self.num_points):
            theta = i * 2 * math.pi / self.num_points
            self.pose.pose.position.x = self.center_x + self.radius * math.cos(theta)
            self.pose.pose.position.y = self.center_y + self.radius * math.sin(theta)
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

                if abs(self.uav_state.position[0] - self.pose.pose.position.x) < 0.15 and \
                   abs(self.uav_state.position[1] - self.pose.pose.position.y) < 0.15:
                    break

                rate.sleep()

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
                    if idx == 2:
                        # 设置偏航到 45
                        rospy.sleep(1.0)
                        desired_yaw_deg = 90.0
                        self.uav_cmd.desired_yaw = math.radians(desired_yaw_deg)
                        self.uav_cmd.cmd = UAVControlCMD.XyzPosYaw
                        rospy.loginfo(f"{self.node_name}: Rotating to yaw {desired_yaw_deg} deg")
                        self.uav_cmd.header.stamp = rospy.Time.now()
                        self.control_cmd_pub.publish(self.uav_cmd)
                        rospy.sleep(1.0)  # 等旋转
                        break
                    
                    if idx == 6:
                        self.hover()
                        break

                    break

                rate.sleep()

        
    "返回原点(速度控制)"
    def return_to_origin_vel(self):
        rate = rospy.Rate(20.0)
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

            if abs(self.uav_state.position[0] - self.pose.pose.position.x) < 0.2 and \
               abs(self.uav_state.position[1] - self.pose.pose.position.y) < 0.2:
                rospy.sleep(1.0)
                break

            rate.sleep()

        "返回原点（位置控制）"
    def return_to_origin_pose(self):
        rate = rospy.Rate(20.0)

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
            dz = target_z - self.uav_state.position[2]

            if abs(dx) < 0.2 and abs(dy) < 0.2 and abs(dz) < 0.2:
                rospy.loginfo(f"{self.node_name}: Arrived at origin (position control).")
                rospy.sleep(1.0)
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
        self.custom_fly_pose()
        self.return_to_origin_pose()
        self.land()

if __name__ == "__main__":
    try:
        controller = CircleVelController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
