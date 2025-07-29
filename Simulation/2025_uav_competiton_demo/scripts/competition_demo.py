#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy 
from sunray_msgs.msg import UAVState, UAVControlCMD, UAVSetup 
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import signal 
import sys 
import math 
from mavros_msgs.srv import ParamSet, ParamSetRequest
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
        

        self.obstacles_sub = [
            rospy.Subscriber('/obstcale_1/pose', PoseStamped, self.obstacle_cb, callback_args=0),
            rospy.Subscriber('/obstcale_2/pose', PoseStamped, self.obstacle_cb, callback_args=1),
            rospy.Subscriber('/obstcale_3/pose', PoseStamped, self.obstacle_cb, callback_args=2),
            rospy.Subscriber('/delivery_target/pose', PoseStamped, self.obstacle_cb, callback_args=3),
        ]

        "使用参数设置服务对舵机进行控制"
        rospy.wait_for_service(self.uav_name + "/mavros/param/set")
        self.servo_ctrl = rospy.ServiceProxy(
            self.uav_name + "/mavros/param/set", ParamSet
        )
        self.servo_mode = ParamSetRequest() # 舵机模式 默认是aux1控制（407），使用offboard控制时，切换到最大值（2）
        self.servo_mode.param_id = "PWM_MAIN_FUNC5"
        self.servo_angel = ParamSetRequest() # 舵机角度 （200 - 2450）
        self.servo_angel.param_id = "PWM_MAIN_MAX5"

        "后续修改这两个参数以适配具体安装方式"
        self.servo_init_angle = 1600        # 初始角度
        self.servo_drop_angle = 2200        # 投放角度

        "初始化APF路径规划器"
        self.start = (-2.0, -2.0)  # 起点坐标
        # self.start = (-1.0, -1.0)  # 起点坐标
        # self.goal = (1.27, -0.5)   # 目标
        self.goal = (1.0, 2.0)   # 目标
        self.obstacles = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]  # 障碍物列表
        # self.obstacles = [(0.0, 0.0)]  # 障碍物列表
        # self.safe_zone = (-1.0, 2.5, -2.5, -0.5)  # 安全区域边界 Xmin Xmax Ymin Ymax
        self.safe_zone = (-3.0, 3.0, -3.0, 3.0)  # 安全区域边界 Xmin Xmax Ymin Ymax
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
            [-0.559, -1.176, self.height],
            [0.706, -0.761, self.height],
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

    "障碍物回调函数"
    def obstacle_cb(self, msg, index):
        if index == 3:
            self.points[7] = [msg.pose.position.x, msg.pose.position.y, self.height]
        else:
            self.obstacles[index] = [msg.pose.position.x, msg.pose.position.y]

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
    def hover(self):
        rospy.sleep(1.0)
        rospy.loginfo(f"{self.node_name}: Send UAV Hover cmd.")
        self.uav_cmd.cmd = UAVControlCMD.Hover
        self.uav_cmd.header.stamp = rospy.Time.now()
        self.control_cmd_pub.publish(self.uav_cmd)
        rospy.sleep(1.0)
        
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


    "舵机控制接口"
    "舵机使用指南："
    "使用servo_setangle需要先运行self.servo_setmode(2)把舵机模式改成最大值输出"
    # CONSTANT_MIN = 1
    # CONSTANT_MAX = 2
    # RC_AUX1 = 407
    def servo_setmode(self, modeID):
        if modeID == 1 or 2 or 407:
            self.servo_mode.value.integer = modeID
            try:
                if self.servo_ctrl.call(self.servo_mode).success:
                    rospy.loginfo(f"{self.node_name}: Turn Servo mode {modeID}!")
                else:
                    rospy.loginfo(f"{self.node_name}: Turn Servo mode False-->no_success")
            except rospy.ROSInterruptException:
                pass
        else:
            rospy.loginfo(f"{self.node_name}: Turn Servo mode False-->modeID_unuse")

    "舵机角度控制接口"
    # angle :1600 - 2200
    def servo_setangle(self, angle):
        if 1600 <= angle and angle <= 2200:
            self.servo_angel.value.integer = angle
            try:
                if self.servo_ctrl.call(self.servo_angel).success:
                    rospy.loginfo(f"{self.node_name}: Set Servo angle {angle}!")
                else:
                    rospy.loginfo(f"{self.node_name}: Set Servo angle False-->no_success")
            except rospy.ROSInterruptException:
                pass
        else:
            rospy.loginfo(f"{self.node_name}: Set Servo angle False-->angle_limit")

    "测试函数"
    def test_demo(self):
        rospy.init_node("test_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        self.set_control_mode()
        self.arm_uav()
        self.takeoff()
        self.hover()
        rospy.sleep(1.0)

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

            marker = Marker()
            marker.header.frame_id = "world"
            marker.type = Marker.ARROW
            marker.scale.x = 0.05; marker.scale.y = 0.1
            marker.color.r = 1.0; marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.1)
            
            start = Point(*self.uav_state.position)
            end = Point(
                self.uav_state.position[0] + vel[0],
                self.uav_state.position[1] + vel[1],
                self.uav_state.position[2] + vz
            )
            marker.points = [start, end]
            self.velocity_pub.publish(marker)

            if abs(dx) < 0.1 and abs(dy) < 0.1:
                break
            rate.sleep()

        self.hover()
        # self.land()

    "测试APF路径规划器"
    def plan_test(self):
        rospy.init_node("plan_test_demo", anonymous=True)
        self.node_name = rospy.get_name()
        self.wait_for_connection()
        self.set_control_mode()
        self.arm_uav()
        self.takeoff()
        self.hover()

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
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                break
            rate.sleep()

        self.hover()

        for idx, point in enumerate(self.points):
            rospy.loginfo(f"{self.node_name}: Flying to point {idx+1}: {point}")
            target_x, target_y, target_z = point

            if idx >= 2:
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
                        
                        if idx == 6:
                            self.hover()
                            rospy.sleep(5.0)
                            break

                        if idx == 7:
                            desired_yaw_deg = 0.0
                            self.uav_cmd.desired_yaw = math.radians(desired_yaw_deg)
                            self.uav_cmd.cmd = UAVControlCMD.XyzPosYaw
                            self.uav_cmd.header.stamp = rospy.Time.now()
                            self.control_cmd_pub.publish(self.uav_cmd)
                            rospy.sleep(2.0)  # 等旋转
                            if (abs(self.uav_state.position[0] - target_x) < 0.1 and
                                abs(self.uav_state.position[1] - target_y) < 0.1 and
                                abs(self.uav_state.position[2] - target_z) < 0.1):
                                self.hover()
                                rospy.sleep(1.0)
                                self.servo_setangle(self.servo_drop_angle)
                                rospy.sleep(1.0)
                            else:
                                self.pose_ctrl(target_x, target_y, target_z - 0.2)
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

        self.servo_setmode(2) # 设置舵机输出模式为constmax
        self.servo_setangle(self.servo_init_angle)

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
        # controller.run()
        controller.test_demo()
        # controller.plan_test()
    except rospy.ROSInterruptException:
        pass
