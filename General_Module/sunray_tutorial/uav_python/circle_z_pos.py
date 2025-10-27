#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能:无人机圆形轨迹飞行(XyVelZPos接口,Python版)
"""
import rospy
from sunray_msgs.msg import UAVState, UAVSetup, UAVControlCMD
import signal
import sys
import time
import numpy as np

node_name = 'circle_z_pos_py'
uav_state = UAVState()
uav_setup = UAVSetup()
uav_cmd = UAVControlCMD()

#无人机状态回调函数
def uav_state_callback(msg):
    global uav_state
    uav_state = msg

# 信号处理，支持Ctrl+C安全退出
def sigint_handler(sig, frame):
    rospy.loginfo('[circle_z_pos] exit...')
    rospy.signal_shutdown('SIGINT')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

def main():
    rospy.init_node(node_name)
    rate = rospy.Rate(20)
    # 获取无人机ID
    uav_id = rospy.get_param('~uav_id', 1)
    # 获取无人机名称
    uav_name = rospy.get_param('~uav_name', 'uav')
    uav_name = '/' + uav_name + str(uav_id)

    #订阅无人机状态
    rospy.Subscriber(uav_name + '/sunray/uav_state', UAVState, uav_state_callback)
    # 发布控制指令
    control_cmd_pub = rospy.Publisher(uav_name + '/sunray/uav_control_cmd', UAVControlCMD, queue_size=1)
    # 发布设置指令
    uav_setup_pub = rospy.Publisher(uav_name + '/sunray/setup', UAVSetup, queue_size=1)

    # 初始化指令
    uav_cmd.cmd = UAVControlCMD.Hover
    uav_cmd.desired_pos = [0.0, 0.0, 0.0]
    uav_cmd.desired_vel = [0.0, 0.0, 0.0]
    uav_cmd.desired_acc = [0.0, 0.0, 0.0]
    uav_cmd.desired_att = [0.0, 0.0, 0.0]
    uav_cmd.desired_yaw = 0.0
    uav_cmd.desired_yaw_rate = 0.0

    time.sleep(0.5)
    # 初始化检查，等待FMT连接
    times = 0
    while not rospy.is_shutdown() and not uav_state.connected:
        rospy.sleep(1.0)
        times += 1
        if times > 5:
            rospy.logwarn('Wait for UAV connect...')
    rospy.loginfo('UAV connected!')

    # 设置控制模式为CMD_CONTROL（同时，FMT会自动切换到OFFBOARD模式）
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.CMD_CONTROL:
        uav_setup.cmd = UAVSetup.SET_CONTROL_MODE
        uav_setup.control_mode = 'CMD_CONTROL'
        uav_setup_pub.publish(uav_setup)
        rospy.loginfo('SET_CONTROL_MODE - [CMD_CONTROL].')
        rospy.sleep(1.0)
    rospy.loginfo('UAV control_mode set to [CMD_CONTROL] successfully!')

    # 解锁无人机
    for i in range(5, 0, -1):
        rospy.loginfo(f'Arm UAV in {i} sec...')
        rospy.sleep(1.0)
    while not rospy.is_shutdown() and not uav_state.armed:
        uav_setup.cmd = UAVSetup.ARM
        uav_setup_pub.publish(uav_setup)
        rospy.loginfo('Arm UAV now.')
        rospy.sleep(1.0)
    rospy.loginfo('Arm UAV successfully!')

    # 起飞无人机
    while not rospy.is_shutdown() and abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2:
        uav_cmd.cmd = UAVControlCMD.Takeoff
        control_cmd_pub.publish(uav_cmd)
        rospy.loginfo('Takeoff UAV now.')
        rospy.sleep(4.0)
    rospy.loginfo('Takeoff UAV successfully!')

    #以上：无人机起飞完成，以下：开始圆形轨迹飞行

    # 悬停
    rospy.sleep(5)
    rospy.loginfo('Send UAV Hover cmd.')
    uav_cmd.cmd = UAVControlCMD.Hover
    control_cmd_pub.publish(uav_cmd)
    rospy.sleep(5)

    #==================================== 轨迹控制关键代码段 BEGIN（二次开发） ====================================
    # 定义圆形中心和半径
    center_x = 0
    center_y = 0
    radius = 1.0

    # 定义圆形点数
    num_points = 50
    # 控制参数
    k_p = 1.0
    max_vel = 1.0
    height = 0.8
    # 圈数
    num_circle = 2

    rospy.loginfo('Start circle.')
    for n in range(num_circle):
        for i in range(num_points):
            # 计算目标点
            theta = i * 2 * np.pi / num_points
            target_x = center_x + radius * np.cos(theta)
            target_y = center_y + radius * np.sin(theta)
            target_z = height
            start_time = rospy.Time.now()
            # 控制无人机飞向目标点
            while not rospy.is_shutdown():
                # 计算当前位置与目标位置的误差
                dx = target_x - uav_state.position[0]
                dy = target_y - uav_state.position[1]

                # 计算速度指令
                vx = k_p * dx
                vy = k_p * dy

                # 限制速度指令的范围
                vx = max(min(vx, max_vel), -max_vel)
                vy = max(min(vy, max_vel), -max_vel)

                # 发布xy速度z位置指令
                uav_cmd.cmd = UAVControlCMD.XyVelZPos
                uav_cmd.desired_vel = [vx, vy, 0.0]
                uav_cmd.desired_pos[2] = height
                control_cmd_pub.publish(uav_cmd)
                # 开环定时退出
                if rospy.Time.now() - start_time > rospy.Duration(0.6):
                    break
                rate.sleep()

    # 回到原点
    target_x = 0
    target_y = 0
    target_z = height
    while not rospy.is_shutdown():
        # 计算当前位置与目标位置的误差
        dx = target_x - uav_state.position[0]
        dy = target_y - uav_state.position[1]

        # 计算速度指令
        vx = k_p * dx
        vy = k_p * dy

        # 限制速度指令的范围
        vx = max(min(vx, max_vel), -max_vel)
        vy = max(min(vy, max_vel), -max_vel)

        # 发布xy速度z位置指令
        uav_cmd.cmd = UAVControlCMD.XyVelZPos
        uav_cmd.desired_vel = [vx, vy, 0.0]
        uav_cmd.desired_pos[2] = height
        uav_cmd.desired_yaw = 0.0
        control_cmd_pub.publish(uav_cmd)

        # 检查是否到达目标点
        if abs(uav_state.position[0] - target_x) < 0.2 and abs(uav_state.position[1] - target_y) < 0.2:
            rospy.sleep(1.0)
            break
        rate.sleep()
    #==================================== 轨迹控制关键代码段 END（二次开发） ======================================
    
    # 降落无人机
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.LAND_CONTROL and uav_state.landed_state != 1:
        uav_cmd.cmd = UAVControlCMD.Land
        control_cmd_pub.publish(uav_cmd)
        rospy.loginfo('Land UAV now.')
        rospy.sleep(4.0)
    
    # 等待降落完成
    while not rospy.is_shutdown() and uav_state.landed_state != 1:
        rospy.loginfo('Landing')
        rospy.sleep(1.0)

    # 降落完成
    rospy.loginfo('Land UAV successfully!')
    
    #Demo 完成，退出
    rospy.loginfo('Demo finished, quit!')

if __name__ == '__main__':
    main()
