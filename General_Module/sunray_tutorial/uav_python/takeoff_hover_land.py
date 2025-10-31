#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能:无人机一键解锁、起飞、悬停、降落演示(Python版)
"""
import rospy
from std_msgs.msg import Header
from sunray_msgs.msg import UAVState, UAVSetup, UAVControlCMD
import signal
import sys
import time

node_name = 'takeoff_hover_land_py'
uav_state = UAVState()
uav_setup = UAVSetup()
uav_cmd = UAVControlCMD()

# 信号处理，支持Ctrl+C安全退出
def sigint_handler(sig, frame):
    rospy.loginfo('[takeoff_hover_land_py] exit...')
    rospy.signal_shutdown('SIGINT')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

def uav_state_callback(msg):
    global uav_state
    uav_state = msg

def main():
    rospy.init_node(node_name)
    rate = rospy.Rate(20)
    uav_id = rospy.get_param('~uav_id', 1)
    uav_name = rospy.get_param('~uav_name', 'uav')
    uav_name = '/' + uav_name + str(uav_id)

    uav_state_sub = rospy.Subscriber(uav_name + '/sunray/uav_state', UAVState, uav_state_callback)
    control_cmd_pub = rospy.Publisher(uav_name + '/sunray/uav_control_cmd', UAVControlCMD, queue_size=1)
    uav_setup_pub = rospy.Publisher(uav_name + '/sunray/setup', UAVSetup, queue_size=1)

    # 初始化指令
    uav_cmd.header = Header()
    uav_cmd.cmd = UAVControlCMD.Hover
    uav_cmd.desired_pos = [0.0, 0.0, 0.0]
    uav_cmd.desired_vel = [0.0, 0.0, 0.0]
    uav_cmd.desired_acc = [0.0, 0.0, 0.0]
    uav_cmd.desired_att = [0.0, 0.0, 0.0]
    uav_cmd.desired_yaw = 0.0
    uav_cmd.desired_yaw_rate = 0.0

    time.sleep(0.5)

    # 等待FMT连接
    times = 0
    while not rospy.is_shutdown() and not uav_state.connected:
        rospy.sleep(1.0)
        times += 1
        if times > 5:
            rospy.logwarn('Wait for UAV connect...')
    rospy.loginfo('UAV connected!')

    # 切换到指令控制模式
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

    # 悬停
    rospy.sleep(5)
    rospy.loginfo('Send UAV Hover cmd.')
    uav_cmd.cmd = UAVControlCMD.Hover
    control_cmd_pub.publish(uav_cmd)
    rospy.sleep(5)

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
    rospy.loginfo('Land UAV successfully!')

    #Demo结束，安全退出
    rospy.loginfo('Demo finished, quit!')

if __name__ == '__main__':
    main()
