#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能:无人机六边形轨迹飞行(XyzPosYawBody接口,Python版)
"""
import rospy
from sunray_msgs.msg import UAVState, UAVSetup, UAVControlCMD
import signal
import sys
import time
import numpy as np

node_name = 'pos_body_hexagon_py'
uav_state = UAVState()
uav_setup = UAVSetup()
uav_cmd = UAVControlCMD()

#无人机状态回调函数
def uav_state_callback(msg):
    global uav_state
    uav_state = msg

# 信号处理，支持Ctrl+C安全退出
def sigint_handler(sig, frame):
    rospy.loginfo('[pos_body_hexagon] exit...')
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

    # 初始化检查，等待FMT连接
    time.sleep(0.5)
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

    #以上：无人机起飞完成，以下：开始六边形轨迹飞行

    # 悬停
    rospy.sleep(5)
    rospy.loginfo('Send UAV Hover cmd.')
    uav_cmd.cmd = UAVControlCMD.Hover
    control_cmd_pub.publish(uav_cmd)
    rospy.sleep(5)

    #==================================== 轨迹控制关键代码段 BEGIN（二次开发） ====================================
    # 六边形轨迹主循环
    vertex = (1, 0, 0)
    for i in range(8):
        uav_cmd.cmd = UAVControlCMD.XyzPosYawBody
        uav_cmd.desired_pos[0] = vertex[0]
        uav_cmd.desired_pos[1] = vertex[1]
        uav_cmd.desired_pos[2] = 1 - uav_state.position[2]
        uav_cmd.desired_yaw = 0.0
        control_cmd_pub.publish(uav_cmd)
        rospy.sleep(5)

        # 计算下一个顶点位置
        if i == 0 or i == 6:
            yaw_deg = 120
        else:
            yaw_deg = 60
        if i == 7:
            break

        uav_cmd.cmd = UAVControlCMD.XyzPosYawBody
        uav_cmd.desired_pos = [0, 0, 0]
        uav_cmd.desired_yaw = yaw_deg / 180.0 * np.pi
        control_cmd_pub.publish(uav_cmd)
        rospy.loginfo(f'Back to origin, set yaw={yaw_deg}')
        rospy.sleep(2)
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
