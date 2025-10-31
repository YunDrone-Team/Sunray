#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能:无人机四边形轨迹飞行(Python版)
"""
import rospy
from std_msgs.msg import Empty
from sunray_msgs.msg import UAVState, UAVSetup, UAVControlCMD
import signal
import sys
import time

node_name = 'block_pos_py'
uav_state = UAVState()
uav_setup = UAVSetup()
uav_cmd = UAVControlCMD()
stop_flag = False

#处理停止任务的空消息，如果收到消息，设置stop_flag为True
def stop_tutorial_cb(msg):
    global stop_flag
    stop_flag = True

#无人机状态回调函数
def uav_state_callback(msg):
    global uav_state
    uav_state = msg

# 信号处理，支持Ctrl+C安全退出
def sigint_handler(sig, frame):
    rospy.loginfo('[block_pos] exit...')
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
    #订阅停止任务话题
    rospy.Subscriber(uav_name + '/sunray/stop_tutorial', Empty, stop_tutorial_cb)
    #发布控制指令
    control_cmd_pub = rospy.Publisher(uav_name + '/sunray/uav_control_cmd', UAVControlCMD, queue_size=1)
    #发布设置指令
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
    times = 0
    #初始化检查；等待FMT连接
    while not rospy.is_shutdown() and not uav_state.connected:
        rospy.sleep(1.0)
        times += 1
        if times > 5:
            rospy.logwarn('Wait for UAV connect...')
    rospy.loginfo('UAV connected!')

    #设置无人机为命令控制模式（同时fmt切换到offboard模式）
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.CMD_CONTROL:
        uav_setup.cmd = UAVSetup.SET_CONTROL_MODE
        uav_setup.control_mode = 'CMD_CONTROL'
        uav_setup_pub.publish(uav_setup)
        rospy.loginfo('SET_CONTROL_MODE - [CMD_CONTROL].')
        rospy.sleep(1.0)
    rospy.loginfo('UAV control_mode set to [CMD_CONTROL] successfully!')

    #解锁无人机
    for i in range(5, 0, -1):
        rospy.loginfo(f'Arm UAV in {i} sec...')
        rospy.sleep(1.0)

    while not rospy.is_shutdown() and not uav_state.armed:
        uav_setup.cmd = UAVSetup.ARM
        uav_setup_pub.publish(uav_setup)
        rospy.loginfo('Arm UAV now.')
        rospy.sleep(1.0)
    rospy.loginfo('Arm UAV successfully!')

    #起飞无人机
    while not rospy.is_shutdown() and abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2:
        uav_cmd.cmd = UAVControlCMD.Takeoff
        control_cmd_pub.publish(uav_cmd)
        rospy.loginfo('Takeoff UAV now.')
        rospy.sleep(4.0)
    rospy.loginfo('Takeoff UAV successfully!')

    #以上：无人机已经起飞，接下来执行四边形轨迹飞行任务

    #悬停5秒
    rospy.sleep(5)
    rospy.loginfo('Send UAV Hover cmd.')
    uav_cmd.cmd = UAVControlCMD.Hover
    control_cmd_pub.publish(uav_cmd)
    rospy.sleep(5)

    #==================================== 轨迹控制关键代码段 BEGIN (二次开发)====================================

    # 定义四边形顶点
    vertices = [
        (0.9, -0.9, 0.8),
        (0.9, 0.9, 0.8),
        (-0.9, 0.9, 0.8),
        (-0.9, -0.9, 0.8),
        (0.9, -0.9, 0.8),
        (0, 0, 0.8)
    ]

    # [轨迹控制优化] 离散化每条边，生成所有中间点
    def interpolate_points(p1, p2, num_segments):
        points = []
        for i in range(1, num_segments+1):
            t = i / num_segments
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            z = p1[2] + t * (p2[2] - p1[2])
            points.append((x, y, z))
        return points

    all_points = []
    num_segments = 20  # 每条边分成10段
    for i in range(len(vertices)-1):
        p1 = vertices[i]
        p2 = vertices[i+1]
        all_points.append(p1)
        all_points.extend(interpolate_points(p1, p2, num_segments))
    all_points.append(vertices[-1])

    # 依次飞往所有离散点（前瞄点方式，轨迹更平滑）
    lookahead_idx = 0
    lookahead_dist = 0.25  # 距离阈值，越小越精确但易抖动，越大越顺滑
    while not rospy.is_shutdown() and lookahead_idx < len(all_points):
        # 计算当前前瞄点索引（确保不超出数组范围）
        lookahead_idx = min(lookahead_idx, len(all_points)-1)
        target = all_points[lookahead_idx]
        uav_cmd.cmd = UAVControlCMD.XyzPos
        uav_cmd.desired_pos = list(target)
        control_cmd_pub.publish(uav_cmd)

        # 判断是否到达当前目标点，达到则切换到下一个点
        if (abs(uav_state.position[0] - target[0]) < lookahead_dist and
            abs(uav_state.position[1] - target[1]) < lookahead_dist and
            abs(uav_state.position[2] - target[2]) < lookahead_dist):
            lookahead_idx += 1

        # 检查是否收到停止指令
        if stop_flag:
            rospy.loginfo('Land UAV now.')
            uav_cmd.cmd = UAVControlCMD.Land
            control_cmd_pub.publish(uav_cmd)
            rospy.sleep(0.5)
            break
        rate.sleep()
    #==================================== 轨迹控制关键代码段 END (二次开发)======================================

    #降落无人机
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.LAND_CONTROL and uav_state.landed_state != 1:
        uav_cmd.cmd = UAVControlCMD.Land
        control_cmd_pub.publish(uav_cmd)
        rospy.loginfo('Land UAV now.')
        rospy.sleep(4.0)

    #等待降落完成    
    while not rospy.is_shutdown() and uav_state.landed_state != 1:
        rospy.loginfo('Landing')
        rospy.sleep(1.0)
    
    #降落完成
    rospy.loginfo('Land UAV successfully!')

    #任务完成，退出
    rospy.loginfo('Demo finished, quit!')

if __name__ == '__main__':
    main()
