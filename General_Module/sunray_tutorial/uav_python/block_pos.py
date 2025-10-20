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
from sunray_logger import Logger, LogColor

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
    Logger.print_color(LogColor.green, node_name, '[block_pos] exit...')
    rospy.signal_shutdown('SIGINT')
    sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler)

def main():
    Logger.init_default()
    Logger.setPrintLevel(False)
    Logger.setPrintTime(False)
    Logger.setPrintToFile(False)
    Logger.setFilename('~/Documents/Sunray_log.txt')

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
            Logger.print_color(LogColor.red, node_name, ': Wait for UAV connect...')
    Logger.print_color(LogColor.green, node_name, ': UAV connected!')

    #设置无人机为命令控制模式（同时fmt切换到offboard模式）
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.CMD_CONTROL:
        uav_setup.cmd = UAVSetup.SET_CONTROL_MODE
        uav_setup.control_mode = 'CMD_CONTROL'
        uav_setup_pub.publish(uav_setup)
        Logger.print_color(LogColor.green, node_name, ': SET_CONTROL_MODE - [CMD_CONTROL].')
        rospy.sleep(1.0)
    Logger.print_color(LogColor.green, node_name, ': UAV control_mode set to [CMD_CONTROL] successfully!')

    #解锁无人机
    for i in range(5, 0, -1):
        Logger.print_color(LogColor.green, node_name, f': Arm UAV in {i} sec...')
        rospy.sleep(1.0)

    while not rospy.is_shutdown() and not uav_state.armed:
        uav_setup.cmd = UAVSetup.ARM
        uav_setup_pub.publish(uav_setup)
        Logger.print_color(LogColor.green, node_name, ': Arm UAV now.')
        rospy.sleep(1.0)
    Logger.print_color(LogColor.green, node_name, ': Arm UAV successfully!')

    #起飞无人机
    while not rospy.is_shutdown() and abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2:
        uav_cmd.cmd = UAVControlCMD.Takeoff
        control_cmd_pub.publish(uav_cmd)
        Logger.print_color(LogColor.green, node_name, ': Takeoff UAV now.')
        rospy.sleep(4.0)
    Logger.print_color(LogColor.green, node_name, ': Takeoff UAV successfully!')

    #以上：无人机已经起飞，接下来执行四边形轨迹飞行任务

    #悬停5秒
    rospy.sleep(5)
    Logger.print_color(LogColor.green, node_name, ': Send UAV Hover cmd.')
    uav_cmd.cmd = UAVControlCMD.Hover
    control_cmd_pub.publish(uav_cmd)
    rospy.sleep(5)

    # 定义四边形顶点，导入容器储存
    vertices = [
        (0.9, -0.9, 0.8),
        (0.9, 0.9, 0.8),
        (-0.9, 0.9, 0.8),
        (-0.9, -0.9, 0.8),
        (0.9, -0.9, 0.8),
        (0, 0, 0.8)
    ]

    for vertex in vertices:
        #输出当前目标点，在每个顶点发送控制指令直到无人机到达该点
        Logger.print_color(LogColor.green, node_name, f': go to point: {vertex}')

        while not rospy.is_shutdown():
            #如果收到停止任务指令，跳出循环，降落无人机
            if stop_flag:
                Logger.print_color(LogColor.green, node_name, ': Land UAV now.')
                uav_cmd.cmd = UAVControlCMD.Land
                control_cmd_pub.publish(uav_cmd)
                rospy.sleep(0.5)
                break

            #发送位置控制指令
            uav_cmd.cmd = UAVControlCMD.XyzPos
            uav_cmd.desired_pos = list(vertex)
            control_cmd_pub.publish(uav_cmd)

            #判断无人机是否到达目标点
            if (abs(uav_state.position[0] - vertex[0]) < 0.15 and
                abs(uav_state.position[1] - vertex[1]) < 0.15 and
                abs(uav_state.position[2] - vertex[2]) < 0.15):
                #停下来1秒等待无人机速度速度降下来
                rospy.sleep(1.0)
                break
            rate.sleep()

    #降落无人机
    while not rospy.is_shutdown() and uav_state.control_mode != UAVSetup.LAND_CONTROL and uav_state.landed_state != 1:
        uav_cmd.cmd = UAVControlCMD.Land
        control_cmd_pub.publish(uav_cmd)
        Logger.print_color(LogColor.green, node_name, ': Land UAV now.')
        rospy.sleep(4.0)

    #等待降落完成
    while not rospy.is_shutdown() and uav_state.landed_state != 1:
        Logger.print_color(LogColor.green, node_name, ': Landing')
        rospy.sleep(1.0)

    #降落完成
    Logger.print_color(LogColor.green, node_name, ': Land UAV successfully!')

    # 任务完成，退出
    Logger.print_color(LogColor.green, node_name, ': Demo finished, quit!')

if __name__ == '__main__':
    main()
