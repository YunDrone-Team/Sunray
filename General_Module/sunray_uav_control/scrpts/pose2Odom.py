#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist

twist_stamped = TwistStamped
odom_pub = None

def pose_stamped_to_odom(pose, twist):
    global odom_pub
    # 创建Odometry消息
    odom = Odometry()
    
    # 设置Odometry消息的时间戳
    odom.header.stamp = pose.header.stamp
    odom.header.frame_id = "world"
    
    # 设置Odometry消息的位姿
    odom.pose.pose = pose.pose
    
    # 设置Odometry消息的速度
    odom.twist.twist = twist.twist
    
    # 发布Odometry消息
    odom_pub.publish(odom)

def pose_stamped_callback(pose_stamped):
    # 将PoseStamped消息转换为Odometry消息
    global twist_stamped
    pose = pose_stamped
    twist = twist_stamped
    pose_stamped_to_odom(pose, twist)
    
    

def twist_stamped_callback(twist):
    # 保存TwistStamped消息
    global twist_stamped
    twist_stamped = twist

def main():
    # 初始化ROS节点
    rospy.init_node('pose_stamped_to_odom_node')
    
    # 创建订阅者，订阅PoseStamped消息
    pose_stamped_sub = rospy.Subscriber('/vrpn_client_node_1/uav1/pose', PoseStamped, pose_stamped_callback)
    
    # 创建订阅者，订阅TwistStamped消息
    twist_stamped_sub = rospy.Subscriber('/vrpn_client_node_1/uav1/twist', TwistStamped, twist_stamped_callback)
    
    # 创建发布者，发布Odometry消息
    global odom_pub
    odom_pub = rospy.Publisher('/odom_topic', Odometry, queue_size=10)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()
