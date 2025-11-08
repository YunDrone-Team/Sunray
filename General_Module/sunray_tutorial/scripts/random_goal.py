#!/usr/bin/env

import rospy
import random
from geometry_msgs.msg import PoseStamped

def send_random_targets():

    # 初始化节点（只初始化一次）
    rospy.init_node('random_target_sender', anonymous=True)
    # 创建发布者（只创建一次）
    pub = rospy.Publisher('/goal_1', PoseStamped, queue_size=10)
    
    # 设置循环频率（单位：Hz），可根据需要调整
    rate = rospy.Rate(0.05)  
    
    # 构建消息模板
    msg = PoseStamped()
    msg.header.frame_id = ""  
    msg.pose.position.z = 0.8
    msg.pose.orientation.w = 1.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    
    rospy.loginfo("开始发布随机目标点...")
    rospy.loginfo("等待2秒...")
    rospy.sleep(2.0)  # 等待2秒，确保发布者连接成功
    
    try:
        while not rospy.is_shutdown():
            
            # 生成-2.5到2.5之间的随机x和y
            x = random.uniform(-1.5, 1.5)
            y = random.uniform(-1.5, 1.5)
            
            # 更新消息内容
            msg.header.stamp = rospy.Time.now()  # 更新时间戳
            msg.pose.position.x = x
            msg.pose.position.y = y
            
            # 发布消息
            pub.publish(msg)
            rospy.loginfo(f"发送目标点: ({x:.2f}, {y:.2f})")  # 保留两位小数显示
            
            # 按照设定频率休眠
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")

if __name__ == '__main__':
    send_random_targets()
