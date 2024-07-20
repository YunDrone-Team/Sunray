#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from _UAVControlCMD import UAVControlCMD 

def publish_control_cmd():
    # 初始化ROS节点
    rospy.init_node('control_cmd_publisher', anonymous=True)

    # 创建一个Publisher，发布到 'drone_control' 主题
    pub = rospy.Publisher('/uav1/sunray/uav_control_cmd', UAVControlCMD, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # 创建消息对象
        msg = UAVControlCMD()

        # 设置消息头
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        # 设置命令ID和命令类型
        msg.cmd_id = 6 
        msg.cmd = UAVControlCMD.Hover  # Init = 0 Takeoff = 1 Hover = 2 Land = 3 XYZ_POS = 4 XY_VEL_Z_POS = 5 XYZ_VEL = 6
                                         # XYZ_POS_BODY = 7 XYZ_VEL_BODY = 8 XY_VEL_Z_POS_BODY = 9 TRAJECTORY = 10 XYZ_ATT = 11 LAT_LON_ALT = 12

        # 设置期望值（根据命令类型不同，可以设置不同的期望值）
        msg.desired_pos = [0.0, 0.0, 0.0]  # 期望位置
        msg.desired_vel = [0.0, 1.0, 0.0]  # 期望速度
        msg.desired_acc = [0.0, 0.0, 0.0]  # 期望加速度
        msg.desired_att = [0.0, 0.0, 0.0]  # 期望姿态
        msg.desired_thrust = 0.0  # 期望推力（例如：50%）
        msg.desired_yaw = 0.0  # 期望偏航角
        msg.desired_yaw_rate = 0.0  # 期望偏航速率
        msg.latitude = 0.0  # 经度
        msg.longitude = 0.0  # 纬度
        msg.altitude = 0.0  # 高度

        # 发布消息
        pub.publish(msg)

        # 打印日志信息
        rospy.loginfo("Published control command: %s", msg)

        # 按照指定频率休眠
        rate.sleep()
        # break

if __name__ == '__main__':
    try:
        publish_control_cmd()
    except rospy.ROSInterruptException:
        pass