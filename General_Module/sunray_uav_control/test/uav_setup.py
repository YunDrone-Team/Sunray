import rospy
from _UAVSetup import UAVSetup

def send_custom_message():
    # 初始化ROS节点
    rospy.init_node('custom_message_sender', anonymous=True)

    # 创建一个发布者，指定要发布的消息类型
    pub = rospy.Publisher('/uav1/sunray/setup', UAVSetup, queue_size=10)

    # 创建一个消息对象
    custom_msg = UAVSetup()

    # 设置消息字段的值
    custom_msg.header.stamp = rospy.Time.now()
    custom_msg.cmd = 0  # 设置cmd字段为ARMING
    custom_msg.arming = True
    custom_msg.px4_mode = "OFFBOARD"
    custom_msg.control_state = "COMMAND_CONTROL"

    # 发布消息
    pub.publish(custom_msg)

    # 延迟一段时间后关闭节点
    rospy.sleep(1)
    rospy.signal_shutdown("Custom message sent")

if __name__ == '__main__':
    try:
        send_custom_message()
    except rospy.ROSInterruptException:
        pass