import rospy
from _UAVControlCMD import UAVControlCMD
from std_msgs.msg import Bool

def callback(data):
    # print(data.data)
    rospy.loginfo("Received control command: cmd_id={}, cmd={}".format(data.cmd_id, data.cmd))
    # 在这里可以根据需要处理接收到的消息

def subscribe_to_uav_control_cmd():
    rospy.init_node('uav_control_cmd_subscriber', anonymous=True)
    rospy.Subscriber('/uav1/sunray/uav_control_cmd', UAVControlCMD, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_to_uav_control_cmd()
    except rospy.ROSInterruptException:
        pass