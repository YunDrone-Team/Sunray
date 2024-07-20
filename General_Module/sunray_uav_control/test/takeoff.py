import rospy
from _UAVControlCMD import UAVControlCMD 
import time
def publish_takeoff_message():
    rospy.init_node('takeoff_publisher', anonymous=True)
    # rospy.Rate(10)
    pub = rospy.Publisher('/uav1/sunray/uav_control_cmd', UAVControlCMD, queue_size=10) 
    time.sleep(0.5)
    # 创建一个消息对象并填充数据
    takeoff_msg = UAVControlCMD()

    for _ in range(1):
        takeoff_msg.header.stamp = rospy.Time.now()
        print(takeoff_msg.header.stamp)
        takeoff_msg.cmd_id = takeoff_msg.Takeoff  # 这里假设cmd_id为1
        takeoff_msg.cmd = takeoff_msg.Takeoff  # 1表示takeoff命令
        # 填充其他字段

        # 发布消息
        pub.publish(takeoff_msg)
        # rospy.loginfo("Published takeoff message")
        time.sleep(1)

if __name__ == '__main__':
    try:
        publish_takeoff_message()
    except rospy.ROSInterruptException:
        pass