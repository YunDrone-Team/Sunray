#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DecompressBridge:
    def __init__(self):
        rospy.init_node("decompress_bridge")

        input_topic = rospy.get_param("~input_topic", "/camera/video_stream")
        output_topic = rospy.get_param("~output_topic", "/usb_cam/image_raw")

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(output_topic, Image, queue_size=1)
        rospy.Subscriber(input_topic, CompressedImage, self.callback)

        rospy.loginfo(f"decompress_bridge: {input_topic} -> {output_topic}")
        rospy.spin()

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            self.pub.publish(img_msg)
        except Exception as e:
            rospy.logwarn(f"解压失败: {e}")

if __name__ == "__main__":
    DecompressBridge()
