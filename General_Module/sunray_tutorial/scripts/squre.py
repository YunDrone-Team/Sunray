import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



# 定义回调函数
def image_callback(data):
    global last_time
    if rospy.Time.now() - last_time < rospy.Duration(0.1):
        return
    last_time = rospy.Time.now()
    # 将ROS图像消息转换为OpenCV图像
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # 灰度化
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 边缘检测
    edges = cv2.Canny(gray, 150, 200)

    # 轮廓检测
    contours, _ = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    # 绘制矩形框
    for contour in contours:
        # if cv2.contourArea(contour) > 200:
        x, y, w, h = cv2.boundingRect(contour)
        # 判断是否接近正方形
        if abs(w - h) < 10 and w > 100:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 将OpenCV图像转换为ROS图像消息
    ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

    # 发布图像消息
    image_pub.publish(ros_image)


# 初始化ROS节点
rospy.init_node('find_shapes', anonymous=True)

# 创建一个订阅者，订阅图像消息
image_sub = rospy.Subscriber('/sunray/camera/monocular_front/image_raw', Image, image_callback)

last_time = rospy.Time.now()
# 创建一个CvBridge对象
bridge = CvBridge()

# 创建一个发布者，发布图像消息
image_pub = rospy.Publisher('shapes_image', Image, queue_size=10)

# 保持ROS节点运行
rospy.spin()
