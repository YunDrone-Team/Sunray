import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
 
# 初始化ROS节点
rospy.init_node('pointcloud_publisher', anonymous=True)
 
# 创建ROS Publisher
pub = rospy.Publisher('your_pointcloud_topic', PointCloud2, queue_size=10)
 
rate = rospy.Rate(1)  # 设置发布频率为1Hz，你可以根据需要调整
 
while not rospy.is_shutdown():
    # 创建一个NumPy数组，包含点云数据
    point_cloud_data = np.array([
        [1.0, 2.0, 0.0],  # 三个坐标 (x, y, z) 和一个强度值
        [2.0, 3.0, 0.0],
        [3.0, 4.0, 0.0],
        # 添加更多点云数据行
    ])
    # 创建Pointcloud2消息
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'your_frame_id'  # 设置帧ID
    cloud_msg = point_cloud2.create_cloud_xyz32(header, point_cloud_data)
 
    pub.publish(cloud_msg)
 
    print(cloud_msg.header.stamp.to_sec())
 
    rate.sleep()