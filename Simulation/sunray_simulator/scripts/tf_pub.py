
import rospy
import tf
from nav_msgs.msg import Odometry

rospy.init_node('tf_publisher')

br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    odom_msg = rospy.wait_for_message('/uav1/sunray/gazebo_pose', Odometry)
    br.sendTransform((odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z),
                     (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     'base_link',
                     'world')
    print("#")