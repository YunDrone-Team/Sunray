#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped

class CylinderPosePublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('cylinder_pose_publisher', anonymous=True)
        
        self.model_names = ["cylinder_25cm", "cylinder_25cm_clone_0", "cylinder_25cm_clone"]
        
        # 初始化发布者，每个圆柱体一个独立话题
        self.pose_pubs = {
            "cylinder_25cm": rospy.Publisher('/obstacle_1/pose', PoseStamped, queue_size=10),
            "cylinder_25cm_clone_0": rospy.Publisher('/obstacle_2/pose', PoseStamped, queue_size=10),
            "cylinder_25cm_clone": rospy.Publisher('/obstacle_3/pose', PoseStamped, queue_size=10)
        }
        
        # 初始化 Gazebo 服务客户端
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # 设置发布频率为 100Hz
        self.rate = rospy.Rate(100)  # 100Hz

    def get_current_poses(self):
        # 获取每个圆柱体的当前坐标并发布
        for model_name in self.model_names:
            try:
                # 调用服务获取模型状态
                state = self.get_model_state(model_name, "world")
                if state.success:
                    # 创建 PoseStamped 消息
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = "world"
                    pose_stamped.pose = state.pose
                    
                    # 发布到对应话题
                    self.pose_pubs[model_name].publish(pose_stamped)
                else:
                    rospy.logwarn("Failed to get state for {}".format(model_name))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed for {}: {}".format(model_name, e))

    def run(self):
        while not rospy.is_shutdown():
            self.get_current_poses()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = CylinderPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass