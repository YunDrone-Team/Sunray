# 控制程序，订阅每一个无人机的当前状态，发布targetpos指令
from std_msgs.msg import Int16
from sunray_msgs.msg import detection
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from sunray_msgs.msg import targetPos

class Center_Ctrl:
    def __init__(self):
        # 初始化ROS节点和CvBridge
        rospy.init_node('center_trol_node')
        self.topic_id1 = rospy.get_param('~uav_id', default= 1) 
        self.topic_id2 = rospy.get_param('~uav_id', default= 2) 
        self.topic_id3 = rospy.get_param('~uav_id', default= 3) 

        self.topic_name = rospy.get_param('~uav_name', default='uav')

        self.topic_name1 = self.topic_name + str(self.topic_id1)
        self.topic_name2 = self.topic_name + str(self.topic_id2)
        self.topic_name3 = self.topic_name + str(self.topic_id3)

        self.search_state1 = 0
        self.search_state2 = 0
        self.search_state3 = 0


        # self.detection_pose = detection()
        # self.detection_pose.id = 1
        # self.detection_pose.pose = np.array([1,1,1]) + self.init_pose

        self.pub_targetPos = rospy.Publisher("/targetPos", targetPos, queue_size=1)
        self.timer = 0
        self.send1_pub = rospy.Publisher(self.topic_name1 + "/send", Int16, queue_size=1)
        self.send2_pub = rospy.Publisher(self.topic_name2 + "/send", Int16, queue_size=1)
        self.send3_pub = rospy.Publisher(self.topic_name3 + "/send", Int16, queue_size=1)


    def state_callback1(self, data): 
        self.search_state1 = data.data # 订阅当前状态
    def state_callback2(self, data): 
        self.search_state2 = data.data # 订阅当前状态
    def state_callback3(self, data): 
        self.search_state3 = data.data # 订阅当前状态

    def init_pose_callback1(self,data):
        if data.pose_x >= 0:
            self.init_pose1 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取初始坐标
    def init_pose_callback2(self,data):
        if data.pose_x >= 0:
            self.init_pose2 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取初始坐标
    def init_pose_callback3(self,data):
        if data.pose_x >= 0:
            self.init_pose3 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取初始坐标
    

    def true_pose_callback1(self,data):
        self.true_pose1 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取真实坐标
    def true_pose_callback2(self,data):
        self.true_pose2 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取真实坐标
    def true_pose_callback3(self,data):
        self.true_pose3 = np.array([data.pose_x, data.pose_y, data.pose_z]) # 读取真实坐标

    def stage_callback1(self,data):
        if data.data == 1:
            self.send1_pub.publish(1)
            print(f"发布send1=1")
    def stage_callback2(self,data):
        if data.data == 1:
            self.send2_pub.publish(1)
            print(f"发布send2=1")
    def stage_callback3(self,data):
        if data.data == 1:
            self.send3_pub.publish(1)
            print(f"发布send3=1")


    def timer_callback(self,data):
        self.timer = self.timer + 1
        if(self.search_state1 & self.search_state2 & self.search_state3):
            p = targetPos()
            if self.timer % 5 == 0:
                p.target_id = 1
                p.pose_x = 15.25
                p.pose_y = 9
                p.pose_z = 0
                self.pub_targetPos.publish(p)
                print(f"发送坐标：{p.pose_x}，{p.pose_y}")
            if self.timer % 10 == 0:
                p.target_id = 2
                p.pose_x = 5.25
                p.pose_y = 3
                p.pose_z = 0
                self.pub_targetPos.publish(p)
                print(f"发送坐标：{p.pose_x}，{p.pose_y}")
            if self.timer % 15 == 0:
                p.target_id = 3
                p.pose_x = 25.25
                p.pose_y = -8
                p.pose_z = 0
                self.pub_targetPos.publish(p)
                print(f"发送坐标：{p.pose_x}，{p.pose_y}")
                self.timer = 0
            
            



    def listener(self):
        # 订阅多个无人机的状态
        rospy.Subscriber(self.topic_name1 + "/uav_search_state", Int16, self.state_callback1,queue_size=1)
        rospy.Subscriber(self.topic_name2 + "/uav_search_state", Int16, self.state_callback2,queue_size=1)
        rospy.Subscriber(self.topic_name3 + "/uav_search_state", Int16, self.state_callback3,queue_size=1)

        rospy.Subscriber(self.topic_name1 + "/init_pose", targetPos, self.init_pose_callback1,queue_size=1)
        rospy.Subscriber(self.topic_name2 + "/init_pose", targetPos, self.init_pose_callback2,queue_size=1)
        rospy.Subscriber(self.topic_name3 + "/init_pose", targetPos, self.init_pose_callback3,queue_size=1)
        
        rospy.Subscriber(self.topic_name1 + "/true_pose", targetPos, self.true_pose_callback1,queue_size=1)
        rospy.Subscriber(self.topic_name2 + "/true_pose", targetPos, self.true_pose_callback2,queue_size=1)
        rospy.Subscriber(self.topic_name3 + "/true_pose", targetPos, self.true_pose_callback3,queue_size=1)

        rospy.Subscriber(self.topic_name1 + "/stage", Int16, self.stage_callback1,queue_size=1)
        rospy.Subscriber(self.topic_name2 + "/stage", Int16, self.stage_callback2,queue_size=1)
        rospy.Subscriber(self.topic_name3 + "/stage", Int16, self.stage_callback3,queue_size=1)


        # 创建一个定时器，每隔0.2秒调用一次timer_callback函数
        timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

        rospy.spin() 

if __name__ == '__main__':
    Ctrl = Center_Ctrl()
    Ctrl.listener()       
