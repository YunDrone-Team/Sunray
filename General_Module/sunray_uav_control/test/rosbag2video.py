#!/home/cvpr/anaconda3/envs/ros_py37/bin/python3.7
# -*- coding: utf-8 -*-

"""
订阅图像并保存为视频
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 

class VideoRecorder:
    def __init__(self, path):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/web_cam/image_raw', Image, self.image_callback)
        self.frames = []
        self.recording = False
        self.path = path
 
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.recording:
                self.frames.append(cv_image)
        except Exception as e:
            print(e)
 
    def start_recording(self):
        print('start recording')
        self.frames = []
        self.recording = True

    def stop_recording(self):
        self.recording = False
        if self.frames:
            self.save_video()
 
    def save_video(self):
        height = self.frames[0].shape[0]
        weight = self.frames[0].shape[1]
        print(height, weight)
        fps = 30
        # fourcc = cv.VideoWriter_fourcc('M', 'J', 'P', 'G') 用于avi格式的生成
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 用于mp4格式的生成
        videowriter = cv2.VideoWriter(self.path, fourcc, fps, (weight, height))  # 创建一个写入视频对象
        for img in self.frames:
            videowriter.write(img)

        videowriter.release()
        print('save to', self.path)

if __name__ == '__main__':
    rospy.init_node('video_recorder_node', anonymous=True)
    path = 'new.mp4'
    recorder = VideoRecorder(path)

    try:
        while not rospy.is_shutdown():
            cmd = input("Enter 'start' to begin recording or 'stop' to stop recording: ")
            if cmd == 's':
                recorder.start_recording()
            elif cmd == 'q':
                recorder.stop_recording()
    except rospy.ROSInterruptException:
        pass
