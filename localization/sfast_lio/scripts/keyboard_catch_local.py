#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import tty
import termios

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd, termios.TCSAFLUSH)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, old_settings)
    return ch

def keyboard_teleop():
    # 初始化ROS节点
    rospy.init_node('keyboard_teleop')

    # 创建一个发布者，发布到名为'cmd_vel'的主题上，消息类型为String
    pub = rospy.Publisher('keyboard_msgs', String, queue_size=10)

    # 设置循环频率
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
            # 读取键盘输入
            key = getch()

            # 根据按键发布不同的消息
            if key == 'w':
                msg = "Moving forward"
                print(msg)
            elif key == 's':
                msg = "Moving backward"
                print(msg)
            elif key == 'a':
                msg = "Moving left"
                print(msg)
            elif key == 'd':
                msg = "Moving right"
                print(msg)
            elif key == 'q':
                msg = "rotate left"
                print(msg)
            elif key == 'e':
                msg = "rotate right"
                print(msg)
            elif key == 'f':
                msg = "finish, start align"
                print(msg)
            elif key == 'j':
                msg = "inc rot resolu"
                print(msg)
            elif key == 'l':
                msg = "red rot resolu"
                print(msg)
            elif key == 'i':
                msg = "inc dis resolu"
                print(msg)
            elif key == 'k':
                msg = "red dis resolu"
                print(msg)
            else:
                msg = "No command"
                print(msg)
                continue

            # 发布消息
            pub.publish(msg)

            # 按照设置的频率延迟
            rate.sleep()

        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            break

if __name__ == '__main__':
    try:
        keyboard_teleop()
    except rospy.ROSInterruptException:
        pass
