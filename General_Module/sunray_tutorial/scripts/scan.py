#!/usr/bin/env python
import math
import sys
import time
from matplotlib import pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import threading
from scipy import signal

from uav_msg._UAVControlCMD import UAVControlCMD
from uav_msg._UAVState import UAVState
from uav_msg._UAVSetup import UAVSetup


setup_pub = None
cmd_pub = None
uav_state = None

setup_msg = UAVSetup()
cmd_msg = UAVControlCMD()
# 雷达点位
x_y_list = []
global_gird = None
scan_data = {}
rlock = threading.RLock()

# 任务
task_flag = 0
task_2_1 = False
task_2_2 = False
task_3_1 = False
task_3_2 = False

last_left = -1
last_right = -1

# 搜索方向
search_flag = 0

def pub_cmd():
    global cmd_pub
    global cmd_msg
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.cmd_id = cmd_msg.cmd_id + 1
    cmd_pub.publish(cmd_msg)
    # print(cmd_msg.desired_pos[2])

def search_column():
    global cmd_msg
    global task_flag
    global scan_data
    global task_2_1
    global task_2_2
    global task_3_1
    global task_3_2
    global last_left
    global last_right
    global search_flag
    global uav_state
    
    with rlock:
        scan_tmp = scan_data.copy()
    # print(scan_data)
    min_id = None
    min_value = 100
    # 获取scan_data中-80到80度之间的数据
    if(len(scan_tmp) == 359):
        if task_flag == 0:
            # print("任务一 开始")
            for i in range(-10, 10):
                # 计算垂直距离
                dis = math.cos(math.pi/180 * abs(i)) * scan_tmp[str(i)]
                if(dis < min_value):
                    min_value = dis
                    min_id = i
            if(min_value >3):
                task_flag = 1
                print("任务一 开始向前通过前排柱子")
            else:
                if search_flag == 0:
                    dis_y = 0
                    for j in range(-100, -90):
                        # 计算垂直距离
                        dis_y = scan_tmp[str(j)] + dis_y
                    if dis_y/10 < 1:
                        search_flag = 1
                    else:
                        cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                        cmd_msg.desired_pos[0] = 0
                        cmd_msg.desired_pos[1] = -0.2
                        cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                        pub_cmd()
                        print("向右搜索")
                
                if search_flag == 1:
                    dis_y = 0
                    for j in range(90, 100):
                        # 计算垂直距离
                        dis_y = scan_tmp[str(j)] + dis_y
                    if dis_y/10 < 1:
                        search_flag = 0
                    else:
                        cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                        cmd_msg.desired_pos[0] = 0
                        cmd_msg.desired_pos[1] = 0.2
                        cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                        pub_cmd()
                        print("向左搜索")


        if task_flag == 1:
            left_dis = scan_tmp[str(85)]
            right_dis = scan_tmp[str(-85)]
            # print(left_dis,right_dis)
            # print(last_left,last_right)
            if(last_left == -1 or last_right == -1):
                last_left = left_dis
                last_right = right_dis
            else:
                
                if((left_dis - last_left) > 1.2 and not task_2_1):
                    task_2_1 = True
                    print("左边柱子通过")
                else:
                    last_left = left_dis
                if((right_dis-last_right) > 1.2 and not task_2_2):
                    task_2_2 = True
                    print("右边边柱子通过")
                else:
                    last_right = right_dis
                    cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                    cmd_msg.desired_pos[0] = 0.2
                    cmd_msg.desired_pos[1] = 0
                    cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                    pub_cmd()
                    print("向前移动")

                if(task_2_1 and task_2_2):
                    task_flag = 2
                    cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                    cmd_msg.desired_pos[0] = 0.6
                    cmd_msg.desired_pos[1] = 0
                    cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                    pub_cmd()
                    time.sleep(5)
                    print("最后一次向前移动")
                    print("任务一结束")
                    print("任务二 开始搜索方框")

        if task_flag == 2:
            # 获取垂直距离数据
            scan_list = []
            for i in range(-80, 80):
                scan_list.append(math.cos(math.pi/180 * abs(i)) * scan_tmp[str(i)])
            np_data = np.array(scan_list)  # 用于绘图
            # 获取谷值  波谷代表可能的障碍物
            valleys, _ = signal.find_peaks(-np.array(scan_list),distance=10)
            
            # 保存到字典方便获取
            scan_dict = {}
            for i in valleys:
                scan_dict[str(i)] = scan_list[i]

            # 获取两个最接近的值
            sorted_keys = sorted(scan_dict, key=scan_dict.get)
            if len(sorted_keys) > 2:
                angle_1 = int(sorted_keys[0])-80
                dis_1 = scan_dict[sorted_keys[0]]
                angle_2 = int(sorted_keys[1])-80
                dis_2 = scan_dict[sorted_keys[1]]

                # 保证angle_1在右边 方便推倒应该向哪边移动
                if angle_1 > angle_2:
                    # 交换值
                    angle_1, angle_2 = angle_2, angle_1
                    dis_1, dis_2 = dis_2, dis_1
                # print("angle1: {}, dis: {}".format(angle_1, dis_1))
                # print("angle1: {}, dis: {}".format(angle_2, dis_2))
                if abs(dis_1 - dis_2) < 0.1:
                    if abs(angle_1) - (angle_2) > 5:
                        cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                        cmd_msg.desired_pos[0] = 0
                        cmd_msg.desired_pos[1] = -0.2
                        cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                        pub_cmd()
                        print("向右移动")
                    elif abs(angle_1) - (angle_2) < -5:
                        cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                        cmd_msg.desired_pos[0] = 0
                        cmd_msg.desired_pos[1] = 0.2
                        cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                        pub_cmd()
                        print("向左移动")
                    else:
                        print("到达方框前方")
                        move_forword = (dis_1 + dis_2) / 2 + 0.6
                        print("向前移动: {}".format(move_forword))
                        cmd_msg.cmd = UAVControlCMD.XYZ_POS_BODY
                        cmd_msg.desired_pos[0] = move_forword
                        cmd_msg.desired_pos[1] = 0
                        cmd_msg.desired_pos[2] = (1.35 - uav_state.position[2])/2
                        pub_cmd()
                        time.sleep(5)
                        if not task_3_1:
                            task_3_1 = True
                        else:
                            task_3_2 = True
                            task_3 = True
                            print("任务二结束")
                            print("降落")
                            cmd_msg.cmd = UAVControlCMD.Land
                            pub_cmd()
                            time.sleep(2)
                            cmd_msg.cmd = UAVControlCMD.Land
                            pub_cmd()
                            task_flag = 3


            
            # plt.plot(np_data)
            # plt.xlabel('Sample')
            # plt.ylabel('Amplitude')

            # # 标记波谷
            # plt.plot(valleys, np_data[valleys], 'o', color='g', markersize=10, label='Valleys')

            # plt.legend()
            # # plt.show()
            # plt.pause(0.001)
            # # 清除图像
            # plt.clf()

                    

def callback(scan):
    global scan_data
    # LaserScan的数据结构
    # std_msgs/Header header
    # float32 angle_min
    # float32 angle_max
    # float32 angle_increment
    # float32 time_increment
    # float32 scan_time
    # float32 range_min
    # float32 range_max
    # float32[] ranges
    # float32[] intensities

    # global x_y_list
    # global global_gird
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment

    # 雷达数据转换为0 - 360的数据字典
    # scan_data = {}
    for i in range(len(scan.ranges)):
        angle = angle_min + i * angle_increment
        # scan_data 使用互斥锁
        with rlock:
            scan_data[str(int(angle/math.pi*180))] = scan.ranges[i]
    # print(scan_data)


    # 根据角度范围和角度增量计算点的数量
    x_list = []
    y_list = []
    num_points = int((angle_max - angle_min) / angle_increment) + 1
    print('range_min: {0}  range_max: {1}'.format(scan.range_min, scan.range_max))
    print('angle_min: {0} angle_max: {1}  angle_increment: {2}  num_points: {3}'.format(angle_min, angle_max, angle_increment, num_points))
    for i in range(0,1000):
        # print('angle: {0}  range: {1}'.format(scan.angle_min + math.pi + i * scan.angle_increment, scan.ranges[i]))
        x = math.cos(scan.angle_min + i * scan.angle_increment) * scan.ranges[i]
        y = math.sin(scan.angle_min + i * scan.angle_increment) * scan.ranges[i]
        # print('x: {0}  y: {1}'.format(x, y))
        x_list.append(x)
        y_list.append(y)

    # # scan.ranges降采样到1/2
    # # scan.ranges = scan.ranges[::2]
    # x_list = np.cos(np.arange(angle_min, angle_max, angle_increment)) * scan.ranges
    # y_list = np.sin(np.arange(angle_min, angle_max, angle_increment)) * scan.ranges
    # x_y_list = np.column_stack((x_list, y_list))
    # # x_max = np.max(x_list)
    # # y_max = np.max(y_list)
    # # print('x_max: {0}  y_max: {1}'.format(x_max, y_max))
    
    # # 地图大小为18x10，范围为(-2.5,2.5) (-4.5,4.5)每个格子0.5x0.5
    # grid = np.zeros((20, 36))
    # # 判断x_y_list中的点位，如果x和y的值在某个grid的范围内，则将对应的格子设置为1
    # for x, y in x_y_list:
    #     if -5 <= y <= 5 and -9 <= x <= 9:
    #         grid[int((y + 5) / 0.5), int((x + 9) / 0.5)] = 1
    # # global_gird = grid

    # # 绘制地图
    # # 反转y轴
    # grid = np.flip(grid, 0)
    # plt.imshow(grid, cmap='gray',extent=(-18, 18, -10, 10))

    plt.plot(x_list, y_list)
    # plt.scatter(x_list, y_list)
    # 设置窗口比例
    # plt.axis('equal')
    
    # plt.show()
    plt.pause(0.001)
    # 清除图像
    plt.clf()

def planner():
    # 将9x5的区域为一个0.5x0.5大小的栅格
    # 0表示没有障碍物，1表示有障碍物
    grid = np.zeros((18, 10))

def state_cb(msg):
    global uav_state
    uav_state = msg
    # print(msg)

def listener():
    global setup_pub
    global uav_state
    global cmd_pub
    global setup_msg
    global cmd_msg

    rospy.Subscriber("scan", LaserScan, callback, queue_size=1)
    rospy.Subscriber("/uav1/sunray/uav_state", UAVState, state_cb, queue_size=1)

    setup_pub = rospy.Publisher("/uav1/sunray/setup", UAVSetup, queue_size=10)
    cmd_pub = rospy.Publisher("/uav1/sunray/uav_control_cmd", UAVControlCMD, queue_size=1)
    # rospy.spin()
    
    time.sleep(2)
    # 解锁
    setup_msg.cmd = 1
    setup_pub.publish(setup_msg)
    print("解锁")
    time.sleep(2)

    # 切换到offboard模式
    setup_msg.cmd = 4
    setup_msg.control_mode = "CMD_CONTROL"
    setup_pub.publish(setup_msg)
    print("切换到offboard模式")
    time.sleep(2)
    setup_pub.publish(setup_msg)
    # 起飞
    cmd_msg.header.stamp = rospy.Time.now()
    cmd_msg.cmd = 1
    cmd_msg.cmd_id = cmd_msg.cmd_id + 1
    cmd_pub.publish(cmd_msg)
    print("起飞")
    time.sleep(2)
    while True:
        if abs(uav_state.position[2] - 1.35) > 0.03:
            cmd_msg.cmd = UAVControlCMD.XYZ_POS
            cmd_msg.desired_pos[0] = uav_state.position[0]
            cmd_msg.desired_pos[1] = uav_state.position[1]
            cmd_msg.desired_pos[2] = 1.38
            cmd_pub.publish(cmd_msg)
            print("起飞中")
        else:
            break
        time.sleep(1)
    while not rospy.is_shutdown():
        time.sleep(1)
        search_column()


if __name__ == "__main__":
    rospy.init_node("lasr_listener", anonymous=False)
    rate = rospy.Rate(20)
    listener()
