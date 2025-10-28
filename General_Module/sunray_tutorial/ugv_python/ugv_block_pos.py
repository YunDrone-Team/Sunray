#!/usr/bin/env python3
import rospy
from sunray_msgs.msg import UGVControlCMD, UGVState
import math

class SquareDemo:
    def __init__(self):
        # 初始化参数
        self.ugv_id = rospy.get_param('~ugv_id', 1)
        self.tolerance = rospy.get_param('~tolerance', 0.1)
        self.max_wait_time = rospy.get_param('~max_wait_time', 30.0)
        self.current_waypoint = 0
        self.first_waypoint_sent = False
        self.state_received = False
        self.last_distance = 1000.0
        self.current_state = None
        self.waypoint_start_time = rospy.Time.now()

        # 定义四边形四个顶点（边长2米）
        self.waypoints = [
            (0.5, 0.0),
            (0.5, 0.5),
            (0.0, 0.5),
            (0.0, 0.0)
        ]

        topic_prefix = f"/ugv{self.ugv_id}/sunray_ugv/ugv_control_cmd"

        # 初始化发布者和订阅者
        self.cmd_pub = rospy.Publisher(topic_prefix, UGVControlCMD, queue_size=10)
        self.state_sub = rospy.Subscriber(f"/ugv{self.ugv_id}/sunray_ugv/ugv_state", UGVState, self.state_callback)

        rospy.loginfo(f"UGV Square Demo initialized for UGV {self.ugv_id}")

    # 状态回调函数
    def state_callback(self, msg):
        self.current_state = msg
        self.state_received = True

    # 计算当前位置与目标位置的距离
    def distance_to_waypoint(self, x, y):
        if self.current_state is None:
            return 1000.0
        dx = x - self.current_state.position[0]
        dy = y - self.current_state.position[1]
        return math.sqrt(dx*dx + dy*dy)

    # 发布目标位置指令
    def publish_waypoint(self):
        # 检查是否还有更多目标点
        if self.current_waypoint >= len(self.waypoints):
            rospy.loginfo("Square trajectory completed.")
            rospy.signal_shutdown("Trajectory finished.")
            return
        
        # 获取当前目标点
        target_x, target_y = self.waypoints[self.current_waypoint]

        # 创建并发布位置控制指令
        ugv_cmd = UGVControlCMD()
        ugv_cmd.cmd = UGVControlCMD.POS_CONTROL_ENU
        ugv_cmd.desired_pos[0] = target_x
        ugv_cmd.desired_pos[1] = target_y
        ugv_cmd.desired_yaw = 0.0

        self.cmd_pub.publish(ugv_cmd)

        # 记录首次发送日志
        if not self.first_waypoint_sent:
            rospy.loginfo(f"Starting square trajectory for UGV {self.ugv_id}")
            self.first_waypoint_sent = True
        
        rospy.loginfo(f"Publishing waypoint {self.current_waypoint+1}: ({target_x:.1f}, {target_y:.1f})")
        
        # 记录发送时间
        self.waypoint_start_time = rospy.Time.now()

        #重置距离
        self.last_distance = self.distance_to_waypoint(target_x, target_y)

    def run(self):
        #10 Hz循环频率
        rate = rospy.Rate(10)

        # 等待首次状态更新
        rospy.loginfo("Waiting for first state update...")
        while not rospy.is_shutdown() and not self.state_received:
            rospy.sleep(0.1)

        rospy.loginfo("First state received. Starting trajectory.")

    #==================================== 轨迹控制关键代码段 BEGIN（二次开发） ====================================
        # 发送第一个目标点
        self.publish_waypoint()
        
        while not rospy.is_shutdown() and self.current_waypoint < len(self.waypoints):
            # 检查状态更新
            if not self.state_received:
                rospy.logwarn_throttle(1.0, "Waiting for state update...")
                rate.sleep()
                continue

            # 计算当前距离
            target_x, target_y = self.waypoints[self.current_waypoint]
            current_distance = self.distance_to_waypoint(target_x, target_y)
            self.last_distance = current_distance

            # 检查是否到达目标点
            if current_distance < self.tolerance:
                # 到达目标点，发送下一个点
                rospy.loginfo(f"Reached waypoint {self.current_waypoint+1}: ({target_x:.1f}, {target_y:.1f})")
                self.current_waypoint += 1

                # 检查是否还有更多目标点
                if self.current_waypoint < len(self.waypoints):
                    self.publish_waypoint()
                else:
                    rospy.loginfo("Trajectory completed!")
                    break

            # 检查是否正在接近目标点
            else:
                # 输出当前距离
                rospy.loginfo_throttle(1.0, f"Distance to waypoint {self.current_waypoint+1}: {current_distance:.2f} m")

                # 检查是否超时
                elapsed = (rospy.Time.now() - self.waypoint_start_time).to_sec()
                if elapsed > self.max_wait_time:
                    # 超时，跳过当前点
                    rospy.logwarn(f"Timeout at waypoint {self.current_waypoint+1} ({elapsed:.1f} s). Moving to next point.")
                    self.current_waypoint += 1

                    # 检查是否还有更多目标点
                    if self.current_waypoint < len(self.waypoints):
                        self.publish_waypoint()
                    else:
                        rospy.loginfo("Trajectory completed (with timeout).")
                        break
            rate.sleep()
    #==================================== 轨迹控制关键代码段 END（二次开发） ======================================

        # 发送停止指令
        stop_cmd = UGVControlCMD()
        stop_cmd.cmd = UGVControlCMD.HOLD
        self.cmd_pub.publish(stop_cmd)
        rospy.loginfo("Trajectory finished. Sending HOLD command.")

        # 等待1秒，确保指令生效
        rospy.sleep(1.0)

def main():
    rospy.init_node('ugv_square_demo')
    demo = SquareDemo()
    demo.run()

if __name__ == '__main__':
    main()
