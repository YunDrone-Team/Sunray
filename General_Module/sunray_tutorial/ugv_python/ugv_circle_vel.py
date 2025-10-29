#!/usr/bin/env python3
import rospy
from sunray_msgs.msg import UGVControlCMD, UGVState
import math
import signal

class UGVCircleVel:
    def __init__(self):
        # 初始化参数
        self.ugv_id = rospy.get_param('~ugv_id', 1)
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)            # 线速度
        self.circle_radius = rospy.get_param('~circle_radius', 1.0)          # 圆半径
        self.circle_center_x = rospy.get_param('~circle_center_x', 0.0)      # 圆中心x坐标
        self.circle_center_y = rospy.get_param('~circle_center_y', 0.0)      # 圆中心y坐标
        self.radial_gain = rospy.get_param('~radial_gain', 0.5)              # 径向增益
        self.initial_move_completed = False                         # 初始移动完成标志  
        self.returning_to_origin = False                            # 返回原点标志
        self.shutdown_requested = False                             # 关闭请求标志
        self.current_state = None                                   # 当前状态  
        self.INITIAL_MOVE_DISTANCE = 0.5                            # 初始移动距离

        # 发布控制指令和订阅状态
        self.cmd_pub = rospy.Publisher(f"/ugv{self.ugv_id}/sunray_ugv/ugv_control_cmd", UGVControlCMD, queue_size=10)
        self.state_sub = rospy.Subscriber(f"/ugv{self.ugv_id}/sunray_ugv/ugv_state", UGVState, self.state_callback)

        # 注册信号处理函数
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.loginfo(f"UGV Circle Vel Demo initialized for UGV {self.ugv_id}")

    # 状态回调函数
    def state_callback(self, msg):
        self.current_state = msg

    # 信号处理函数
    def signal_handler(self, signum, frame):
        rospy.loginfo(f"Interrupt signal ({signum}) received. Preparing to return to origin...")
        self.shutdown_requested = True

    # 计算两点间距离
    def distance(self, x1, y1, x2, y2):
        dx = x1 - x2
        dy = y1 - y2
        return math.sqrt(dx * dx + dy * dy)

    # 返回原点函数
    def return_to_origin(self):
        self.returning_to_origin = True
        rospy.loginfo("Returning to origin (0,0)...")

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown() and self.shutdown_requested:
            # 检查是否有状态更新
            if self.current_state is None:
                rate.sleep()
                continue

            # 计算当前位置到圆心的距离
            dx = self.current_state.position[0] - self.circle_center_x
            dy = self.current_state.position[1] - self.circle_center_y
            dist = self.distance(self.current_state.position[0], self.current_state.position[1], self.circle_center_x, self.circle_center_y)
            
            #检查是否到达原点
            if dist < 0.1:
                rospy.loginfo("Reached origin (0,0). Stopping...")
                break

            # 检查超时（10秒）
            if (rospy.Time.now() - start_time).to_sec() > 10.0:
                rospy.logwarn("Timeout while returning to origin. Stopping...")
                break

            # 创建并发布速度控制指令
            ugv_cmd = UGVControlCMD()
            ugv_cmd.cmd = UGVControlCMD.POS_CONTROL_ENU
            ugv_cmd.desired_pos[0] = self.circle_center_x
            ugv_cmd.desired_pos[1] = self.circle_center_y
            ugv_cmd.desired_yaw = 0.0

            # 计算速度因子（根据距离调整速度）
            speed_factor = min(1.0, dist / 2.0)
            ugv_cmd.desired_vel[0] = self.linear_speed * speed_factor * (-dx / dist) if dist > 1e-3 else 0.0
            ugv_cmd.desired_vel[1] = self.linear_speed * speed_factor * (-dy / dist) if dist > 1e-3 else 0.0

            self.cmd_pub.publish(ugv_cmd)

            rospy.loginfo_throttle(0.5, f"Distance to origin: {dist:.2f}m")

            rate.sleep()

        # 发布停止指令
        stop_cmd = UGVControlCMD()
        stop_cmd.cmd = UGVControlCMD.HOLD
        self.cmd_pub.publish(stop_cmd)
        rospy.loginfo("UGV stopped at origin.")


    # 生成圆轴运动控制指令
    def generate_commands(self):
        # 检查是否返回原点
        if self.returning_to_origin:
            return
        
        # 创建并发布速度控制指令
        ugv_cmd = UGVControlCMD()
        ugv_cmd.cmd = UGVControlCMD.VEL_CONTROL_ENU

        # 计算当前位置到圆心的距离
        dx = self.current_state.position[0] - self.circle_center_x if self.current_state else 0.0
        dy = self.current_state.position[1] - self.circle_center_y if self.current_state else 0.0
        dist = self.distance(self.current_state.position[0], self.current_state.position[1], self.circle_center_x, self.circle_center_y) if self.current_state else 0.0

        # 处理原点位置或未完成初始移动的情况
        if (dist < 0.05) or not self.initial_move_completed:
            # 初始移动到圆周上
            target_x = self.circle_center_x + self.INITIAL_MOVE_DISTANCE
            target_y = self.circle_center_y

            # 计算到目标点的方向向量
            tx = target_x - self.current_state.position[0] if self.current_state else 0.0
            ty = target_y - self.current_state.position[1] if self.current_state else 0.0
            target_dist = self.distance(self.current_state.position[0], self.current_state.position[1], target_x, target_y) if self.current_state else 0.0
            
            # 归一化向量
            if target_dist > 1e-3:
                tx /= target_dist
                ty /= target_dist

            # 设置速度
            ugv_cmd.desired_vel[0] = self.linear_speed * tx
            ugv_cmd.desired_vel[1] = self.linear_speed * ty

            # 检查是否到达目标点
            if target_dist < 0.1:
                self.initial_move_completed = True
                rospy.loginfo("Initial move completed. Starting circular motion.")
        
        # 圆周运动控制
        else:
            # 计算半径误差
            radius_error = dist - self.circle_radius

            # 计算径向单位向量
            radial_x = dx / dist if dist > 1e-3 else 0.0
            radial_y = dy / dist if dist > 1e-3 else 0.0

            # 计算切向单位向量（逆时针方向）
            tangent_x = -radial_y
            tangent_y = radial_x

            # 计算径向校正速度分量（指向圆心）
            v_radial = -self.radial_gain * radius_error

            # 组合速度向量 = 切向速度 + 径向校正速度（指向圆心）
            vx = self.linear_speed * tangent_x + v_radial * radial_x
            vy = self.linear_speed * tangent_y + v_radial * radial_y

            # 限制最大速度
            speed = math.sqrt(vx * vx + vy * vy)
            if speed > 1.5 * self.linear_speed:
                vx *= (1.5 * self.linear_speed) / speed
                vy *= (1.5 * self.linear_speed) / speed
            
            ugv_cmd.desired_vel[0] = vx
            ugv_cmd.desired_vel[1] = vy

        ugv_cmd.desired_yaw = 0.0
        self.cmd_pub.publish(ugv_cmd)

        # 调试信息
        if self.current_state:
            rospy.loginfo_throttle(1.0, f"Position: ({self.current_state.position[0]:.2f}, {self.current_state.position[1]:.2f}), Distance: {dist:.2f}m, State: {'Circular' if self.initial_move_completed else 'Initial Move'}")

    def run(self):
        # 20 Hz循环频率
        rate = rospy.Rate(20)

        # 等待首次状态更新
        rospy.loginfo("Waiting for initial state...")
        while not rospy.is_shutdown() and self.current_state is None:
            rospy.sleep(0.1)

        # 打印初始位置
        rospy.loginfo(f"Starting UGV circular motion controller. Initial position: ({self.current_state.position[0]:.2f}, {self.current_state.position[1]:.2f})")

        # 检查是否需要在原点开始移动
        dist = self.distance(self.current_state.position[0], self.current_state.position[1], self.circle_center_x, self.circle_center_y)

        # 检查是否在圆心位置，若不在则直接开始圆周运动
        if dist < 0.05:
            rospy.loginfo(f"UGV at center. Performing initial move to ({self.INITIAL_MOVE_DISTANCE:.1f}, 0)")
            self.initial_move_completed = False
        else:
            self.initial_move_completed = True
            rospy.loginfo("Starting circular motion directly")
        
        # 主循环，持续生成指令
        while not rospy.is_shutdown() and not self.shutdown_requested:
            self.generate_commands()
            rate.sleep()

        # 程序退出前返回原点
        if not self.returning_to_origin:
            self.return_to_origin()

        # 发布停止指令，确保小车停止
        stop_cmd = UGVControlCMD()
        stop_cmd.cmd = UGVControlCMD.HOLD
        self.cmd_pub.publish(stop_cmd)
        rospy.loginfo("Program exiting.")

def main():
    rospy.init_node('ugv_circular_motion')
    demo = UGVCircleVel()
    demo.run()

if __name__ == '__main__':
    main()
