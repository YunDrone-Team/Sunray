#!/usr/bin/env python3
import rospy
from sunray_msgs.msg import UGVControlCMD
import math

class CircularBodyControl:
    def __init__(self):
        # 初始化参数
        self.ugv_id = rospy.get_param('~ugv_id', 1)
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)      # 线速度
        self.circle_radius = rospy.get_param('~circle_radius', 0.5)    # 圆半径
        self.angular_vel = self.linear_speed / self.circle_radius      # 角速度
        self.motion_completed = False                                  # 运动完成标志
        self.start_time = rospy.Time.now()                             # 开始时间
        
        # 计算完成两圈所需时间：周长=2*π*半径，两圈时间=2*周长/线速度
        self.target_duration = (4 * math.pi * self.circle_radius) / self.linear_speed  

        # 发布控制指令
        cmd_topic = f"/ugv{self.ugv_id}/sunray_ugv/ugv_control_cmd" 
        self.cmd_pub = rospy.Publisher(cmd_topic, UGVControlCMD, queue_size=10)
        rospy.loginfo(f"UGV Circular Body Demo initialized for UGV {self.ugv_id}")

    #==================================== 轨迹控制关键代码段 BEGIN（二次开发） ====================================
    # 生成控制指令
    def generate_commands(self):
        # 如果运动已完成，则不再发送指令
        if self.motion_completed:
            return
         
        ugv_cmd = UGVControlCMD()
        ugv_cmd.cmd = UGVControlCMD.VEL_CONTROL_BODY

        # 计算已运行的时间
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()

        # 如果未完成目标时间，则继续发送速度指令
        if elapsed_time < self.target_duration + 0.5:
            ugv_cmd.desired_vel[0] = self.linear_speed
            ugv_cmd.desired_vel[1] = 0.0
            ugv_cmd.angular_vel = self.angular_vel
            self.cmd_pub.publish(ugv_cmd)
        
        # 运动完成，发送停止指令并返回原点
        else:
            ugv_cmd.desired_vel[0] = 0.0
            ugv_cmd.desired_vel[1] = 0.0
            ugv_cmd.angular_vel = 0.0
            self.cmd_pub.publish(ugv_cmd)
            self.motion_completed = True
            ugv_cmd.cmd = UGVControlCMD.POS_CONTROL_ENU
            ugv_cmd.desired_pos[0] = 0.0
            ugv_cmd.desired_pos[1] = 0.0
            self.cmd_pub.publish(ugv_cmd)
            rospy.loginfo("Motion completed. Shutting down...")
            rospy.signal_shutdown("Trajectory finished.")
        
        # 打印当前速度指令
        rospy.loginfo_throttle(1.0, f"Publishing body vel: [{ugv_cmd.desired_vel[0]:.2f}, {ugv_cmd.desired_vel[1]:.2f}] m/s | ang: {ugv_cmd.angular_vel:.2f} rad/s")
    #==================================== 轨迹控制关键代码段 END（二次开发） ======================================
    
    # 主循环，持续生成指令直到运动完成或节点被关闭
    def run(self):
        # 20 Hz循环频率
        rate = rospy.Rate(20)

        # 主循环，持续生成指令直到运动完成或节点被关闭
        while not rospy.is_shutdown() and not self.motion_completed:
            self.generate_commands()
            rate.sleep()

def main():
    rospy.init_node('ugv_circular_body_demo')
    controller = CircularBodyControl()
    controller.run()

if __name__ == '__main__':
    main()
