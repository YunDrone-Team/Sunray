import sys
import time
import pygame
import cv2
import numpy as np
import rospy

from _UAVControlCMD import UAVControlCMD
from _UAVSetup import UAVSetup
from _UAVState import UAVState

class UAVControl:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((320, 240))
        pygame.display.set_caption("keyboard ctrl")
        self.screen.fill((0, 0, 0))

        self.vehicle_velocity = 0.5
        self.speedup_ratio = 2
        self.speedup_flag = False

        self.vehicle_yaw_rate = 60
        self.K_t_pressed = False
        self.stop_flag = False
        self.last_time = None
        self.k_dict = {}

        self.setup_msg = UAVSetup()
        self.cmd_msg = UAVControlCMD()

    def pub_cmd(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.cmd_id = self.cmd_msg.cmd_id + 1
        self.cmd_pub.publish(self.cmd_msg)

    def keyboard_control(self):
        # print(rospy.is_shutdown())
        while True and not rospy.is_shutdown():
            yaw_rate = 0.0
            velocity_x = 0.0
            velocity_y = 0.0
            velocity_z = 0.0

            time.sleep(0.02)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()

            scan_wrapper = pygame.key.get_pressed()

            if scan_wrapper[pygame.K_SPACE]:
                scale_ratio = self.speedup_ratio
            else:
                scale_ratio = self.vehicle_velocity

            if scan_wrapper[pygame.K_q] or scan_wrapper[pygame.K_e]:
                yaw_rate = (
                    (scan_wrapper[pygame.K_q] - scan_wrapper[pygame.K_e])
                    * scale_ratio
                    * self.vehicle_yaw_rate
                )

            if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
                velocity_x = (scan_wrapper[pygame.K_w] - scan_wrapper[pygame.K_s]) * scale_ratio

            if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
                velocity_y = (
                    -(scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio
                )

            if scan_wrapper[pygame.K_z] or scan_wrapper[pygame.K_x]:
                velocity_z = (
                    -(scan_wrapper[pygame.K_x] - scan_wrapper[pygame.K_z]) * scale_ratio
                )

            if scan_wrapper[pygame.K_t]:
                if "t" in self.k_dict:
                    if time.time() - self.k_dict["t"] > 1:
                        self.setup_msg.cmd = UAVSetup.ARM
                        self.setup_pub.publish(self.setup_msg)

                        self.k_dict.pop("t", None)
                        print("解锁")
                else:
                    self.k_dict["t"] = time.time()
            else:
                self.k_dict.pop("t", None)

            if scan_wrapper[pygame.K_u]:
                if "u" in self.k_dict:
                    if time.time() - self.k_dict["u"] > 0.5:
                        self.setup_msg.cmd = UAVSetup.DISARM
                        self.setup_pub.publish(self.setup_msg)

                        self.k_dict.pop("u", None)
                        print("上锁")
                else:
                    self.k_dict["u"] = time.time()
            else:
                self.k_dict.pop("u", None)

            if scan_wrapper[pygame.K_y] and (scan_wrapper[pygame.K_t] or scan_wrapper[pygame.K_u]):
                self.setup_msg.cmd = UAVSetup.EMERGENCY_KILL
                self.setup_pub.publish(self.setup_msg)
                print("KILL")

            if scan_wrapper[pygame.K_g]:
                if "g" in self.k_dict:
                    if time.time() - self.k_dict["g"] > 0.5:
                        self.cmd_msg.cmd = UAVControlCMD.Takeoff
                        self.pub_cmd()

                        self.k_dict.pop("g", None)
                        print("Takeoff")
                else:
                    self.k_dict["g"] = time.time()
            else:
                self.k_dict.pop("g", None)

            if scan_wrapper[pygame.K_h]:
                if "h" in self.k_dict:
                    if time.time() - self.k_dict["h"] > 0.5:
                        self.cmd_msg.cmd = UAVControlCMD.Land
                        self.cmd_msg.header.stamp = rospy.Time.now()
                        self.cmd_msg.cmd_id = self.cmd_msg.cmd_id + 1
                        self.pub_cmd()

                        self.k_dict.pop("h", None)
                        print("Land")
                else:
                    self.k_dict["h"] = time.time()
            else:
                self.k_dict.pop("h", None)

            if scan_wrapper[pygame.K_o]:
                if "o" in self.k_dict:
                    if time.time() - self.k_dict["o"] > 1:
                        self.setup_msg.cmd = 4
                        self.setup_msg.control_mode = "CMD_CONTROL"
                        self.setup_pub.publish(self.setup_msg)

                        self.k_dict.pop("o", None)
                        print("Switch to offboard")
                else:
                    self.k_dict["o"] = time.time()
            else:
                self.k_dict.pop("o", None)

            if velocity_x == 0 and velocity_y == 0 and velocity_z == 0 and yaw_rate == 0:
                if self.stop_flag and time.time() - self.stop_flag > 0.5:
                    self.cmd_msg.cmd = UAVControlCMD.Hover
                else:
                    self.stop_flag = True
                    if not self.stop_flag:
                        self.stop_flag = time.time()

                self.pub_cmd()
            else:
                self.stop_flag = False
                self.cmd_msg.cmd = UAVControlCMD.XYZ_VEL_BODY
                self.cmd_msg.desired_vel[0] = velocity_x
                self.cmd_msg.desired_vel[1] = velocity_y
                self.cmd_msg.desired_vel[2] = velocity_z
                self.cmd_msg.desired_yaw_rate = yaw_rate/180.0*np.pi
                self.cmd_msg.enable_yawRate = True
                self.pub_cmd()
                print("vx:{} vy:{} vz:{} yaw:{}".format(velocity_x, velocity_y, velocity_z, yaw_rate))

            if scan_wrapper[pygame.K_ESCAPE]:
                pygame.quit()
                sys.exit()

    def listener(self):
        self.setup_pub = rospy.Publisher("/uav1/sunray/setup", UAVSetup, queue_size=10)
        self.cmd_pub = rospy.Publisher("/uav1/sunray/uav_control_cmd", UAVControlCMD, queue_size=1)
        
if __name__ == "__main__":
    rospy.init_node("key_control_node", anonymous=False)
    rate = rospy.Rate(20)
    key_control = UAVControl()
    key_control.listener()
    time.sleep(2)
    key_control.keyboard_control()
    
