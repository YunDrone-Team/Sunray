import rospy
import numpy as np
import torch
from map_manager.srv import RayCast
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Quaternion, Vector3
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from navigation_runner.srv import GetSafeAction, GetSafeActionMap
from onboard_detector.srv import GetDynamicObstacles
from map_manager.srv import GetStaticObstacles
from ppo import PPO
from torchrl.data import CompositeSpec, UnboundedContinuousTensorSpec
from tensordict.tensordict import TensorDict
from torchrl.envs.utils import ExplorationType, set_exploration_type
from navigation_runner.srv import GetPolicyInference
from utils import vec_to_new_frame
import math
from std_srvs.srv import Empty
import tf.transformations
import time
import threading
import os

class Navigation:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lidar_hbeams = int(360/self.cfg.sensor.lidar_hres)
        self.raypoints = []
        self.dynamic_obstacles = self.make_empty_dynamic_obstacles()
        self.robot_size = 0.3 # radius
        self.raycast_vres = ((self.cfg.sensor.lidar_vfov[1] - self.cfg.sensor.lidar_vfov[0]))/(self.cfg.sensor.lidar_vbeams - 1) * np.pi/180.0
        self.raycast_hres = self.cfg.sensor.lidar_hres * np.pi/180.0

        self.goal = None
        self.goal_received = False
        self.target_dir = None
        self.stable_times = 0
        self.has_action = False
        self.goal_hold_latched = False
        self.laser_points_msg = None

        self.height_control = False 
        self.px4_control = rospy.get_param('rl/use_px4', True)

        self.use_policy_server = False

        self.odom_received = False
        if (self.px4_control):
            # PX4/MAVROS 控制模式：
            # 1. 从 MAVROS local_position/odom 获取无人机里程计。
            # 2. 从 MAVROS state 获取当前飞控模式和解锁状态。
            # 3. 将 NAVRL 生成的速度/位置指令直接发布给 MAVROS setpoint 接口。
            self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
            self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
            self.action_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
            self.pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

            # PX4 OFFBOARD 模式切换和解锁服务。
            self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
            self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

            self.mavros_state = None
            self.offb_set_mode = SetModeRequest()
            self.offb_set_mode.custom_mode = 'OFFBOARD'
            self.arm_cmd = CommandBoolRequest()
            self.arm_cmd.value = True
        else:
            # 非 PX4 控制模式：
            # 1. 使用 NAVRL 原始示例的 /CERLAB/quadcopter/odom 作为里程计输入。
            # 2. 将速度指令发布到 /CERLAB/quadcopter/cmd_vel。
            #    sunray_navrl_adapter/NavRL2Sunray_node 会订阅该话题，并转换成 Sunray UAVControlCMD。
            # 3. setpoint_pose 用于 NAVRL 内部起飞/目标位姿相关逻辑。
            # 以下odom三选一均可
            self.odom_sub1 = rospy.Subscriber("/uav1/sunray_sim/odom", Odometry, self.odom_callback)
            self.odom_sub2 = rospy.Subscriber("/uav1/sunray/localization/local_odom", Odometry, self.odom_callback)
            self.odom_sub3 = rospy.Subscriber("/ugv1/sunray_sim/odom", Odometry, self.odom_callback)
            self.odom_sub = rospy.Subscriber("/CERLAB/quadcopter/odom", Odometry, self.odom_callback)
            self.action_pub = rospy.Publisher("/CERLAB/quadcopter/cmd_vel", TwistStamped, queue_size=10)
            self.pose_pub = rospy.Publisher("/CERLAB/quadcopter/setpoint_pose", PoseStamped, queue_size=10)

        # RViz 2D Nav Goal 输入的导航目标点。
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # 以下话题主要用于 RViz 调试显示：
        # raycast：策略输入使用的射线采样点。
        # cmd：RL 原始速度和安全过滤后速度。
        # goal：当前目标点。
        # rollout_traj：策略推理过程中的预测轨迹。
        # in_range_dynamic_obstacles：进入感知范围的动态障碍物。
        self.raycast_vis_pub = rospy.Publisher("/rl_navigation/raycast", MarkerArray, queue_size=10)
        self.cmd_vis_pub = rospy.Publisher("/rl_navigation/cmd", MarkerArray, queue_size=10)
        self.goal_vis_pub = rospy.Publisher("rl_navigation/goal", MarkerArray, queue_size=10)
        self.rollout_traj_pub = rospy.Publisher("/rollout_traj", Path, queue_size=10)
        self.dynamic_obstacle_vis_pub = rospy.Publisher("/rl_navigation/in_range_dynamic_obstacles", MarkerArray, queue_size=10)

        if (not self.use_policy_server):
            self.policy = self.init_model()
            self.policy.eval()


        # safety thread
        self.safety_stop = False
        safety_thread = threading.Thread(target = self.safety_check)
        safety_thread.start()

        self.takeoff()
  
    def make_empty_dynamic_obstacles(self):
        dynamic_obstacle_num = self.cfg.algo.feature_extractor.dyn_obs_num
        dynamic_obstacle_pos = torch.zeros(dynamic_obstacle_num, 3, dtype=torch.float, device=self.cfg.device)
        dynamic_obstacle_vel = torch.zeros(dynamic_obstacle_num, 3, dtype=torch.float, device=self.cfg.device)
        dynamic_obstacle_size = torch.zeros(dynamic_obstacle_num, 3, dtype=torch.float, device=self.cfg.device)
        return (dynamic_obstacle_pos, dynamic_obstacle_vel, dynamic_obstacle_size)

    def init_model(self):
        observation_dim = 8
        num_dim_each_dyn_obs_state = 10
        observation_spec = CompositeSpec({
            "agents": CompositeSpec({
                "observation": CompositeSpec({
                    "state": UnboundedContinuousTensorSpec((observation_dim,), device=self.cfg.device), 
                    "lidar": UnboundedContinuousTensorSpec((1, self.lidar_hbeams, self.cfg.sensor.lidar_vbeams), device=self.cfg.device),
                    "direction": UnboundedContinuousTensorSpec((1, 3), device=self.cfg.device),
                    "dynamic_obstacle": UnboundedContinuousTensorSpec((1, self.cfg.algo.feature_extractor.dyn_obs_num, num_dim_each_dyn_obs_state), device=self.cfg.device),
                }),
            }).expand(1)
        }, shape=[1], device=self.cfg.device)

        action_dim = 3
        action_spec = CompositeSpec({
            "agents": CompositeSpec({
                "action": UnboundedContinuousTensorSpec((action_dim,), device=self.cfg.device), 
            })
        }).expand(1, action_dim).to(self.cfg.device)

        policy = PPO(self.cfg.algo, observation_spec, action_spec, self.cfg.device)

        file_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ckpts")
        checkpoint = "navrl_checkpoint.pt"

        policy.load_state_dict(torch.load(os.path.join(file_dir, checkpoint), map_location=self.cfg.device))
        return policy

    def takeoff(self):
        takeoff_height = 1.0
        r = rospy.Rate(10)
        while (not rospy.is_shutdown() and self.odom_received == False):
            print("[nav-ros]: Wait for robot odom...")
            r.sleep()

        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position.x = self.odom.pose.pose.position.x
        takeoff_pose.pose.position.y = self.odom.pose.pose.position.y
        takeoff_pose.pose.position.z = takeoff_height
        takeoff_pose.pose.orientation = self.odom.pose.pose.orientation
        self.takeoff_pose = takeoff_pose
        if (self.px4_control):
            pose = PoseStamped()

            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2
            rate = rospy.Rate(20)
            # Send a few setpoints before starting
            for i in range(100):
                if(rospy.is_shutdown()):
                    break
                self.pose_pub.publish(pose)
                rate.sleep()
            last_req = rospy.Time.now()
        while (not rospy.is_shutdown() and not (np.abs(self.odom.pose.pose.position.z - takeoff_height) <= 2.0)):
            if (self.px4_control):
                if (self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                        print("[nav-ros]: OFFBOARD enabled.")

                    last_req = rospy.Time.now()
                else:
                    if(not self.mavros_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                        if(self.arming_client.call(self.arm_cmd).success == True):
                            print("[nav-ros]: Vehicle armed.")

                        last_req = rospy.Time.now()
            self.pose_pub.publish(takeoff_pose)
            r.sleep()
        print("[nav-ros]: take off completed at height: ", takeoff_height)


    def safety_check(self):
        while not rospy.is_shutdown():
            if (self.safety_stop == False):
                # 终端人工暂停开关：程序启动后该线程会阻塞等待用户按 Enter。
                # 第一次按 Enter 时进入 safety_stop 状态，不是由避障/控制器自动触发。
                input("[nav-ros]: Press Enter to STOP motion!\n")
                self.safety_stop = True

                # stop_pose 记录“按下停止时刻”的当前里程计位姿：
                # position 为当前无人机位置，orientation 为当前无人机姿态。
                # control_callback 在 safety_stop=True 时会持续发布该位姿，
                # 使原始 NAVRL 示例进入当前位置悬停/保持。
                self.stop_pose = PoseStamped()
                self.stop_pose.pose = self.odom.pose.pose
            else:
                # 再次按 Enter 退出人工暂停，恢复后续目标点/速度控制逻辑。
                input("[nav-ros]: Press Enter to CONTINUE motion!\n")
                self.safety_stop = False

    def get_raycast(self, pos: np.array , start_angle: float):
        raypoints = []
        try:
            raycast = rospy.ServiceProxy("occupancy_map/raycast", RayCast)
            pos_msg = Point()
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]
            response = raycast(pos_msg,
                               start_angle,
                               self.cfg.sensor.lidar_range, 
                               self.cfg.sensor.lidar_vfov[0], 
                               self.cfg.sensor.lidar_vfov[1], 
                               self.cfg.sensor.lidar_vbeams, 
                               self.cfg.sensor.lidar_hres
                               )
            num_points = int(len(response.points)/3)
            self.laser_points_msg = response.points
            for i in range(num_points):
                p = [response.points[3*i+0], response.points[3*i+1], response.points[3*i+2]]
                raypoints.append(p)
        except rospy.service.ServiceException as e:
            print("[nav-ros]: raycast func err!")        
        return raypoints

    def get_dynamic_obstacles(self, pos: np.array):
        dynamic_obstacle_pos = torch.zeros(self.cfg.algo.feature_extractor.dyn_obs_num, 3, dtype=torch.float, device=self.cfg.device)
        dynamic_obstacle_vel = torch.zeros(self.cfg.algo.feature_extractor.dyn_obs_num, 3, dtype=torch.float, device=self.cfg.device)
        dynamic_obstacle_size = torch.zeros(self.cfg.algo.feature_extractor.dyn_obs_num, 3, dtype=torch.float, device=self.cfg.device)
        try:
            distance_range = 4.0
            pos_msg = Point()
            pos_msg.x = pos[0]
            pos_msg.y = pos[1]
            pos_msg.z = pos[2]
            get_obstacle = rospy.ServiceProxy("onboard_detector/get_dynamic_obstacles", GetDynamicObstacles)
            response = get_obstacle(pos_msg, distance_range)
            total_obs_num = len(response.position)
            for i in range(self.cfg.algo.feature_extractor.dyn_obs_num):
                if (i < total_obs_num):
                    pos_vec = response.position[i]
                    vel_vec = response.velocity[i]
                    size_vec = response.size[i]
                    dynamic_obstacle_pos[i] = torch.tensor([pos_vec.x, pos_vec.y, pos_vec.z], dtype=torch.float, device=self.cfg.device)
                    dynamic_obstacle_vel[i] = torch.tensor([vel_vec.x, vel_vec.y, vel_vec.z], dtype=torch.float, device=self.cfg.device)
                    dynamic_obstacle_size[i] = torch.tensor([size_vec.x, size_vec.y, size_vec.z], dtype=torch.float, device=self.cfg.device)
        except rospy.service.ServiceException as e:
            print("[nav-ros]: dynamic obstacle func err!")   
        return dynamic_obstacle_pos, dynamic_obstacle_vel, dynamic_obstacle_size

    def get_static_obstacles(self):
        static_obstacle_pos = []
        static_obstacle_size = []
        static_obstacle_angle = []
        try:
            get_static_obstacles_server = rospy.ServiceProxy("occupancy_map/get_static_obstacles", GetStaticObstacles)
            static_obstacle_response = get_static_obstacles_server()
            static_obstacle_pos = static_obstacle_response.position
            static_obstacle_size = static_obstacle_response.size
            static_obstacle_angle = static_obstacle_response.angle
        except rospy.service.ServiceException as e:
            print("[nav-ros]: static obstacle func err!")
        return static_obstacle_pos, static_obstacle_size, static_obstacle_angle
           
    def raycast_callback(self, event):
        if not self.odom_received or not self.goal_received:
            return
        pos = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
        self.update_target_dir()
        start_angle = np.arctan2(self.target_dir[1].cpu().numpy(), self.target_dir[0].cpu().numpy())
        self.raypoints = self.get_raycast(pos, start_angle)

    def dynamic_obstacle_callback(self, event):
        if not self.odom_received:
            return
        pos = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
        dynamic_obstacle_pos, dynamic_obstacle_vel, dynamic_obstacle_size = self.get_dynamic_obstacles(pos)
        self.dynamic_obstacles = (dynamic_obstacle_pos, dynamic_obstacle_vel, dynamic_obstacle_size)

    def odom_callback(self, odom):
        self.odom = odom
        self.odom_received = True
    
    def state_callback(self, state):
        self.mavros_state = state
    
    def goal_callback(self, goal):
        if not self.odom_received:
            return

        self.goal = goal
        self.goal.pose.position.z = self.takeoff_pose.pose.position.z
        self.update_target_dir()

        self.goal_received = True
        self.stable_times = 0
        self.goal_hold_latched = False
        rospy.loginfo(
            "[nav-ros]: received goal: pos=(%.2f, %.2f, %.2f), target_dir=(%.2f, %.2f, %.2f)",
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            self.goal.pose.position.z,
            self.target_dir[0].item(),
            self.target_dir[1].item(),
            self.target_dir[2].item(),
        )

    def update_target_dir(self):
        if self.goal is None or not self.odom_received:
            return
        dir_x = self.goal.pose.position.x - self.odom.pose.pose.position.x
        dir_y = self.goal.pose.position.y - self.odom.pose.pose.position.y
        dir_z = self.goal.pose.position.z - self.odom.pose.pose.position.z
        self.target_dir = torch.tensor([dir_x, dir_y, dir_z], device=self.cfg.device)

    def quaternion_to_rotation_matrix(self, quaternion):
        # w, x, y, z = quaternion
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        xx, xy, xz = x**2, x*y, x*z
        yy, yz = y**2, y*z
        zz = z**2
        wx, wy, wz = w*x, w*y, w*z
        
        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
        ])        
    
    def check_obstacle(self, lidar_scan, dyn_obs_states):
        # return true if there is obstacles in the range
        # has_static = not torch.all(lidar_scan == 0.)
        # has_static = not torch.all(lidar_scan[..., 1:] < 0.2) # hardcode to tune
        quarter_size = lidar_scan.shape[2] // 4
        first_quarter_check, last_quarter_check = torch.all(lidar_scan[:, :, :quarter_size, 1:] < 0.2), torch.all(lidar_scan[:, :, -quarter_size:, 1:] < 0.2)
        has_static = (not first_quarter_check) or (not last_quarter_check)
        has_dynamic = not torch.all(dyn_obs_states == 0.)
        return has_static or has_dynamic

    def get_safe_action(self, vel_world, action_vel_world):
        safe_action = np.zeros(3)
        try:
            pos_msg = Point(x=self.odom.pose.pose.position.x, y=self.odom.pose.pose.position.y, z=self.odom.pose.pose.position.z)
            get_safe_action = rospy.ServiceProxy("rl_navigation/get_safe_action", GetSafeAction)
            vel_msg = Vector3(x=vel_world[0].item(), y=vel_world[1].item(), z=vel_world[2].item())
            action_vel_msg = Vector3(x=action_vel_world[0], y=action_vel_world[1], z=action_vel_world[2])
            max_vel = np.sqrt(3. * self.cfg.algo.actor.action_limit**2)
            obstacle_pos_list = []
            obstacle_vel_list = []
            obstacle_size_list = []
            for i in range(len(self.dynamic_obstacles[0])):
                if (self.dynamic_obstacles[2][i][0] != 0):
                    obs_pos = Vector3(x=self.dynamic_obstacles[0][i][0].item(), y=self.dynamic_obstacles[0][i][1].item(), z=self.dynamic_obstacles[0][i][2].item())
                    obs_vel = Vector3(x=self.dynamic_obstacles[1][i][0].item(), y=self.dynamic_obstacles[1][i][1].item(), z=self.dynamic_obstacles[1][i][2].item())
                    obs_size = Vector3(x=self.dynamic_obstacles[2][i][0].item(), y=self.dynamic_obstacles[2][i][1].item(), z=self.dynamic_obstacles[2][i][2].item())
                    obstacle_pos_list.append(obs_pos)
                    obstacle_vel_list.append(obs_vel)
                    obstacle_size_list.append(obs_size)
            response = get_safe_action(pos_msg, vel_msg, self.robot_size, obstacle_pos_list, obstacle_vel_list,\
                                    obstacle_size_list, self.laser_points_msg, self.cfg.sensor.lidar_range,\
                                    max(self.raycast_vres, self.raycast_hres), max_vel, action_vel_msg)
            safe_action = np.array([response.safe_action.x, response.safe_action.y, response.safe_action.z])
            return safe_action
        except rospy.service.ServiceException as e:
            # print("[nav-ros]: no safety running!")
            return action_vel_world   

    def get_safe_action_map(self, vel_world, action_vel_world):
        safe_action = np.zeros(3)
        try:
            pos_msg = Point(x=self.odom.pose.pose.position.x, y=self.odom.pose.pose.position.y, z=self.odom.pose.pose.position.z)
            get_safe_action = rospy.ServiceProxy("rl_navigation/get_safe_action_map", GetSafeActionMap)
            vel_msg = Vector3(x=vel_world[0].item(), y=vel_world[1].item(), z=vel_world[2].item())
            action_vel_msg = Vector3(x=action_vel_world[0], y=action_vel_world[1], z=action_vel_world[2])
            max_vel = np.sqrt(3. * self.cfg.algo.actor.action_limit**2)
            
            # Dynamic Obstacles
            obstacle_pos_list = []
            obstacle_vel_list = []
            obstacle_size_list = []
            for i in range(len(self.dynamic_obstacles[0])):
                if (self.dynamic_obstacles[2][i][0] != 0):
                    obs_pos = Vector3(x=self.dynamic_obstacles[0][i][0].item(), y=self.dynamic_obstacles[0][i][1].item(), z=self.dynamic_obstacles[0][i][2].item())
                    obs_vel = Vector3(x=self.dynamic_obstacles[1][i][0].item(), y=self.dynamic_obstacles[1][i][1].item(), z=self.dynamic_obstacles[1][i][2].item())
                    obs_size = Vector3(x=self.dynamic_obstacles[2][i][0].item(), y=self.dynamic_obstacles[2][i][1].item(), z=self.dynamic_obstacles[2][i][2].item())
                    obstacle_pos_list.append(obs_pos)
                    obstacle_vel_list.append(obs_vel)
                    obstacle_size_list.append(obs_size)
            
            # Static Obstacles
            static_obstacle_pos, static_obstacle_size, static_obstacle_angle = self.get_static_obstacles()

            response = get_safe_action(pos_msg, vel_msg, self.robot_size, obstacle_pos_list, obstacle_vel_list,\
                                    obstacle_size_list, static_obstacle_pos, static_obstacle_size,\
                                    static_obstacle_angle, max_vel, action_vel_msg)
            safe_action = np.array([response.safe_action.x, response.safe_action.y, response.safe_action.z])
            return safe_action
        except rospy.service.ServiceException as e:
            # print("[nav-ros]: no safety running!")
            return action_vel_world   
    
    def get_action(self, pos: torch.Tensor, vel: torch.Tensor, goal: torch.Tensor): # use world velocity
        rpos = goal - pos
        distance = rpos.norm(dim=-1, keepdim=True)
        distance_2d = rpos[..., :2].norm(dim=-1, keepdim=True)
        distance_z = rpos[..., 2].unsqueeze(-1)


        target_dir_2d = self.target_dir.clone()
        target_dir_2d[2] = 0.

        rpos_clipped = rpos / distance.clamp(1e-6) # start to goal direction
        rpos_clipped_g = vec_to_new_frame(rpos_clipped, target_dir_2d).squeeze(0).squeeze(0)

        # "relative" velocity
        vel_g = vec_to_new_frame(vel, target_dir_2d).squeeze(0).squeeze(0) # goal velocity

        # drone_state = torch.cat([rpos_clipped, orientation, vel_g], dim=-1).squeeze(1)
        drone_state = torch.cat([rpos_clipped_g, distance_2d, distance_z, vel_g], dim=-1).unsqueeze(0)

        # Lidar States
        lidar_scan = torch.tensor(self.raypoints, device=self.cfg.device)
        lidar_scan = (lidar_scan - pos).norm(dim=-1).clamp_max(self.cfg.sensor.lidar_range).reshape(1, 1, self.lidar_hbeams, self.cfg.sensor.lidar_vbeams)
        lidar_scan = self.cfg.sensor.lidar_range - lidar_scan


        # dynamic obstacle states
        dynamic_obstacle_pos = self.dynamic_obstacles[0].clone()
        dynamic_obstacle_vel = self.dynamic_obstacles[1].clone()
        dynamic_obstacle_size = self.dynamic_obstacles[2].clone()
        closest_dyn_obs_rpos = dynamic_obstacle_pos - pos
        closest_dyn_obs_rpos[dynamic_obstacle_size[:, 2] == 0] = 0.
        closest_dyn_obs_rpos[:, 2][dynamic_obstacle_size[:, 2] > 1] = 0.
        closest_dyn_obs_rpos_g = vec_to_new_frame(closest_dyn_obs_rpos.unsqueeze(0), target_dir_2d).squeeze(0)
        closest_dyn_obs_distance = closest_dyn_obs_rpos.norm(dim=-1, keepdim=True)
        closest_dyn_obs_distance_2d = closest_dyn_obs_rpos_g[..., :2].norm(dim=-1, keepdim=True)
        closest_dyn_obs_distance_z = closest_dyn_obs_rpos_g[..., 2].unsqueeze(-1)
        closest_dyn_obs_rpos_gn = closest_dyn_obs_rpos_g / closest_dyn_obs_distance.clamp(1e-6)



        closest_dyn_obs_vel_g = vec_to_new_frame(dynamic_obstacle_vel.unsqueeze(0), target_dir_2d).squeeze(0)
        
        obs_res = 0.25
        closest_dyn_obs_width = torch.max(dynamic_obstacle_size[:, 0], dynamic_obstacle_size[:, 1])
        closest_dyn_obs_width += self.robot_size * 2.
        closest_dyn_obs_width = torch.clamp(torch.ceil(closest_dyn_obs_width / 0.25) - 1, min=0, max=1./obs_res - 1)
        closest_dyn_obs_width[dynamic_obstacle_size[:, 2] == 0] = 0.
        closest_dyn_obs_height = dynamic_obstacle_size[:, 2]
        closest_dyn_obs_height[(closest_dyn_obs_height <= 1) & (closest_dyn_obs_height != 0)] = 1.
        closest_dyn_obs_height[closest_dyn_obs_height > 1] = 0.
        # dyn_obs_states = torch.cat([closest_dyn_obs_rpos_g, closest_dyn_obs_vel_g, \
        #                             closest_dyn_obs_width.unsqueeze(1), closest_dyn_obs_height.unsqueeze(1)], dim=-1).unsqueeze(0).unsqueeze(0)
        dyn_obs_states = torch.cat([closest_dyn_obs_rpos_gn, closest_dyn_obs_distance_2d, closest_dyn_obs_distance_z, closest_dyn_obs_vel_g, \
                                    closest_dyn_obs_width.unsqueeze(1), closest_dyn_obs_height.unsqueeze(1)], dim=-1).unsqueeze(0).unsqueeze(0)
        # states
        obs = TensorDict({
            "agents": TensorDict({
                "observation": TensorDict({
                    "state": drone_state,
                    "lidar": lidar_scan,
                    "direction": target_dir_2d,
                    "dynamic_obstacle": dyn_obs_states
                })
            })
        })

        has_obstacle_in_range = self.check_obstacle(lidar_scan, dyn_obs_states)
        # if (False):
        if (has_obstacle_in_range):
            if (not self.use_policy_server):
                with set_exploration_type(ExplorationType.MEAN):
                    output = self.policy(obs)
                vel_world = output["agents", "action"]
            else:
                try:
                    get_policy_inference = rospy.ServiceProxy("rl_navigation/GetPolicyInference", GetPolicyInference)

                    response = get_policy_inference(obs["agents"]["observation"]["state"].cpu().numpy().flatten().tolist(),
                                                    obs["agents"]["observation"]["state"].size(),
                                                    obs["agents"]["observation"]["lidar"].cpu().numpy().flatten().tolist(),
                                                    obs["agents"]["observation"]["lidar"].size(), 
                                                    obs["agents"]["observation"]["direction"].cpu().numpy().flatten().tolist(),
                                                    obs["agents"]["observation"]["direction"].size(),
                                                    obs["agents"]["observation"]["dynamic_obstacle"].cpu().numpy().flatten().tolist(),
                                                    obs["agents"]["observation"]["dynamic_obstacle"].size())
                    vel_world = torch.tensor(response.action, device=self.cfg.device, dtype=torch.float).unsqueeze(0).unsqueeze(0)
                except rospy.service.ServiceException as e:
                    print("[nav-ros]: Policy server err!")
                    vel_world = torch.tensor([0., 0., 0.], device=self.cfg.device).unsqueeze(0).unsqueeze(0)
        else:
            vel_world =  (goal - pos)/torch.norm(goal - pos) * self.cfg.algo.actor.action_limit
        return vel_world


    def get_rollout_traj(self, pos: torch.Tensor, vel: torch.Tensor, goal: torch.Tensor, dt=0.1, horizon=3.0):
        traj = [pos.cpu().detach().numpy()]
        t = 0.
        while (t < horizon):
            vel_curr_world = self.get_action(pos, vel, goal)
            t += dt
            pos = (pos + dt * vel_curr_world).squeeze(0).squeeze(0)
            vel = vel_curr_world.squeeze(0).squeeze(0)
            traj.append(pos.cpu().detach().numpy())
        return np.array(traj)

    def control_callback(self, event):
        if (not self.odom_received):
            return

        if (not self.goal_received or len(self.raypoints) == 0 or len(self.dynamic_obstacles) == 0):
            self.pose_pub.publish(self.takeoff_pose)
            return

        # 到达目标后进入锁存保持状态：目标点只发布一次位置设定值，之后不再继续发布速度指令。
        # Sunray 的 MOVE_POINT 是事件型命令，发送一次即可持续定点保持；继续发布速度指令反而容易在目标附近抖动。
        if (self.goal_hold_latched):
            return

        # safety_stop 由 safety_check() 线程中的终端 Enter 键触发。
        # stop_pose 是触发停止时记录的当前 odom 位姿，不是目标点，也不是固定参数。
        # 这里持续发布 stop_pose，用于在人工暂停期间保持当前位置。
        if (self.safety_stop):
            self.pose_pub.publish(self.stop_pose)
            return
        
        start_time = time.time()
        # check for angle
        self.update_target_dir()
        goal_angle = np.arctan2(self.target_dir[1].cpu().numpy(), self.target_dir[0].cpu().numpy())
        _, _, curr_angle = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])
        angle_diff = np.abs(goal_angle - curr_angle)
        if (angle_diff > math.pi):
            angle_diff = np.abs(angle_diff - math.pi * 2)
        if (angle_diff >= 0.1):
            pose_msg = PoseStamped()
            pose_msg.pose = self.odom.pose.pose
            quaternion = tf.transformations.quaternion_from_euler(0, 0, goal_angle)
            pose_msg.pose.orientation.w = quaternion[3]
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            self.pose_pub.publish(pose_msg)
            return
        else:
            self.stable_times += 1
            if (self.stable_times <= 10):
                return

        pos = torch.tensor([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z], device=self.cfg.device)
        goal = torch.tensor([self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z], device=self.cfg.device)
        orientation = torch.tensor([self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z], device=self.cfg.device)
        rot = self.quaternion_to_rotation_matrix(self.odom.pose.pose.orientation)
        vel_body = np.array([self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y, self.odom.twist.twist.linear.z])
        vel_world = torch.tensor(rot @ vel_body, device=self.cfg.device, dtype=torch.float) # world vel
        
        # get RL action from model
        cmd_vel_world = self.get_action(pos, vel_world, goal).squeeze(0).squeeze(0).detach().cpu().numpy()        
        self.cmd_vel_world = cmd_vel_world.copy()

        # get safe action
        safe_cmd_vel_world = self.get_safe_action(vel_world, cmd_vel_world)
        # safe_cmd_vel_world = self.get_safe_action_map(vel_world, cmd_vel_world)
        # safe_cmd_vel_world[2] = 0
        self.safe_cmd_vel_world = safe_cmd_vel_world.copy()
        quat_no_tilt = tf.transformations.quaternion_from_euler(0, 0, curr_angle)
        quat_msg = Quaternion()
        quat_msg.w = quat_no_tilt[3]
        quat_msg.x = quat_no_tilt[0]
        quat_msg.y = quat_no_tilt[1]
        quat_msg.z = quat_no_tilt[2]
        rot_no_tilt = self.quaternion_to_rotation_matrix(quat_msg)
        safe_cmd_vel_local = np.linalg.inv(rot_no_tilt) @ safe_cmd_vel_world

        # Goal condition
        distance = (pos - goal).norm() 
        if (distance <= 2. and distance > 0.4):
            if (np.linalg.norm(safe_cmd_vel_local) != 0):
                safe_cmd_vel_local = 0.5 * safe_cmd_vel_local/np.linalg.norm(safe_cmd_vel_local)
                safe_cmd_vel_world = 0.5 * safe_cmd_vel_world/np.linalg.norm(safe_cmd_vel_world)
        # Sunray适配修改：原始NAVRL在1.0m内清零速度，仍然持续发布cmd_vel。
        # 这里改为0.4m内发布明确的位置设定值，使适配器转换为Sunray MOVE_POINT定点保持。
        # elif (distance <= 1.0):
        #     safe_cmd_vel_local *= 0.
        #     safe_cmd_vel_world *= 0.
        if (distance <= 0.4):
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = rospy.Time.now()
            goal_pose_msg.header.frame_id = "map"
            goal_pose_msg.pose.position.x = self.goal.pose.position.x
            goal_pose_msg.pose.position.y = self.goal.pose.position.y
            goal_pose_msg.pose.position.z = self.goal.pose.position.z
            quaternion = tf.transformations.quaternion_from_euler(0, 0, goal_angle)
            goal_pose_msg.pose.orientation.w = quaternion[3]
            goal_pose_msg.pose.orientation.x = quaternion[0]
            goal_pose_msg.pose.orientation.y = quaternion[1]
            goal_pose_msg.pose.orientation.z = quaternion[2]
            self.pose_pub.publish(goal_pose_msg)
            self.cmd_vel_world = np.zeros(3)
            self.safe_cmd_vel_world = np.zeros(3)
            self.has_action = True
            self.goal_hold_latched = True
            rospy.loginfo(
                "[nav-ros]: goal reached within 0.4 m, publish one setpoint_pose and stop velocity commands."
            )
            return

        # final action
        if (self.px4_control):
            final_cmd_vel = PositionTarget()
            final_cmd_vel.coordinate_frame = final_cmd_vel.FRAME_LOCAL_NED
            final_cmd_vel.header.stamp = rospy.Time.now()
            final_cmd_vel.header.frame_id = "map"
            if (self.height_control):
                final_cmd_vel.velocity.x = safe_cmd_vel_world[0]
                final_cmd_vel.velocity.y = safe_cmd_vel_world[1]
                final_cmd_vel.velocity.z = safe_cmd_vel_world[2]
                final_cmd_vel.yaw = goal_angle
                final_cmd_vel.type_mask = final_cmd_vel.IGNORE_PX + final_cmd_vel.IGNORE_PY + final_cmd_vel.IGNORE_PZ + \
                    final_cmd_vel.IGNORE_AFX + final_cmd_vel.IGNORE_AFY + final_cmd_vel.IGNORE_AFZ + final_cmd_vel.IGNORE_YAW_RATE
            else:
                final_cmd_vel.velocity.x = safe_cmd_vel_world[0]
                final_cmd_vel.velocity.y = safe_cmd_vel_world[1]
                final_cmd_vel.position.z = self.takeoff_pose.pose.position.z
                final_cmd_vel.yaw = goal_angle
                final_cmd_vel.type_mask = final_cmd_vel.IGNORE_PX + final_cmd_vel.IGNORE_PY + final_cmd_vel.IGNORE_VZ + \
                    final_cmd_vel.IGNORE_AFX + final_cmd_vel.IGNORE_AFY + final_cmd_vel.IGNORE_AFZ + final_cmd_vel.IGNORE_YAW_RATE                           
        else:
            final_cmd_vel = TwistStamped()
            final_cmd_vel.header.stamp = rospy.Time.now()
            #final_cmd_vel.twist.linear.x = safe_cmd_vel_local[0]
            #final_cmd_vel.twist.linear.y = safe_cmd_vel_local[1]
            final_cmd_vel.twist.linear.x = safe_cmd_vel_world[0]
            final_cmd_vel.twist.linear.y = safe_cmd_vel_world[1]
            if (self.height_control):
                final_cmd_vel.twist.linear.z = safe_cmd_vel_world[2]
            else:
                final_cmd_vel.twist.linear.z = 0
        self.action_pub.publish(final_cmd_vel)
        self.has_action = True

        # rollout_traj = self.get_rollout_traj(pos, vel_world, goal, dt=0.1, horizon=3.0)
        # traj_msg = Path()
        # traj_msg.header.frame_id = "map"
        # for i in range(len(rollout_traj)):
        #     p = PoseStamped()
        #     p.pose.position.x = rollout_traj[i][0]
        #     p.pose.position.y = rollout_traj[i][1]
        #     p.pose.position.z = rollout_traj[i][2]
        #     traj_msg.poses.append(p)
        # self.rollout_traj_pub.publish(traj_msg)
        end_time = time.time()
        # print("[nav-ros]: control time ", end_time - start_time)
        
    def pause_sim():
        rospy.wait_for_service('/gazebo/pause_physics')
        pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause()

    def unpause_sim():
        rospy.wait_for_service('/gazebo/unpause_physics')
        unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause()

    def run(self):
        raycast_timer = rospy.Timer(rospy.Duration(0.05), self.raycast_callback)
        raycast_vis_timer = rospy.Timer(rospy.Duration(0.05), self.raycast_vis_callback)
        control_timer = rospy.Timer(rospy.Duration(0.05), self.control_callback)
        goal_vis_timer = rospy.Timer(rospy.Duration(0.05), self.goal_vis_callback)
        # dynamic_obstacle_timer = rospy.Timer(rospy.Duration(0.05), self.dynamic_obstacle_callback)
        # dynamic_obstacle_vis_timer = rospy.Timer(rospy.Duration(0.05), self.dynamic_obstacle_vis_callback)
        cmd_vis_timer = rospy.Timer(rospy.Duration(0.05), self.cmd_vis_callback)

    def raycast_vis_callback(self, event):
        if not self.odom_received and not self.goal_received:
            return
        msg = MarkerArray()
        pos = self.odom.pose.pose.position
        direction_init = None
        for i in range(len(self.raypoints)):
            point = Marker()
            point.header.frame_id = "map"
            point.header.stamp = rospy.get_rostime()
            point.ns = "raycast_points"
            point.id = i
            point.type = point.SPHERE
            point.action = point.ADD
            point.pose.position.x = self.raypoints[i][0]
            point.pose.position.y = self.raypoints[i][1]
            point.pose.position.z = self.raypoints[i][2]
            point.lifetime = rospy.Time(0.5)
            point.scale.x = 0.1
            point.scale.y = 0.1
            point.scale.z = 0.1
            point.color.a = 1.0
            point.color.r = 1.0
            msg.markers.append(point)

            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = rospy.get_rostime()
            line.ns = "raycast_lines"
            line.id = i
            line.type = line.LINE_LIST
            p = Point()
            p.x = self.raypoints[i][0]
            p.y = self.raypoints[i][1]
            p.z = self.raypoints[i][2]
            line.points.append(p)
            line.points.append(pos)
            line.scale.x = 0.03
            line.scale.y = 0.03
            line.scale.z = 0.03
            x_diff = (p.x - self.odom.pose.pose.position.x)
            y_diff = (p.y - self.odom.pose.pose.position.y)
            direction = np.array([x_diff, y_diff])
            direction = direction/np.linalg.norm(direction)
            if (i == 0 or (np.linalg.norm(direction - direction_init) <= 0.1)):
                line.color.b = 1.0
                line.color.a = 1.0
                if (i == 0):
                    direction_init = direction
            else:
                line.color.g = 1.0
                line.color.a = 0.5
            line.lifetime = rospy.Time(0.5)
            msg.markers.append(line)
        self.raycast_vis_pub.publish(msg)
    
    def goal_vis_callback(self, event):
        if not self.goal_received:
            return
        msg = MarkerArray()
        goal_point = Marker()
        goal_point.header.frame_id = "map"
        goal_point.header.stamp = rospy.get_rostime()
        goal_point.ns = "goal_point"
        goal_point.id = 1
        goal_point.type = goal_point.SPHERE
        goal_point.action = goal_point.ADD
        goal_point.pose.position.x = self.goal.pose.position.x
        goal_point.pose.position.y = self.goal.pose.position.y
        goal_point.pose.position.z = self.goal.pose.position.z
        goal_point.lifetime = rospy.Time(0.1)
        goal_point.scale.x = 0.3
        goal_point.scale.y = 0.3
        goal_point.scale.z = 0.3
        goal_point.color.r = 1.0
        goal_point.color.b = 1.0
        goal_point.color.a = 1.0
        msg.markers.append(goal_point)
        self.goal_vis_pub.publish(msg)

    def dynamic_obstacle_vis_callback(self, event):
        if (len(self.dynamic_obstacles) == 0):
            return
        dynamic_obstacle_pos = self.dynamic_obstacles[0]
        dynamic_obstacle_size = self.dynamic_obstacles[2]

        msg = MarkerArray()
        for i in range(dynamic_obstacle_pos.size(0)):
            pos = dynamic_obstacle_pos[i]
            size = dynamic_obstacle_size[i]

            # Increase the width
            width = torch.max(size[0], size[1])
            height = size[2]

            # Create the marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "dynamic_obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2] 
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = width
            marker.scale.y = width
            marker.scale.z = height 
            marker.color.a = 0.5  # Alpha value
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0

            msg.markers.append(marker)

        # Publish the marker array
        self.dynamic_obstacle_vis_pub.publish(msg)            

    def cmd_vis_callback(self, event):
        if (not self.has_action):
            return
        msg = MarkerArray()

        # rl action vis
        rl_action_arrow = Marker()
        rl_action_arrow.header.frame_id = "map"
        rl_action_arrow.header.stamp = rospy.get_rostime()
        rl_action_arrow.ns = "rl_action"
        rl_action_arrow.id = 0
        rl_action_arrow.type = rl_action_arrow.ARROW
        rl_action_arrow.action = rl_action_arrow.ADD

        # start
        agent_pos = Point()
        agent_pos.x = self.odom.pose.pose.position.x
        agent_pos.y = self.odom.pose.pose.position.y
        agent_pos.z = self.odom.pose.pose.position.z
        
        # end
        vel_end = Point()
        vel_end.x = self.cmd_vel_world[0] + agent_pos.x
        vel_end.y = self.cmd_vel_world[1] + agent_pos.y
        vel_end.z = self.cmd_vel_world[2] + agent_pos.z

        rl_action_arrow.points.append(agent_pos)
        rl_action_arrow.points.append(vel_end)
        rl_action_arrow.lifetime = rospy.Duration(0.1)
        rl_action_arrow.scale.x = 0.06
        rl_action_arrow.scale.y = 0.06
        rl_action_arrow.scale.z = 0.06
        rl_action_arrow.color.a = 1.0
        rl_action_arrow.color.r = 1.0
        rl_action_arrow.color.g = 0.0
        rl_action_arrow.color.b = 0.0
        msg.markers.append(rl_action_arrow)


        # safe action vis
        safe_action_arrow = Marker()
        safe_action_arrow.header.frame_id = "map"
        safe_action_arrow.header.stamp = rospy.get_rostime()
        safe_action_arrow.ns = "safe_action"
        safe_action_arrow.id = 1
        safe_action_arrow.type = safe_action_arrow.ARROW
        safe_action_arrow.action = safe_action_arrow.ADD

        # start
        agent_pos = Point()
        agent_pos.x = self.odom.pose.pose.position.x
        agent_pos.y = self.odom.pose.pose.position.y
        agent_pos.z = self.odom.pose.pose.position.z

        # end
        vel_end = Point()
        vel_end.x = self.safe_cmd_vel_world[0] + agent_pos.x
        vel_end.y = self.safe_cmd_vel_world[1] + agent_pos.y
        vel_end.z = self.safe_cmd_vel_world[2] + agent_pos.z

        safe_action_arrow.points.append(agent_pos)
        safe_action_arrow.points.append(vel_end)
        safe_action_arrow.lifetime = rospy.Duration(0.1)
        safe_action_arrow.scale.x = 0.06
        safe_action_arrow.scale.y = 0.06
        safe_action_arrow.scale.z = 0.06
        safe_action_arrow.color.a = 1.0
        safe_action_arrow.color.r = 0.0
        safe_action_arrow.color.g = 1.0
        safe_action_arrow.color.b = 0.0

        msg.markers.append(safe_action_arrow)
        self.cmd_vis_pub.publish(msg)
