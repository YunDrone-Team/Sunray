import rospy 
import math 
import time
from sunray_msgs.msg import UAVState
from datetime import datetime, timedelta
from geometry_msgs.msg import PoseStamped
import threading
from typing import Dict, Tuple


warn_sum=0
score=0
crash_obs_count=0
# 全局变量：存储目标位置数据
obs_1 = (0.0, 0.0, 0.0)
obs_2 = (0.0, 0.0, 0.0)
obs_3 = (0.0, 0.0, 0.0)
delivery_target = (0.0, 0.0, 0.0)

# 全局锁：保证多线程下全局变量的读写安全
global_data_lock = threading.Lock()

class GlobalPoseUpdater:
    """后台更新全局位置变量的类"""
    
    def __init__(self, node_name: str = "global_pose_updater"):
        self._node_name = node_name
        self._is_running = False
        self._ros_thread = None  # 存储ROS后台线程
        self._subscribers = []   # 存储订阅者对象
        
        # 自动启动后台更新
        self.start_background()

    def _ros_background_worker(self):
        """ROS后台工作线程：初始化节点并处理订阅"""
        # 初始化ROS节点（避免重复初始化）
        # if not rospy.core.is_initialized():
        #     rospy.init_node(self._node_name, anonymous=True, disable_signals=True)
        rospy.loginfo("全局变量更新线程已启动")
        
        # 订阅所有目标话题
        self._subscribers = [
            rospy.Subscriber('/obstcale_1/pose', PoseStamped, self._update_obs1),
            rospy.Subscriber('/obstcale_2/pose', PoseStamped, self._update_obs2),
            rospy.Subscriber('/obstcale_3/pose', PoseStamped, self._update_obs3),
            rospy.Subscriber('/delivery_target/pose', PoseStamped, self._update_target)
        ]
        
        # 保持线程运行（不阻塞主线程）
        rate = rospy.Rate(100)  # 100Hz循环检查
        while self._is_running and not rospy.is_shutdown():
            rate.sleep()
        
        # 清理订阅者
        for sub in self._subscribers:
            sub.unregister()
        rospy.loginfo("全局变量更新线程已停止")

    def _update_obs1(self, msg: PoseStamped):
        """更新全局变量obs_1"""
        global obs_1
        with global_data_lock:
            obs_1 = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _update_obs2(self, msg: PoseStamped):
        """更新全局变量obs_2"""
        global obs_2
        with global_data_lock:
            obs_2 = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _update_obs3(self, msg: PoseStamped):
        """更新全局变量obs_3"""
        global obs_3
        with global_data_lock:
            obs_3 = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _update_target(self, msg: PoseStamped):
        """更新全局变量delivery_target"""
        global delivery_target
        with global_data_lock:
            delivery_target = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def start_background(self):
        """启动后台更新线程"""
        if not self._is_running:
            self._is_running = True
            self._ros_thread = threading.Thread(target=self._ros_background_worker, daemon=True)
            self._ros_thread.start()

    def stop_background(self):
        """停止后台更新线程"""
        if self._is_running:
            self._is_running = False
            if self._ros_thread is not None:
                self._ros_thread.join(timeout=1.0)
                self._ros_thread = None

    def __del__(self):
        """对象销毁时自动停止后台线程"""
        self.stop_background()


class CollisionCounter_obs:
    """碰撞计数器（处理状态转换和去抖动）"""
    def __init__(self):
        self.collision_count = 0  # 统一使用collision_count作为属性名
        self.current_state = "NO_COLLISION"
        self.last_collision_time = 0
        self.debounce_time = 0.1  # 防抖时间（秒）
    
    def update(self, is_collision, current_time, uav_pos=None, distance=None):
        """更新碰撞状态并计数，新增参数用于打印信息"""
        if self.current_state == "NO_COLLISION" and is_collision:
            # 从无碰撞到碰撞：检查防抖时间
            if current_time - self.last_collision_time > self.debounce_time:
                self.collision_count += 1
                self.last_collision_time = current_time
                # 打印碰撞信息
                print(f"检测到碰撞！UAV位置: {uav_pos}, 距离: {distance:.4f}，检测距离")
            self.current_state = "COLLISION"
        elif self.current_state == "COLLISION" and not is_collision:
            # 从碰撞到无碰撞：更新状态
            self.current_state = "NO_COLLISION"
    
    def get_collision_count(self):
        """获取碰撞次数"""
        return self.collision_count

class CollisionDetector_obs:
    def __init__(self):
        self.collision_counter = CollisionCounter_obs()
    
    def check_collision(self, uav_pos, uav_radius, obs_pos, obs_radius):
        dx = uav_pos[0] - obs_pos[0]
        dy = uav_pos[1] - obs_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        return distance < (obs_radius + uav_radius), distance  # 同时返回是否碰撞和距离
    
    def detect_with_count(self, uav_pos, uav_radius, obs_pos, obs_radius):
        current_time = rospy.get_time() if rospy.core.is_initialized() else time.time()
        is_collision, distance = self.check_collision(uav_pos, uav_radius, obs_pos, obs_radius)  # 获取距离
        # 传递额外参数给update方法
        self.collision_counter.update(is_collision, current_time, uav_pos, distance)
        return is_collision, self.collision_counter.get_collision_count()
# 全局变量：记录警告状态
height_warned = False
position_warned = False

def check_xyz(position):
    global warn_sum, height_warned, position_warned
    
    # 重置警告标志（每次循环开始时清除上次的警告状态）
    if not height_warned and not position_warned:
        pass  # 首次检查或上次警告已处理
    
    # 高度检查
    if not (0.5 <= position[2] <= 1.5):
        if not height_warned:  # 仅在未警告过时打印
            rospy.loginfo(f"无人机高度超过可行范围!z={position[2]}")
            height_warned = True  # 标记已警告
            warn_sum += 1
    else:
        height_warned = False  # 高度恢复正常，重置标记
    
    # 位置检查
    if not (-2.5 <= position[0] <= 2.5 and -2.5 <= position[1] <= 2.5):
        if not position_warned:  # 仅在未警告过时打印
            rospy.loginfo(f"无人机xy超过可行范围!x={position[0]} y={position[1]}")
            position_warned = True  # 标记已警告
            warn_sum += 1
    else:
        position_warned = False  # 位置恢复正常，重置标记


class Uav_state:
    def __init__(self):
        self.node_name = "Score_system" 
        self.uav_state = UAVState() 
        self.uav_id = rospy.get_param("~uav_id", 1) #
        self.uav_name = rospy.get_param("~uav_name", "uav")
        self.uav_name = f"/{self.uav_name}{self.uav_id}"
        self.height_limit={ "min": 0.5,"max": 1.5}
        self.area_boundary = {
            "x_min": -2.5,   # X轴最小值
            "x_max": 2.5,    # X轴最大值
            "y_min": -2.5,   # Y轴最小值
            "y_max": 2.5     # Y轴最大值
        }
        self.uav_state_sub = rospy.Subscriber(
            self.uav_name + "/sunray/uav_state", UAVState, self.uav_state_cb)
    def uav_state_cb(self, msg):
        self.uav_state = msg
    def wait_for_connection(self):
        rate = rospy.Rate(1.0) #1 Hz
        times = 0
        while not rospy.is_shutdown() and not self.uav_state.connected:
            times += 1
            if times > 5:
                rospy.loginfo(f"{self.node_name}:Wait for UAV connect...")
            rate.sleep()
        rospy.loginfo(f"{self.node_name}:UAV connected!")
    def is_takeoff_complete(self):
        #判断无人机是否完成起飞
        start_time = rospy.get_time()
        rate = rospy.Rate(10)  # 10Hz检测频率
        timeout=60
        rospy.loginfo("开始等待无人机起飞完成...")
        
        while not rospy.is_shutdown() and warn_sum == 0:
            # 检查是否超时
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout:
                rospy.logerr(f"起飞超时！{timeout}秒内未完成起飞")
                return False
            
            # 计算目标高度和当前高度
            try:
                target_height = self.uav_state.home_pos[2] + self.uav_state.takeoff_height
                current_height = self.uav_state.position[2]
                height_diff = abs(current_height - target_height)
            except AttributeError as e:
                rospy.logwarn(f"获取高度数据失败: {e}")
                rate.sleep()
                continue
            
            # 检查飞行模式（根据实际消息结构调整）
            is_flying = False
            if hasattr(self.uav_state, 'mode'):
                is_flying = self.uav_state.mode in ["AUTO", "GUIDED", "OFFBOARD"]
            else:
                rospy.logwarn("UAVState中未找到mode字段，仅使用高度判断")
                is_flying = True  # 没有模式信息时仅依赖高度
            
            # 起飞判断条件（高度差小于等于0.2米且处于飞行模式）
            if height_diff <= 0.1 and is_flying:
                rospy.loginfo(f"起飞完成！当前高度: {current_height:.2f}m, 目标高度: {target_height:.2f}m")
                return True
            
            # 打印实时状态（每2秒一次）
            # if int(elapsed_time) % 2 == 0:
            #     rospy.loginfo(
            #         f"起飞中... 已等待{int(elapsed_time)}s, "
            #         f"当前高度: {current_height:.2f}m, "
            #         f"目标高度: {target_height:.2f}m, "
            #         f"高度差: {height_diff:.2f}m"
            #     )
            rate.sleep()
        
        # ROS节点关闭时退出
        rospy.loginfo("节点已关闭，终止起飞检测")
        return False
    def is_obs_zone_test_obs(self):
        rate = rospy.Rate(20) #20 Hz 
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入障碍区!")
        while not condition_met and not rospy.is_shutdown() and warn_sum == 0:
            condition_obs=(-1 <= self.uav_state.position[0]<= 2.5) and (-2.5 <= self.uav_state.position[1] <=-0.5)  # 自定义函数获取状态
            #print(f"正在检测是否进入障碍区,无人机位置x:{self.uav_state.position[0]} y:{self.uav_state.position[1]}!")
            if condition_obs:  
                condition_met = True 
                print("已进入障碍物区")
            else:
                pass  
            check_xyz(self.uav_state.position)
            rate.sleep()
        count_1 = 0
        count_2 = 0
        count_3 = 0
        print(f"obs_1: ({obs_1[0]:.2f}, {obs_1[1]:.2f})")
        print(f"obs_2: ({obs_2[0]:.2f}, {obs_2[1]:.2f})")
        print(f"obs_3: ({obs_3[0]:.2f}, {obs_3[1]:.2f})")
        detector_obs_1 = CollisionDetector_obs()
        detector_obs_2 = CollisionDetector_obs()
        detector_obs_3 = CollisionDetector_obs()
        rospy.loginfo("\n===== 障碍物碰撞检测 =====")
        # 定义障碍物位置和检测范围
        obs_radius = 0.25
        uav_radius = 0.15
        while not rospy.is_shutdown() and (-1 <= self.uav_state.position[0]<= 2.5) and (-2.5 <= self.uav_state.position[1] <=-0.5) and warn_sum == 0:
           is_collision_1, count_1 = detector_obs_1.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_1[:2] , obs_radius)
           is_collision_2, count_2 = detector_obs_2.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_2[:2], obs_radius)
           is_collision_3, count_3 = detector_obs_3.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_3[:2], obs_radius)
           check_xyz(self.uav_state.position)
        return count_3+count_2+count_1
    def is_rect_zone_test_rect(self):
        rate = rospy.Rate(20) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入穿框区!")
        while not condition_met and not rospy.is_shutdown() and warn_sum == 0:
            condition_rect=(0.5 <= self.uav_state.position[0]<= 2.5) and (-0.5 <= self.uav_state.position[1] <=2.5)  # 自定义函数获取状态
            if condition_rect:  
                condition_met = True 
                rospy.loginfo("已进入穿框区")
                rospy.loginfo(f"经过障碍区后，碰撞{crash_obs_count},总得分为{score}")
            else:
                pass  
            check_xyz(self.uav_state.position)

        
        rospy.loginfo(f"进行穿框区的穿框与碰撞检测!")
        rect_1=(1.46,1.27,1, math.radians(0))#x，y的中心位置，旋转角度deg
        rect_2=(1.26,0.12,0.6,math.radians(0))
        obs_radius = 0.05
        uav_radius = 0.10
        #因为飞行时的高度限制，所以此处只进行框的左右部分检测，不进行上下部分检测
        rect_1_end=calculate_rotated_rod_endpoints(*rect_1)
        rect_2_end=calculate_rotated_rod_endpoints(*rect_2)
        rospy.loginfo(f"坐标点{rect_1_end[0]},{rect_1_end[1]},{rect_2_end[0]},{rect_2_end[1]}!")
        detector_obs_1 = CollisionDetector_obs()
        detector_obs_2 = CollisionDetector_obs()
        detector_obs_3 = CollisionDetector_obs()
        detector_obs_4 = CollisionDetector_obs()
        count_1 = 0
        count_2 = 0
        count_3 = 0
        count_4 = 0
        plane_1 = RotatedPlaneCrossDetector(*rect_1)
        plane_2 = RotatedPlaneCrossDetector(*rect_2)
        while not rospy.is_shutdown() and (0.5 <= self.uav_state.position[0]<= 2.5) and (-0.5 <= self.uav_state.position[1] <=2.5) and warn_sum == 0:
            rect_start = rospy.get_time()
            check_xyz(self.uav_state.position)
            is_collision_1, count_1 = detector_obs_1.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius,rect_1_end[0], obs_radius)
            is_collision_2, count_2 = detector_obs_2.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius,rect_1_end[1], obs_radius)
            is_collision_3, count_3 = detector_obs_3.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius,rect_2_end[0], obs_radius)
            is_collision_4, count_4 = detector_obs_4.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius,rect_2_end[1], obs_radius)
            is_crossing_1 = plane_1.update(self.uav_state.position[0],self.uav_state.position[1])
            is_crossing_2 = plane_2.update(self.uav_state.position[0],self.uav_state.position[1])
            rate.sleep()
        rect_sum=count_1+count_2+count_3+count_4
        rospy.loginfo(f"坐标点{rect_1_end[0]}:{count_1},坐标点{rect_1_end[1]}:{count_2},坐标点{rect_2_end[0]}:{count_3},坐标点{rect_2_end[1]}:{count_4}!")
        rospy.loginfo(f"碰撞次数为{rect_sum}!")
        rospy.loginfo(f"大框穿越次数为{plane_1.get_cross_count()}，小框穿越次数为{plane_2.get_cross_count()}!")
        rect_1_count=plane_1.get_cross_count()
        rect_2_count=plane_2.get_cross_count()
        
        return rect_sum,rect_1_count,rect_2_count
    def wind_zone_text(self):
        rate = rospy.Rate(10) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入风扰区!")
        while not condition_met and not rospy.is_shutdown() and warn_sum == 0:
            condition_rect=(-0.5 <= self.uav_state.position[0]<= 0.5) and (0.5 <= self.uav_state.position[1] <=2.5)  # 自定义函数获取状态
            #print(f"正在检测是否进入障碍区,无人机位置x:{self.uav_state.position[0]} y:{self.uav_state.position[1]}!")
            if condition_rect:  
                condition_met = True 
                print("已进入风扰区")
            else:
                pass  
            rate.sleep()
            check_xyz(self.uav_state.position)
        wind_start = rospy.get_time()
        RMSE_list=[]
        MAE=0
        k=0
        crash_wind=False
        crash_wind_cout=0
        rospy.loginfo(f"风扰区碰撞检测!")
        while not rospy.is_shutdown() and (-0.5 <= self.uav_state.position[0]<= 0.5) and (0.5 <= self.uav_state.position[1] <=2.5) and warn_sum == 0:
            x=self.uav_state.position[0]
            y=self.uav_state.position[1]
            z=self.uav_state.position[2]
            if (-0.65<x<0.65) and (-0.65<y<0.65) and not crash_wind:
                crash_wind_cout=crash_wind_cout+1
                rospy.loginfo(f"碰撞到长方体障碍物!!!!!{crash_wind_cout}次")
                crash_wind=True
            elif (-2.5<x<-0.65) or (0.65<x<2.5) or (-2.5<y<-0.65) or (0.65<y<2.5):
                crash_wind=False
            pe=math.sqrt(x*x+(1.5-y)*(1.5-y)+(1-z)*(1-z))
            MAE=pe+MAE
            RMSE_list.append(pe)
            check_xyz(self.uav_state.position)
            k=k+1
            rate.sleep()
        wind_end = rospy.get_time()
        wind_time = wind_end-wind_start
        if k == 0:
            # 处理k为0的情况，可以设置一个默认值或记录错误
            MAE = 0  # 或其他合适的默认值
            # 可选：添加日志记录
            # import logging
            # logging.warning("k值为0，无法计算MAE，已设置为默认值0")
        else:
            MAE = MAE / k
        RMSE_M=0
        for e in RMSE_list:
            RMSE_M=(e)**2
        RMSE=(RMSE_M/k)**0.5
        if wind_time>=10 and wind_time<=20:
            rospy.loginfo(f"风扰区的MAE为{MAE:.4f},RMSE为{RMSE:.4f}!")
            rospy.loginfo(f"风扰区的停留时间为{wind_time:.4f}秒,计算数量k为{k}!")
            rospy.loginfo(f"后续根据MAE和RMSE进行排名加分!")
        else:
            print("未在风扰区停留10至20秒,因此,不进行附加分评比")
        return wind_time,crash_wind_cout

    def drop_zone_text(self):
        rate = rospy.Rate(10) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入投送区!")
        while not condition_met and not rospy.is_shutdown() and warn_sum == 0:
            condition_rect=(-2.5 <= self.uav_state.position[0]<= -0.5) and (0.5 <= self.uav_state.position[1] <=2.5)  # 自定义函数获取状态
            if condition_rect:  
                condition_met = True 
                print("已进入投送区")
            else:
                pass  
            check_xyz(self.uav_state.position)
        arrive_drop=False
        while not rospy.is_shutdown() and (-2.5 <= self.uav_state.position[0]<= -0.5) and (0.5 <= self.uav_state.position[1] <=2.5) and warn_sum == 0:
            check_xyz(self.uav_state.position)
            if (delivery_target[0]-0.25 <= self.uav_state.position[0]<= delivery_target[0]+0.25) and (delivery_target[1]-0.25 <= self.uav_state.position[1] <=delivery_target[1]+0.25):
                arrive_drop=True
                print("正确进入投送点")
                break
            else:
                pass
        return arrive_drop
    def land_text(self):
        """判断无人机是否完成降落"""
        start_time = rospy.get_time()
        rate = rospy.Rate(10)  # 10Hz检测频率
        timeout = 90  # 降落超时时间设为90秒，可根据实际情况调整
        rospy.loginfo("开始等待无人机降落完成...")
        
        while not rospy.is_shutdown() and warn_sum == 0:
            # 检查是否超时
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout:
                rospy.logerr(f"降落超时！{timeout}秒内未完成降落")
                return False
            
            # 计算目标降落点和当前位置
            try:
                # 假设降落点位置存储在self.landing_position中，格式为[x, y, z]
                target_x = -2
                target_y = -2
                target_z = 0.05
                
                current_x = self.uav_state.position[0]
                current_y = self.uav_state.position[1]
                current_z = self.uav_state.position[2]
                
                # 计算水平位置误差和高度误差
                horizontal_distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
                height_diff = abs(current_z - target_z)
                
            except AttributeError as e:
                rospy.logwarn(f"获取位置数据失败: {e}")
                rate.sleep()
                continue
            except Exception as e:
                rospy.logwarn(f"位置计算错误: {e}")
                rate.sleep()
                continue
            
            # 检查飞行模式（根据实际消息结构调整）
            is_landed = False
            if hasattr(self.uav_state, 'mode'):
                is_landed = self.uav_state.mode in ["LAND", "RTL"]
            else:
                rospy.logwarn("UAVState中未找到mode字段,仅使用位置判断")
            
            # 检查无人机是否处于降落状态或已着陆
            if hasattr(self.uav_state, 'landed_state'):
                # 假设landed_state为1表示降落中，2表示已着陆
                is_landed = is_landed or self.uav_state.landed_state == 2
            
            # 降落判断条件（高度接近地面且水平位置误差小）
            if height_diff <= 0.05 and horizontal_distance <= 0.3 and is_landed:
                rospy.loginfo(f"降落完成！当前位置: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m")
                return horizontal_distance
            
            rate.sleep()
    
        # ROS节点关闭时退出
        rospy.loginfo("节点已关闭，终止降落检测")
        return False

class RotatedPlaneCrossDetector:
    def __init__(self, center_x, center_y, width, rotation_angle=0):
        """
        初始化旋转平面穿越检测器
        
        参数:
        center_x, center_y (float): 面的中心点坐标
        width (float): 面的宽度（长度）
        rotation_angle (float): 面绕Z轴的旋转角度（弧度），初始与x轴平行
        """
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.rotation_angle = rotation_angle
        
        # 计算旋转后的面的两个端点（修正核心）
        half_length = width / 2  # 半长度（更准确的命名）
        
        # point1：从中心沿旋转方向延伸半长度
        self.point1 = (
            center_x + half_length * math.cos(rotation_angle),
            center_y + half_length * math.sin(rotation_angle)
        )
        
        # point2：从中心沿旋转方向的反方向延伸半长度（修正点）
        self.point2 = (
            center_x - half_length * math.cos(rotation_angle),  # 与point1相反的x分量
            center_y - half_length * math.sin(rotation_angle)   # 与point1相反的y分量
        )
        
        # 初始化穿越状态
        self.last_position = None
        self.cross_count = 0
    
    def _is_segment_intersect(self, p1, p2, p3, p4):
        """判断两条线段是否相交（包括端点接触）"""
        def ccw(A, B, C):
            """计算三点的叉积"""
            return (B[0]-A[0])*(C[1]-A[1]) - (B[1]-A[1])*(C[0]-A[0])
        
        ccw1 = ccw(p1, p2, p3)
        ccw2 = ccw(p1, p2, p4)
        ccw3 = ccw(p3, p4, p1)
        ccw4 = ccw(p3, p4, p2)
        
        # 严格相交条件
        if (ccw1 * ccw2 < 0) and (ccw3 * ccw4 < 0):
            return True
        
        # 处理共线或端点接触的情况
        if ccw1 == 0 and self._is_point_on_segment(p3, p1, p2):
            return True
        if ccw2 == 0 and self._is_point_on_segment(p4, p1, p2):
            return True
        if ccw3 == 0 and self._is_point_on_segment(p1, p3, p4):
            return True
        if ccw4 == 0 and self._is_point_on_segment(p2, p3, p4):
            return True
        
        return False
    
    def _is_point_on_segment(self, p, a, b):
        """判断点是否在线段上"""
        return (min(a[0], b[0]) - 1e-9 <= p[0] <= max(a[0], b[0]) + 1e-9) and \
               (min(a[1], b[1]) - 1e-9 <= p[1] <= max(a[1], b[1]) + 1e-9)
    
    def update(self, x, y):
        """更新位置并检测穿越"""
        current_position = (x, y)
        is_crossing = False
        
        if self.last_position is not None:
            is_crossing = self._is_segment_intersect(
                self.last_position, current_position,
                self.point1, self.point2
            )
            
            if is_crossing:
                self.cross_count += 1
        
        self.last_position = current_position
        return is_crossing
    
    def get_cross_count(self):
        return self.cross_count

def calculate_rotated_rod_endpoints(x, y, width, rotat_ang):
    """
    计算旋转杆的两个端点在世界坐标系下的位置（不使用NumPy）
    
    参数:
    x, y (float): 杆的中心位置坐标
    width (float): 杆的长度
    rotat_ang (float): 杆绕自身坐标系的旋转角度（弧度）
    
    返回:
    tuple: 两个端点的坐标 ((x1, y1), (x2, y2))
    """
    # 计算半长度
    half_length = width / 2
    
    # 计算旋转角度的正弦和余弦值
    cos_theta = math.cos(rotat_ang)
    sin_theta = math.sin(rotat_ang)
    
    # 计算端点1的坐标
    x1 = x + half_length * cos_theta
    y1 = y + half_length * sin_theta
    
    # 计算端点2的坐标（中心对称点）
    x2 = x - half_length * cos_theta
    y2 = y - half_length * sin_theta
    
    return ((x1, y1), (x2, y2))

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self):
        return f"Point3D({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

class Cuboid:
    def __init__(self, vertices):
        if len(vertices) != 8:
            raise ValueError("长方体必须包含8个顶点")
            
        self.vertices = vertices
        self.min_x = min(v.x for v in vertices)
        self.max_x = max(v.x for v in vertices)
        self.min_y = min(v.y for v in vertices)
        self.max_y = max(v.y for v in vertices)
        self.min_z = min(v.z for v in vertices)
        self.max_z = max(v.z for v in vertices)
    
    def is_inside(self, point):
        return (self.min_x <= point.x <= self.max_x and
                self.min_y <= point.y <= self.max_y and
                self.min_z <= point.z <= self.max_z)
    
    def distance_to_point(self, point):
        clamped_x = max(self.min_x, min(point.x, self.max_x))
        clamped_y = max(self.min_y, min(point.y, self.max_y))
        clamped_z = max(self.min_z, min(point.z, self.max_z))
        
        dx = point.x - clamped_x
        dy = point.y - clamped_y
        dz = point.z - clamped_z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

class Sphere:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    
    def __repr__(self):
        return f"Sphere(center={self.center}, radius={self.radius:.2f})"

class CollisionDetector:
    def __init__(self, debounce_time=0.05):
        self.debounce_time = debounce_time
        self.current_state = False
        self.pending_state = None
        self.state_change_time = None
        self.collision_count = 0
        self.last_collision_state = False
    
    def check_collision(self, cuboid, sphere):
        distance = cuboid.distance_to_point(sphere.center)
        return distance <= sphere.radius
    
    def update(self, cuboid, sphere, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        collision = self.check_collision(cuboid, sphere)
        
        if collision == self.current_state:
            self.pending_state = None
            self.state_change_time = None
            return self.current_state
        
        if self.pending_state is None:
            self.pending_state = collision
            self.state_change_time = current_time
        else:
            if self.state_change_time is not None:
                elapsed = current_time - self.state_change_time
                if elapsed >= self.debounce_time:
                    if collision and not self.last_collision_state:
                        self.collision_count += 1
                        self.last_collision_state = True
                    if not collision:
                        self.last_collision_state = False
                        
                    self.current_state = self.pending_state
                    self.pending_state = None
                    self.state_change_time = None
        
        return self.current_state
    
    def get_count(self):
        return self.collision_count
    
    def reset(self):
        self.current_state = False
        self.pending_state = None
        self.state_change_time = None
        self.collision_count = 0
        self.last_collision_state = False

class PassageDetector:
    def __init__(self):
        self.has_passed = False
        self.is_inside = False
        self.passage_count = 0
        self.entry_point = None
        self.exit_point = None
    
    def update(self, inner_cuboid, outer_cuboid, sphere):
        was_inside = self.is_inside
        self.is_inside = inner_cuboid.is_inside(sphere.center)
        is_within_boundary = outer_cuboid.is_inside(sphere.center)
        
        if self.is_inside and not was_inside:
            self.entry_point = sphere.center
            self.has_passed = True
        
        if was_inside and not self.is_inside and self.has_passed:
            self.exit_point = sphere.center
            
            if self.entry_point and self.exit_point:
                displacement = math.sqrt(
                    (self.exit_point.x - self.entry_point.x)**2 +
                    (self.exit_point.y - self.entry_point.y)**2 +
                    (self.exit_point.z - self.entry_point.z)**2
                )
                
                if displacement > 0.2:
                    self.passage_count += 1
            
            self.has_passed = False
        
        if not is_within_boundary:
            self.entry_point = None
            self.exit_point = None
            self.has_passed = False
        
        return self.is_inside
    
    def get_passage_count(self):
        return self.passage_count
    
    def reset(self):
        self.has_passed = False
        self.is_inside = False
        self.passage_count = 0
        self.entry_point = None
        self.exit_point = None

def rotate_cube_around_z(cube_vertices, angle_rad):
    center_x = sum(v.x for v in cube_vertices) / 8
    center_y = sum(v.y for v in cube_vertices) / 8
    center_z = sum(v.z for v in cube_vertices) / 8
    
    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)
    
    rotated_vertices = []
    for vertex in cube_vertices:
        x_local = vertex.x - center_x
        y_local = vertex.y - center_y
        z_local = vertex.z - center_z
        
        x_rot = x_local * cos_theta - y_local * sin_theta
        y_rot = x_local * sin_theta + y_local * cos_theta
        z_rot = z_local
        
        rotated_vertices.append(Point3D(x_rot + center_x, y_rot + center_y, z_rot + center_z))
    return rotated_vertices

def rect_pos_cal(x, y, z_center, width, height, depth, thickness):
    half_w = width / 2
    half_h = height / 2
    half_d = depth / 2
    
    rect1 = [
        Point3D(x - half_w, y - half_h, z_center + half_d),
        Point3D(x + half_w, y - half_h, z_center + half_d),
        Point3D(x - half_w, y - half_h - thickness, z_center + half_d),
        Point3D(x + half_w, y - half_h - thickness, z_center + half_d),
        Point3D(x - half_w, y - half_h, z_center - half_d),
        Point3D(x + half_w, y - half_h, z_center - half_d),
        Point3D(x - half_w, y - half_h - thickness, z_center - half_d),
        Point3D(x + half_w, y - half_h - thickness, z_center - half_d),
    ]
    
    rect2 = [
        Point3D(x - half_w, y + half_h, z_center + half_d),
        Point3D(x + half_w, y + half_h, z_center + half_d),
        Point3D(x - half_w, y + half_h + thickness, z_center + half_d),
        Point3D(x + half_w, y + half_h + thickness, z_center + half_d),
        Point3D(x - half_w, y + half_h, z_center - half_d),
        Point3D(x + half_w, y + half_h, z_center - half_d),
        Point3D(x - half_w, y + half_h + thickness, z_center - half_d),
        Point3D(x + half_w, y + half_h + thickness, z_center - half_d),
    ]
    
    rect3 = [
        Point3D(x - half_w, y - half_h, z_center + half_d),
        Point3D(x - half_w - thickness, y - half_h, z_center + half_d),
        Point3D(x - half_w, y + half_h, z_center + half_d),
        Point3D(x - half_w - thickness, y + half_h, z_center + half_d),
        Point3D(x - half_w, y - half_h, z_center - half_d),
        Point3D(x - half_w - thickness, y - half_h, z_center - half_d),
        Point3D(x - half_w, y + half_h, z_center - half_d),
        Point3D(x - half_w - thickness, y + half_h, z_center - half_d),
    ]
    
    rect4 = [
        Point3D(x + half_w, y - half_h, z_center + half_d),
        Point3D(x + half_w + thickness, y - half_h, z_center + half_d),
        Point3D(x + half_w, y + half_h, z_center + half_d),
        Point3D(x + half_w + thickness, y + half_h, z_center + half_d),
        Point3D(x + half_w, y - half_h, z_center - half_d),
        Point3D(x + half_w + thickness, y - half_h, z_center - half_d),
        Point3D(x + half_w, y + half_h, z_center - half_d),
        Point3D(x + half_w + thickness, y + half_h, z_center - half_d),
    ]
    
    return [rect1, rect2, rect3, rect4]

def rect_inner_pos_cal(x, y, z_center, width, height, depth):
    half_w = width / 2
    half_h = height / 2
    half_d = depth / 2
    
    return [
        Point3D(x - half_w, y - half_h, z_center + half_d),
        Point3D(x + half_w, y - half_h, z_center + half_d),
        Point3D(x - half_w, y + half_h, z_center + half_d),
        Point3D(x + half_w, y + half_h, z_center + half_d),
        Point3D(x - half_w, y - half_h, z_center - half_d),
        Point3D(x + half_w, y - half_h, z_center - half_d),
        Point3D(x - half_w, y + half_h, z_center - half_d),
        Point3D(x + half_w, y + half_h, z_center - half_d),
    ]

def rect_pos_cal_outer(x, y, z_center, width, height, depth, thickness):
    half_w = width / 2
    half_h = height / 2
    half_d = depth / 2
    
    return [
        Point3D(x - half_w - thickness, y - half_h - thickness, z_center + half_d + thickness),
        Point3D(x + half_w + thickness, y - half_h - thickness, z_center + half_d + thickness),
        Point3D(x - half_w - thickness, y + half_h + thickness, z_center + half_d + thickness),
        Point3D(x + half_w + thickness, y + half_h + thickness, z_center + half_d + thickness),
        Point3D(x - half_w - thickness, y - half_h - thickness, z_center - half_d - thickness),
        Point3D(x + half_w + thickness, y - half_h - thickness, z_center - half_d - thickness),
        Point3D(x - half_w - thickness, y + half_h + thickness, z_center - half_d - thickness),
        Point3D(x + half_w + thickness, y + half_h + thickness, z_center - half_d - thickness),
    ]

def detect(rect_params, rot_ang, sphere, t, border_detectors, passage_detector):
    x, y, z_center, width, height, depth, thickness = rect_params
    
    border_rects = rect_pos_cal(x, y, z_center, width, height, depth, thickness)
    inner_rect = rect_inner_pos_cal(x, y, z_center, width, height, depth)
    outer_rect = rect_pos_cal_outer(x, y, z_center, width, height, depth, thickness)
    
    rotated_borders = [rotate_cube_around_z(vertices, math.radians(rot_ang)) 
                      for vertices in border_rects]
    rotated_inner = rotate_cube_around_z(inner_rect, math.radians(rot_ang))
    rotated_outer = rotate_cube_around_z(outer_rect, math.radians(rot_ang))
    
    border_cuboids = [Cuboid(vertices) for vertices in rotated_borders]
    inner_cuboid = Cuboid(rotated_inner)
    outer_cuboid = Cuboid(rotated_outer)
    
    for cuboid, detector in zip(border_cuboids, border_detectors):
        detector.update(cuboid, sphere, current_time=t)
    
    passage_detector.update(inner_cuboid, outer_cuboid, sphere)
    
    border_collisions = [d.get_count() for d in border_detectors]
    total_collisions = sum(border_collisions)
    
    return {
        "total_collisions": total_collisions,
        "border_collisions": border_collisions,
        "is_inside": passage_detector.is_inside,
        "passage_count": passage_detector.get_passage_count()
    }

class DetectionSystem:
    def __init__(self):
        self.frames = {}
    
    def add_frame(self, frame_id, rect_params, rotation=0):
        """添加检测框并设置初始旋转角度（Z轴，单位：度）"""
        border_detectors = [CollisionDetector() for _ in range(4)]
        passage_detector = PassageDetector()
        
        self.frames[frame_id] = {
            'params': rect_params,
            'rotation': rotation,  # 存储初始旋转角度
            'border_detectors': border_detectors,
            'passage_detector': passage_detector,
            'last_result': None
        }
    
    def set_frame_rotation(self, frame_id, rotation):
        """更新指定框的旋转角度（Z轴，单位：度）"""
        if frame_id in self.frames:
            self.frames[frame_id]['rotation'] = rotation
        else:
            raise ValueError(f"Frame {frame_id} not found")
    
    def detect_all(self, sphere, t, custom_rotations=None):
        """检测球体与所有框的交互
        
        Args:
            sphere: 球体对象
            t: 当前时间
            custom_rotations: 可选，覆盖默认旋转角度的字典 {frame_id: angle}
        """
        results = {}
        for frame_id, frame in self.frames.items():
            # 使用自定义旋转角度或默认角度
            rot_angle = custom_rotations.get(frame_id, frame['rotation']) if custom_rotations else frame['rotation']
            
            result = detect(
                frame['params'],
                rot_angle,
                sphere,
                t,
                frame['border_detectors'],
                frame['passage_detector']
            )
            frame['last_result'] = result
            results[frame_id] = {
                'result': result,
                'rotation': rot_angle  # 返回实际使用的旋转角度
            }
        return results
    
    def reset(self):
        for frame in self.frames.values():
            for detector in frame['border_detectors']:
                detector.reset()
            frame['passage_detector'].reset()



if __name__ == "__main__":
    rospy.init_node('Score_system', anonymous=True)
    updater = GlobalPoseUpdater()
    rospy.loginfo("障碍物监控器已启动，正在实时更新障碍物位置...")
    start = rospy.get_time()
    UAV_s=Uav_state()
    UAV_s.wait_for_connection()
    fly_keep = True
# 只有当warn_sum等于0时才执行评分逻辑
    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        if UAV_s.is_takeoff_complete() == True:
            rospy.loginfo("检测到自主起飞成功，得分+5分")
            score = score + 5
        else:
            fly_keep = False
    rospy.sleep(2.5) 

    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        crash_obs_count = UAV_s.is_obs_zone_test_obs()
        if crash_obs_count <= 2:
            score = score + 20 - crash_obs_count * 10
        else:
            score = score

    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        crash_frame_count, big_passed, small_passed = UAV_s.is_rect_zone_test_rect()
        rospy.loginfo(f"经过穿框区后，碰撞{crash_frame_count}次,穿越大框{big_passed}次，穿越小框{small_passed}次")
        base_score = 0
        # 检查是否按顺序穿框（先大框后小框）并成功进入风扰区
        if 0 <= big_passed <= 1 or 0 <= small_passed <= 1:
            base_score = 10 * big_passed + 15 * small_passed  # 大框10分 + 小框15分
        
        # 处理碰撞扣分（每次扣10分，最低至0）
        if base_score > 0 and crash_frame_count > 0:
            penalty = crash_frame_count * 10
            final_score = max(base_score - penalty, 0)
        else:
            final_score = base_score  # 未满足基础条件或无碰撞，直接取基础分
        score = score + final_score
        rospy.loginfo(f"经过穿框区后，总得分为{score}分")

    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        windtime, count = UAV_s.wind_zone_text()
        base_score = 15  # 基础分值
        score_w = 0        # 初始得分
        # 判断停留时间是否符合要求（10-20秒）
        if 10 <= windtime <= 20:
            # 停留时间符合要求，基础分15分
            score_w = base_score
            
            # 计算碰撞扣分（每次扣10分，最低扣至0）
            penalty = count * 10
            score_w = max(score_w - penalty, 0)
        else:
            # 停留时间不符合要求（不足10s或超过20s）
            # 扣10分，最低至0
            score_w = max(base_score - 10, 0)
        score = score + score_w
        rospy.loginfo(f"经过风扰区后，总得分为{score}分")

    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        is_drop = UAV_s.drop_zone_text()
        if is_drop:
            score = score + 25   
        rospy.loginfo(f"经过投送区后，总得分为{score}分")

    if not rospy.is_shutdown() and fly_keep and warn_sum == 0:
        distance = UAV_s.land_text()
        if distance < 0.2:
            rospy.loginfo(f"降落区得分为10分,总得分为{score}分")
            score = score + 10
        elif distance > 0.2 and distance < 0.8:
            rospy.loginfo(f"降落区得分为5分,总得分为{score}分")
            score = score + 5
        else:
            rospy.loginfo(f"降落区得分为0分,总得分为{score}分")
            score = score
        rospy.loginfo(f"进入降落区后，总得分为{score}分")

    end = rospy.get_time()
    elapsed = end - start
    if warn_sum == 0:
        rospy.loginfo(f"飞行过程中飞机一直在可行安全空域")
    else:
        rospy.loginfo(f"飞行过程中飞机超出可行安全空域，成绩作废")
    rospy.loginfo(f"{elapsed:.2f}秒")
    rospy.loginfo(f"比赛结束，该队伍总得分为{score}分")






