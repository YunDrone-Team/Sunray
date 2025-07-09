import rospy 
import math 
import time
from sunray_msgs.msg import UAVState
from datetime import datetime, timedelta


warn_sum=0
class CollisionCounter_obs:
    """碰撞计数器（处理状态转换和去抖动）"""
    def __init__(self):
        self.collision_count = 0  # 统一使用collision_count作为属性名
        self.current_state = "NO_COLLISION"
        self.last_collision_time = 0
        self.debounce_time = 0.1  # 防抖时间（秒）
    
    def update(self, is_collision, current_time):
        """更新碰撞状态并计数"""
        if self.current_state == "NO_COLLISION" and is_collision:
            # 从无碰撞到碰撞：检查防抖时间
            if current_time - self.last_collision_time > self.debounce_time:
                self.collision_count += 1
                self.last_collision_time = current_time
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
        return distance < (obs_radius + uav_radius)
    
    def detect_with_count(self, uav_pos, uav_radius, obs_pos, obs_radius):
        current_time = rospy.get_time() if rospy.core.is_initialized() else time.time()
        is_collision = self.check_collision(uav_pos, uav_radius, obs_pos, obs_radius)
        self.collision_counter.update(is_collision, current_time)
        return is_collision, self.collision_counter.get_collision_count()
    
def simulate_horizontal_collision(detector):
    obs_pos = (15, 15)
    obs_radius = 2
    uav_radius = 1.5
    # 无人机轨迹（x, y）
    uav_trajectory = [
        (10, 10),  # t=0s
        (12, 12),  # t=1s
        (14, 14),  # t=2s（接近障碍物）
        (15, 15),  # t=3s（碰撞）
        (15.5, 15.5), # t=3.5s（碰撞中）
        (16, 16),  # t=4s（离开）
        (18, 18),  # t=5s
        (15, 15),  # t=6s（再次碰撞）
        (15, 15),   # t=7s
        (15, 15),   # t=7s
        (10, 10),   # t=7s
        (10, 10)   # t=7s
    ]
    
    print("时间(s)\t是否碰撞\t碰撞次数")
    for t, pos in enumerate(uav_trajectory):
        time.sleep(0.5)  # 延长间隔便于观察
        is_collision, count = detector.detect_with_count(pos, uav_radius, obs_pos, obs_radius)
        print(f"{t}\t{is_collision}\t{count}")
def check_xyz(position):
    if (0.5 <= position[2] <=1.5):
        pass
    else:
        rospy.loginfo(f"无人机高度超过可行范围!z={position[2]}")
        warn_sum=warn_sum+1
    if (-2.5 <= position[0] <=2.5) and (-2.5 <= position[1] <=2.5):
        pass
    else:
        rospy.loginfo(f"无人机xy超过可行范围!x={position[0]} y={position[1]}")
        warn_sum=warn_sum+1

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
    #判断无人机是否完成起飞""
        start_time = rospy.get_time()
        rate = rospy.Rate(10)  # 10Hz检测频率
        timeout=60
        rospy.loginfo("开始等待无人机起飞完成...")
        
        while not rospy.is_shutdown():
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

        while not condition_met and not rospy.is_shutdown():
            condition_obs=(-1 <= self.uav_state.position[0]<= 2.5) and (-2.5 <= self.uav_state.position[1] <=-0.5)  # 自定义函数获取状态
            #print(f"正在检测是否进入障碍区,无人机位置x:{self.uav_state.position[0]} y:{self.uav_state.position[1]}!")
            if condition_obs:  
                condition_met = True 
                print("已进入障碍物区")
            else:
                pass  
            check_xyz(self.uav_state.position)
            rate.sleep()
        detector_obs_1 = CollisionDetector_obs()
        detector_obs_2 = CollisionDetector_obs()
        detector_obs_3 = CollisionDetector_obs()
        rospy.loginfo("\n===== 障碍物碰撞检测 =====")
        # 定义障碍物位置和检测范围
        obs_1 = (-0.5, -2)
        obs_2 = (0.74, -1)
        obs_3 = (1.86, -2)
        obs_radius = 0.25
        uav_radius = 0.15
        while not rospy.is_shutdown() and (-1 <= self.uav_state.position[0]<= 2.5) and (-2.5 <= self.uav_state.position[1] <=-0.5):
           is_collision_1, count_1 = detector_obs_1.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_1, obs_radius)
           is_collision_2, count_2 = detector_obs_2.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_2, obs_radius)
           is_collision_3, count_3 = detector_obs_3.detect_with_count((self.uav_state.position[0],self.uav_state.position[1]), uav_radius, obs_3, obs_radius)
           check_xyz(self.uav_state.position)
        return count_3+count_2+count_1
    def is_rect_zone_test_rect(self):
        rate = rospy.Rate(20) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入穿框区!")
        while not condition_met and not rospy.is_shutdown():
            condition_rect=(0.5 <= self.uav_state.position[0]<= 2.5) and (-0.5 <= self.uav_state.position[1] <=2.5)  # 自定义函数获取状态
            if condition_rect:  
                condition_met = True 
                rospy.loginfo("已进入穿框区")
            else:
                pass  
            check_xyz(self.uav_state.position)
        rospy.loginfo(f"进行穿框区的检测!")
        rect_1=(1.46,1.27,1, math.radians(0))#x，y的中心位置，旋转角度deg
        rect_2=(1.63,0.12,0.6,math.radians(0))
        obs_radius = 0.1
        uav_radius = 0.15
        #因为飞行时的高度限制，所以此处只进行框的左右部分检测，不进行上下部分检测
        rect_1_end=calculate_rotated_rod_endpoints(*rect_1)
        rect_2_end=calculate_rotated_rod_endpoints(*rect_2)
        rospy.loginfo(f"坐标点{rect_1_end[0]},{rect_1_end[1]},{rect_2_end[0]},{rect_2_end[1]}!")
        detector_obs_1 = CollisionDetector_obs()
        detector_obs_2 = CollisionDetector_obs()
        detector_obs_3 = CollisionDetector_obs()
        detector_obs_4 = CollisionDetector_obs()
        plane_1 = RotatedPlaneCrossDetector(*rect_1)
        plane_2 = RotatedPlaneCrossDetector(*rect_2)
        while not rospy.is_shutdown() and (0.5 <= self.uav_state.position[0]<= 2.5) and (-0.5 <= self.uav_state.position[1] <=2.5):
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
        rospy.loginfo(f"碰撞次数为{rect_sum}!")
        rospy.loginfo(f"大框穿越次数为{plane_1.get_cross_count()}，小框穿越次数为{plane_2.get_cross_count()}!")
        rect_1_count=plane_1.get_cross_count()
        rect_2_count=plane_2.get_cross_count()
        
        return rect_sum,rect_1_count,rect_2_count
    def wind_zone_text(self):
        rate = rospy.Rate(10) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入风扰区!")
        while not condition_met and not rospy.is_shutdown():
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
        while not rospy.is_shutdown() and (-0.5 <= self.uav_state.position[0]<= 0.5) and (0.5 <= self.uav_state.position[1] <=2.5):
            rect_start = rospy.get_time()
            x=self.uav_state.position[0]
            y=self.uav_state.position[1]
            z=self.uav_state.position[2]
            pe=math.sqrt(x*x+(1.5-y)*(1.5-y)+(1-z)*(1-z))
            MAE=pe+MAE
            RMSE_list.append(pe)
            check_xyz(self.uav_state.position)
            k=k+1
            rate.sleep()
        wind_end = rospy.get_time()
        wind_time = wind_end-wind_start
        MAE=MAE/k
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
        return wind_time

    def drop_zone_text(self):
        rate = rospy.Rate(10) #20 Hz
        condition_met = False  # 初始化标志
        rospy.loginfo(f"正在检测是否进入投送区!")
        while not condition_met and not rospy.is_shutdown():
            condition_rect=(-2.5 <= self.uav_state.position[0]<= -0.5) and (0.5 <= self.uav_state.position[1] <=2.5)  # 自定义函数获取状态
            if condition_rect:  
                condition_met = True 
                print("已进入投送区")
            else:
                pass  
            check_xyz(self.uav_state.position)
        while not rospy.is_shutdown() and (-2.5 <= self.uav_state.position[0]<= -0.5) and (0.5 <= self.uav_state.position[1] <=2.5):
            check_xyz(self.uav_state.position)
            distance=0
        return distance
    def land_text(self):
        """判断无人机是否完成降落"""
        start_time = rospy.get_time()
        rate = rospy.Rate(10)  # 10Hz检测频率
        timeout = 90  # 降落超时时间设为90秒，可根据实际情况调整
        rospy.loginfo("开始等待无人机降落完成...")
        
        while not rospy.is_shutdown():
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
    score=0
    rospy.init_node('Score_system', anonymous=True)
    start = rospy.get_time()
    UAV_s=Uav_state()
    UAV_s.wait_for_connection()
    fly_keep=True
    if not rospy.is_shutdown() and fly_keep:
        if UAV_s.is_takeoff_complete()==True:
            rospy.loginfo("检测到自主起飞成功，得分+5分")
            score=score+5
        else:
            fly_keep=False
    rospy.sleep(2.5) 
    if not rospy.is_shutdown() and fly_keep:
        if fly_keep:
            crash_obs_count=UAV_s.is_obs_zone_test_obs()
            if crash_obs_count<=2:
                score=score+20-crash_obs_count*10
            else:
                score=score
            rospy.loginfo(f"经过障碍区后，总得分为{score}分")
        else:
            pass
    if not rospy.is_shutdown() and fly_keep:
        if fly_keep:
            crash_rect_count,big,small=UAV_s.is_rect_zone_test_rect()
            rospy.loginfo(f"经过穿框区后，碰撞{crash_rect_count},穿越大框{big}次，穿越小框{small}次")
            rect_score=0
            if big<=1 and small<=1:
                rect_score=big*10+small*15
            else:
                rect_score=0
            if crash_rect_count==0 and small==1:
                rect_score=rect_score
            elif 0<crash_rect_count<=3 and small==1:
                rect_score=rect_score+5-10*crash_rect_count
            else:
                rect_score=0
            score=score+rect_score
            rospy.loginfo(f"经过穿框区后，总得分为{score}分")
        else:
            pass
    if not rospy.is_shutdown() and fly_keep:
        if fly_keep:
            windtime=UAV_s.wind_zone_text()
            score=score+15
        else:
            pass
        rospy.loginfo(f"经过风扰区后，总得分为{score}分")
    if not rospy.is_shutdown() and fly_keep:
        if fly_keep:
            UAV_s.drop_zone_text()
            score=score
            
        else:
            pass
        rospy.loginfo(f"经过投送区后，总得分为{score}分")
    if not rospy.is_shutdown() and fly_keep:
        if fly_keep:
            distance=UAV_s.land_text()
            if distance<0.2:
                rospy.loginfo(f"降落区得分为10分,总得分为{score}分")
                score=score+10
            elif distance>0.2 and distance<0.8:
                rospy.loginfo(f"降落区得分为5分,总得分为{score}分")
                score=score+5
            else:
                rospy.loginfo(f"降落区得分为0分,总得分为{score}分")
                score=score
        else:
            pass
        rospy.loginfo(f"进入降落区后，总得分为{score}分")

    end = rospy.get_time()
    elapsed = end - start
    rospy.loginfo(f"{elapsed:.2f}秒")
    rospy.loginfo(f"比赛结束，该队伍总得分为{score}分")
