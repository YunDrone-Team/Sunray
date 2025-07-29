import numpy as np

class APF_Planner:
    def __init__(self, start, goal, obstacles, safe_zone, k_attract=4.0, k_repel=1.0, r0=0.17, d_safe=0.15, v_max=0.3):
    # def __init__(self, start, goal, obstacles, safe_zone, k_attract=4.0, k_repel=1.0, r0=0.65, d_safe=0.15, v_max=0.3):
        """
        人工势场法路径规划器
        
        参数:
        start: 起点坐标 (x, y)
        goal: 目标点坐标 (x, y)
        obstacles: 障碍物列表 [(x1, y1), (x2, y2), ...]
        safe_zone: 安全区域边界 (x_min, x_max, y_min, y_max)
        k_attract: 引力系数，控制目标点吸引力强度
        k_repel: 斥力系数，控制障碍物排斥力强度
        r0: 障碍物斥力作用范围
        d_safe: 安全边界缓冲距离
        v_max: 最大允许速度
        """
        # 初始化起点和目标点
        self.start = np.array(start)
        self.goal = np.array(goal)
        
        # 将障碍物转换为NumPy数组
        self.obstacles = [np.array(obs) for obs in obstacles]
        
        # 设置安全区域
        self.safe_zone = safe_zone
        
        # 设置势场参数
        self.k_attract = k_attract
        self.k_repel = k_repel
        self.r0 = r0
        self.d_safe = d_safe
        self.v_max = v_max
        
        # 初始化路径和当前位置
        self.path = [self.start.copy()]
        self.current_pos = self.start.copy()

    def update_obstacles(self, obstacles):
        """
        更新障碍物列表
        
        参数:
        obstacles: 新的障碍物列表 [(x1, y1), (x2, y2), ...]
        """
        self.obstacles = [np.array(obs) for obs in obstacles]
    
    def _attractive_force(self, pos):
        """
        计算目标点的引力
        
        参数:
        pos: 当前位置 (x, y)
        
        返回:
        引力向量 (F_x, F_y)
        """
        # 计算当前位置到目标点的方向向量
        direction = self.goal - pos
        
        # 计算到目标点的距离
        distance = np.linalg.norm(direction)
        
        # 避免除以零错误
        if distance > 0:
            # 引力 = 引力系数 * 方向向量 / 距离(单位向量) 引力的模长是固定的
            return self.k_attract * direction / distance
        else:
            return np.zeros(2)  # 已在目标点，无引力
    
    def _repulsive_force(self, pos):
        """
        计算障碍物的斥力
        
        参数:
        pos: 当前位置 (x, y)
        
        返回:
        所有障碍物的斥力合力 (F_x, F_y)
        """
        force = np.zeros(2)  # 初始化斥力向量
        
        for obs in self.obstacles:
            # 计算当前位置到障碍物的向量
            vec_to_obs = pos - obs
            
            # 计算到障碍物的距离
            dist = np.linalg.norm(vec_to_obs) - 0.38  # 减去无人机和障碍物的半径和
            # dist = dist - 0.25  # 减去无人机半径
            # print("obs:", obs, "distance:", dist)
            
            # 如果障碍物在作用范围内(r0)，则计算斥力
            if dist < self.r0:
                # 斥力计算: 
                # (1/dist - 1/r0) 表示距离越近斥力越大，在r0距离处斥力为零
                # / dist**3 确保斥力随距离快速衰减
                # 乘以方向向量得到向量形式的斥力
                rep_force = self.k_repel * (1/dist - 1/self.r0) * (vec_to_obs / dist**3)
                force += rep_force  # 累加到总斥力
                # print(f"Obstacle at {obs}, Distance: {dist}, Repulsive Force: {rep_force}")
        
        return force
    
    def _boundary_force(self, pos):
        """
        计算安全区边界的斥力（防止无人机飞出安全区）
        
        参数:
        pos: 当前位置 (x, y)
        
        返回:
        边界斥力向量 (F_x, F_y)
        """
        x, y = pos
        x_min, x_max, y_min, y_max = self.safe_zone
        force = np.zeros(2)  # 初始化边界力向量

        dis2left = x - x_min
        dis2right = x_max - x
        dis2down = y - y_min
        dis2up = y_max - y
        
        # # 检查X轴左边界
        # if abs(dis2left) < self.d_safe:
        #     print("dis2left:", dis2left)
        #     force[0] += self.k_repel * (1/abs(dis2left) - 1/self.d_safe)
        
        # # 检查X轴右边界
        # elif abs(dis2right) < self.d_safe:
        #     # 离右边界越近，向左的斥力越大
        #     print("dis2right:", dis2right)
        #     force[0] -= self.k_repel * (1/abs(dis2right) - 1/self.d_safe)

        # 检查X轴右边界，左边界是起点，不用检测
        if abs(dis2right) < self.d_safe:
            # 离右边界越近，向左的斥力越大
            # print("dis2right:", dis2right)
            force[0] -= self.k_repel * (1/abs(dis2right) - 1/self.d_safe)
            
        # 检查Y轴下边界
        if abs(dis2down) < self.d_safe:
            # 离下边界越近，向上的斥力越大
            # print("dis2down:", dis2down)
            force[1] += self.k_repel * (1/abs(dis2down) - 1/self.d_safe)
        
        # 检查Y轴上边界
        elif abs(dis2up) < self.d_safe:
            # 离上边界越近，向下的斥力越大
            # print("dis2up:", dis2up)
            force[1] -= self.k_repel * (1/abs(dis2up) - 1/self.d_safe)
        
        print(f"Boundary Force: {force}")
        return force
    
    def calculate_velocity(self, pos):
        """
        计算当前位置的目标速度
        
        参数:
        pos: 指定位置，默认为当前无人机位置
        
        返回:
        目标速度向量 (v_x, v_y)
        """
        # 计算各分力：引力、障碍物斥力、边界斥力
        F_att = self._attractive_force(pos)
        F_rep = self._repulsive_force(pos)
        F_boundary = self._boundary_force(pos)
        
        # 计算合力：引力 + 障碍物斥力 + 边界斥力
        F_total = F_att + F_rep + F_boundary
        # F_total = F_att + F_rep
        
        # 计算合力大小
        force_magnitude = np.linalg.norm(F_total)
        
        # 如果合力不为零，计算速度向量
        if force_magnitude > 0:
            # 速度方向与合力方向相同，大小按比例缩放至v_max
            velocity = F_total * self.v_max / force_magnitude
        else:
            # 合力为零时，施加随机扰动速度（避免陷入局部极小值）
            angle = np.random.uniform(0, 2 * np.pi)  # 随机生成角度 [0, 2π]
            disturbance_magnitude = 0.2 * self.v_max  # 扰动大小为最大速度的20%
            velocity = np.array([
                disturbance_magnitude * np.cos(angle),
                disturbance_magnitude * np.sin(angle)
            ])
        return velocity
    
    def calculate_target_position(self, pos, dt=0.1):
        """
        计算下一时刻的目标位置
        
        参数:
        pos: 指定位置，默认为当前无人机位置
        dt: 时间步长
        
        返回:
        下一时刻目标位置 (x, y)
        """

        # 计算当前速度
        velocity = self.calculate_velocity(pos)
        
        # 目标位置 = 当前位置 + 速度 × 时间步长
        return pos + velocity * dt
    
    def step(self, pos=None, dt=0.1, output_type='velocity'):
        """
        执行单步规划
        
        参数:
        pos: 指定位置，默认为当前无人机位置
        dt: 时间步长
        output_type: 输出类型 ('velocity' 或 'position')
        
        返回:
        目标速度或目标位置
        
        异常:
        ValueError: 如果输出类型不合法
        """
        # 如果提供了新位置，更新当前位置
        if pos is not None:
            self.current_pos = np.array(pos)
            
        # 根据请求的输出类型返回相应值
        if output_type == 'velocity':
            return self.calculate_velocity(pos)
        elif output_type == 'position':
            return self.calculate_target_position(pos, dt=dt)
        else:
            # 无效输出类型时抛出异常
            raise ValueError("无效输出类型。请使用 'velocity' 或 'position'")
    
    def plan(self, max_steps=1000, tol=0.1, dt=0.1):
        """
        执行完整路径规划
        
        参数:
        max_steps: 最大规划步数
        tol: 到达目标的容差距离
        dt: 时间步长
        
        返回:
        完整路径列表 [(x0,y0), (x1,y1), ...]
        """
        # 重置当前位置和路径
        self.current_pos = self.start.copy()
        self.path = [self.current_pos.copy()]
        
        # 执行路径规划
        for _ in range(max_steps):
            # 检查是否到达目标点（距离小于容差）
            if np.linalg.norm(self.current_pos - self.goal) < tol:
                break
                
            # 获取下一目标位置
            target_pos = self.step(output_type='position', dt=dt)
            
            # 更新当前位置
            self.current_pos = target_pos
            
            # 记录路径点
            self.path.append(self.current_pos.copy())
            
        return self.path