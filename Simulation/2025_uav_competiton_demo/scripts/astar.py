import heapq
import numpy as np

class AStar:
    def __init__(self, grid=None):
        """
        grid: 2D numpy array, 0=free, 1=obstacle
        """
        self.grid = grid
        self.resolution = 0.05
        if grid is not None:
            self.rows, self.cols = grid.shape
        else:
            self.rows, self.cols = 0, 0
        self.neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]  # 8邻域包含对角线
        self.uav_radius = 0.08  # 假设 UAV 半径 0.2m


    def heuristic(self, a, b):
        # 欧几里得距离，更适合8邻域
        return self.resolution * ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

    def in_bounds(self, node):
        x, y = node
        return 0 <= x < self.rows and 0 <= y < self.cols

    def passable(self, node):
        x, y = node
        return self.grid[x, y] == 0

    def get_neighbors(self, node):
        result = []
        for dx, dy in self.neighbors:
            neighbor = (node[0] + dx, node[1] + dy)
            if self.in_bounds(neighbor) and self.passable(neighbor):
                result.append(neighbor)
        return result

    def set_grid(self, grid):
        """设置新的grid"""
        self.grid = grid
        self.rows, self.cols = grid.shape

    def search(self, start, goal, grid=None):
        """
        start, goal: (x, y) tuple, grid坐标
        返回路径点列表（包含起点和终点），如果无路返回空列表
        """
        if grid is not None:
            self.set_grid(grid)
        
        if self.grid is None:
            return []
            
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                return self.reconstruct_path(came_from, current)
            closed_set.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                dx, dy = abs(neighbor[0] - current[0]), abs(neighbor[1] - current[1])
                move_cost = self.resolution * (1.414 if dx + dy == 2 else 1)
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return []  # 无路可走

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def create_grid_from_obstacles(self, obstacle_coords, rows, cols, resolution, origin, inflation_radius=0.5):
        """
        obstacle_coords: [(x1, y1), (x2, y2), ...] 世界坐标
        rows, cols: 地图大小
        resolution: 每个栅格的实际长度（米）
        origin: (x0, y0) 地图左下角世界坐标
        inflation_radius: 障碍物膨胀半径（米）
        返回: 0=free, 1=obstacle 的二维np数组
        """
        grid = np.zeros((rows, cols), dtype=int)
        inflation_cells = int(inflation_radius / resolution)  # 膨胀半径对应的栅格数
        
        for ox, oy in obstacle_coords:
            gx = int((ox - origin[0]) / resolution)
            gy = int((oy - origin[1]) / resolution)
            
            # 膨胀障碍物
            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    # 圆形膨胀
                    if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                        new_gx, new_gy = gx + dx, gy + dy
                        if 0 <= new_gy < rows and 0 <= new_gx < cols:
                            grid[new_gy, new_gx] = 1
        return grid
    
    "世界坐标转栅格坐标"
    def world_to_grid(self, x, y, origin, resolution):
        gx = int((x - origin[0]) / resolution)
        gy = int((y - origin[1]) / resolution)
        return (gy, gx)  # 行、列

    "栅格坐标转世界坐标"
    def grid_to_world(self, gy, gx, origin, resolution):
        x = gx * resolution + origin[0]
        y = gy * resolution + origin[1]
        return (x, y)
    
    def smooth_path(self, waypoints, smoothing_factor=0.5, iterations=5):
        """使用简单平滑算法优化路径"""
        if len(waypoints) < 3:
            return waypoints
            
        smoothed = waypoints.copy()
        for _ in range(iterations):
            new_path = [smoothed[0]]  # 保持起点
            for i in range(1, len(smoothed) - 1):
                # 取相邻点的加权平均
                prev_x, prev_y = smoothed[i-1]
                curr_x, curr_y = smoothed[i]
                next_x, next_y = smoothed[i+1]
                
                new_x = curr_x + smoothing_factor * ((prev_x + next_x) / 2 - curr_x)
                new_y = curr_y + smoothing_factor * ((prev_y + next_y) / 2 - curr_y)
                new_path.append((new_x, new_y))
            new_path.append(smoothed[-1])  # 保持终点
            smoothed = new_path
        return smoothed
    
    def interpolate_path(self, waypoints, num_points=3):
        """在路径点之间插值增加密度"""
        if len(waypoints) < 2:
            return waypoints
            
        interpolated = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            curr_x, curr_y = waypoints[i]
            next_x, next_y = waypoints[i + 1]
            
            for j in range(1, num_points + 1):
                t = j / (num_points + 1)
                interp_x = curr_x + t * (next_x - curr_x)
                interp_y = curr_y + t * (next_y - curr_y)
                interpolated.append((interp_x, interp_y))
            interpolated.append(waypoints[i + 1])
        return interpolated
    
    def is_path_safe(self, waypoints, grid, origin, resolution):
        # 由于栅格生成时已经考虑了膨胀半径，这里只需要检查路径点本身
        for wx, wy in waypoints:
            gy, gx = self.world_to_grid(wx, wy, origin, resolution)
            # 检查是否超出边界
            if (gy < 0 or gy >= grid.shape[0] or 
                gx < 0 or gx >= grid.shape[1]):
                return False
            # 检查路径点是否在障碍物内
            if grid[gy, gx] == 1:
                return False
        return True
    
    