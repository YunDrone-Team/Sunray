import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import io

data = None
with open('/home/yundrone/Sunray/sunray_formation/uav_formation/config/circle.yaml') as f:
# with open('/home/yundrone/Sunray/sunray_formation/uav_formation/config/figure_eight.yaml') as f:
    data = yaml.safe_load(f)

# 提取点的数量
num_points = data['initial_point']['num']

# 收集所有点数据
points = []
i = 1
max_axis = 0
while f'point_{i}' in data:
    point_data = data[f'point_{i}']
    points.append((point_data[0], point_data[1]))  # 取前两位作为x,y
    i += 1
    max_axis = max(max_axis, abs(point_data[0]), abs(point_data[1]))

# 准备初始点（这里简化处理，实际可能需要根据ReadyPoint数据）
initial_indices = [data['initial_point'][f'ReadyPoint_{j}'] for j in range(1, num_points+1)]
initial_points = [points[idx] for idx in initial_indices]


# 创建图形
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_aspect('equal')
# 自动调整坐标轴范围
max_axis += 1
ax.set_xlim(-max_axis, max_axis)
ax.set_ylim(-max_axis, max_axis)
ax.grid(True)
ax.set_title('Dynamic Points Visualization')
# 绘制轨迹
for point in points:
    ax.plot(point[0], point[1], 'o', color='gray', markersize=2)
# 初始化物体位置 (不同颜色)
colors = plt.cm.viridis(np.linspace(0, 1, len(initial_points)))
lines = []
for i in range(len(initial_points)):
        line, = ax.plot([], [], 'o', color=colors[i], markersize=10, label=f'Object {i+1}')
        lines.append(line)


# 更新函数
def update(frame):
    for i in range(len(initial_points)):
        idx = (initial_indices[i] + frame) % len(points)
        lines[i].set_data(points[idx][0], points[idx][1])
        # 绘制文字
        # ax.text(points[idx][0], points[idx][1], f'{i+1}', color=colors[i], fontsize=10)
    return lines
        

# 创建动画
ani = FuncAnimation(fig, update, frames=len(points),  blit=True, interval=50)

ax.legend()
plt.tight_layout()
plt.show()