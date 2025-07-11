import rospy
import threading
import time
from typing import Dict, Tuple
from geometry_msgs.msg import PoseStamped

# 全局变量，用于存储三个障碍物的位置
obs_1 = (0.0, 0.0)
obs_2 = (0.0, 0.0)
obs_3 = (0.0, 0.0)

class Obstacle:
    """表示障碍物的类，包含ID、位置和时间戳信息"""
    
    def __init__(self, obstacle_id: int, initial_position: Tuple[float, float] = (0.0, 0.0)):
        self.obstacle_id = obstacle_id
        self._position = initial_position  # 私有属性，存储当前位置
        self._timestamp = rospy.Time.now().to_sec()  # 私有属性，记录最后更新时间
        self._lock = threading.Lock()      # 用于线程安全的锁
    
    def update_position(self, position: Tuple[float, float]):
        """更新障碍物位置并记录时间戳，使用锁保证线程安全"""
        with self._lock:
            self._position = position
            self._timestamp = rospy.Time.now().to_sec()
    
    def get_position(self) -> Tuple[float, float]:
        """获取障碍物的当前位置，使用锁保证线程安全"""
        with self._lock:
            return self._position
    
    def get_timestamp(self) -> float:
        """获取最后一次位置更新的时间戳"""
        with self._lock:
            return self._timestamp

class ObstacleMonitor:
    """障碍物监控器，负责订阅话题并更新障碍物位置"""
    
    def __init__(self):
        rospy.init_node('obstacle_monitor', anonymous=True)
        
        self.obstacles = {}  # 存储障碍物对象的字典，键为障碍物ID，值为Obstacle对象
        self._subscribers = []  # 存储所有订阅者的列表
        
        # 初始化三个障碍物
        for i in range(1, 4):
            self.obstacles[i] = Obstacle(i)
        
        # 订阅三个话题
        for obstacle_id in self.obstacles:
            topic_name = f"/obstacle_{obstacle_id}/pose"  # 话题名称格式: /obstacle_1/pose, /obstacle_2/pose, ...
            subscriber = rospy.Subscriber(
                topic_name, 
                PoseStamped, 
                callback=self._topic_callback, 
                callback_args=obstacle_id,
                queue_size=10
            )
            self._subscribers.append(subscriber)
            
            rospy.loginfo(f"已订阅话题: {topic_name}")
    
    def _topic_callback(self, msg: PoseStamped, obstacle_id: int):
        """话题回调函数，处理接收到的位置消息"""
        global obs_1, obs_2, obs_3  # 声明使用全局变量
        
        try:
            # 从消息中提取位置数据
            x = msg.pose.position.x
            y = msg.pose.position.y
            position = (x, y)
            
            # 更新障碍物位置
            self.obstacles[obstacle_id].update_position(position)
            rospy.logdebug(f"障碍物 {obstacle_id} 位置更新为: {position}")
            
            # 将位置数据赋值给对应的全局变量
            if obstacle_id == 1:
                obs_1 = position
            elif obstacle_id == 2:
                obs_2 = position
            elif obstacle_id == 3:
                obs_3 = position
                
        except Exception as e:
            rospy.logerr(f"处理障碍物 {obstacle_id} 位置数据时出错: {e}")
    
    def get_all_obstacles(self) -> Dict[int, Obstacle]:
        """获取所有障碍物对象"""
        return self.obstacles

# 使用示例
if __name__ == "__main__":
    try:
        # 创建障碍物监控器实例
        monitor = ObstacleMonitor()
        
        # 设置ROS日志级别
        rospy.loginfo("障碍物监控器已启动，正在实时更新障碍物位置...")
        rospy.loginfo("按 Ctrl+C 停止程序")
        
        # 定期打印障碍物位置
        rate = rospy.Rate(0.5)  # 0.5Hz，即每2秒一次
        while not rospy.is_shutdown():
            print("\n当前障碍物位置:")
            # 方法1: 直接使用全局变量
            print(f"obs_1: ({obs_1[0]:.2f}, {obs_1[1]:.2f})")
            print(f"obs_2: ({obs_2[0]:.2f}, {obs_2[1]:.2f})")
            print(f"obs_3: ({obs_3[0]:.2f}, {obs_3[1]:.2f})")
            
            # 方法2: 从字典中获取
            for obstacle_id, obstacle in monitor.get_all_obstacles().items():
                pos = obstacle.get_position()
                timestamp = obstacle.get_timestamp()
                print(f"障碍物 {obstacle_id}: 位置=({pos[0]:.2f}, {pos[1]:.2f}), "
                      f"最后更新时间={time.ctime(timestamp)}")
            
            # 在这里可以使用全局变量obs_1, obs_2, obs_3进行其他计算
            # 例如避障算法、路径规划等
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        print("\n程序已停止")
    except KeyboardInterrupt:
        print("\n程序已停止")
