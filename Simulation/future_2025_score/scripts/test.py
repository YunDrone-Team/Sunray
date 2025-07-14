#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import threading
from typing import Tuple

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
        if not rospy.core.is_initialized():
            rospy.init_node(self._node_name, anonymous=True, disable_signals=True)
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


# 示例：主程序如何使用
if __name__ == '__main__':
    import time

    # 创建更新器对象（后台开始更新全局变量）
    updater = GlobalPoseUpdater()
    
    try:
        # 主程序正常执行，不受后台更新影响
        print("主程序启动，全局变量将在后台自动更新...")
        print("按Ctrl+C停止")
        
        # 主程序循环（示例）
        while True:
            # 主程序逻辑：需要时直接读取全局变量
            with global_data_lock:  # 读取时加锁确保数据一致性
                print(f"\n当前全局变量值:")
                print(f"obs_1: {obs_1}")
                print(f"obs_2: {obs_2}")
                print(f"obs_3: {obs_3}")
                print(f"delivery_target: {delivery_target}")
            
            # 主程序其他工作（模拟）
            time.sleep(2)  # 每2秒打印一次
        
    except KeyboardInterrupt:
        print("\n主程序退出")
    finally:
        updater.stop_background()
