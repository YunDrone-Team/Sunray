#!/usr/bin/env python3
import numpy as np
import os
import time
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class PGMMapGenerator:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pgm_map_generator', anonymous=True)
        
        # 获取ROS参数配置
        self.map_topic = rospy.get_param('~map_topic', '/occupancy_grid')
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/occupancy_grid'))
        self.flip_vertical = rospy.get_param('~flip_vertical', True)  # 垂直翻转地图
        self.gray_scale = rospy.get_param('~gray_scale', False)  # 使用灰度表示概率
        self.timeout = rospy.get_param('~timeout', 10.0)  # 等待地图超时时间(秒)
        
        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        rospy.loginfo(f"PGM地图生成器已启动，保存目录: {self.output_dir}")
        
        # 订阅地图话题
        self.map_received = False
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        
        # 记录程序启动时间
        self.start_time = rospy.Time.now()

    def map_callback(self, msg):
        """处理接收到的地图消息"""
        if self.map_received:
            return  # 已经处理过地图，忽略后续消息
            
        self.map_received = True
        rospy.loginfo(f"已接收到地图，尺寸: {msg.info.width}x{msg.info.height}")
        rospy.loginfo("开始生成PGM地图...")
        start_time = time.time()
        
        try:
            # 提取地图元数据
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin
            
            # 转换地图数据
            grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
            
            # 垂直翻转地图 (ROS地图原点在左下角，图像原点在左上角)
            if self.flip_vertical:
                grid = np.flipud(grid)
            
            # 转换为PGM图像数据
            if self.gray_scale:
                # 使用灰度表示占用概率
                pgm_map = np.ones_like(grid, dtype=np.uint8) * 205  # 默认未知区域
                pgm_map[grid >= 0] = 255 - np.clip(grid[grid >= 0], 0, 100) * 2.55  # 0-100 -> 255-0
            else:
                # 二值化表示
                pgm_map = np.ones_like(grid, dtype=np.uint8) * 205  # 未知区域
                pgm_map[grid == 0] = 255    # 空闲区域
                pgm_map[grid >= 65] = 0     # 占用区域 (阈值65%)
            
            # 生成带时间戳的文件名
            timestamp = rospy.Time.now().to_sec()
            file_prefix = f'map_{timestamp:.0f}'
            pgm_filename = os.path.join(self.output_dir, f'{file_prefix}.pgm')
            yaml_filename = os.path.join(self.output_dir, f'{file_prefix}.yaml')
            
            # 保存PGM和YAML文件
            self.save_pgm(pgm_filename, pgm_map)
            self.save_yaml(yaml_filename, pgm_filename, width, height, resolution, origin)
            
            elapsed = time.time() - start_time
            rospy.loginfo(f"地图保存成功: {pgm_filename} (耗时: {elapsed:.2f}s)")
            
            # 关闭节点
            rospy.signal_shutdown("地图已生成，程序正常退出")
            
        except Exception as e:
            rospy.logerr(f"地图生成失败: {str(e)}")
            rospy.signal_shutdown(f"错误: {str(e)}")

    def save_pgm(self, filename, data):
        """保存为PGM格式文件"""
        height, width = data.shape
        
        with open(filename, 'wb') as f:
            # 写入PGM文件头 (P5表示二进制灰度图像)
            f.write(f"P5\n{width} {height}\n255\n".encode())
            
            # 写入图像数据
            data.tofile(f)

    def save_yaml(self, filename, pgm_file, width, height, resolution, origin):
        """保存YAML配置文件"""
        # 提取原点位置和方向
        position = origin.position
        
        # 计算地图物理尺寸
        real_width = width * resolution
        real_height = height * resolution
        
        with open(filename, 'w') as f:
            f.write(f"image: {os.path.basename(pgm_file)}\n")
            f.write(f"resolution: {resolution}\n")
            f.write(f"origin: [{position.x}, {position.y}, 0.0]\n")
            f.write(f"occupied_thresh: 0.65\n")
            f.write(f"free_thresh: 0.196\n")
            f.write(f"negate: 0\n")
            f.write(f"# 地图尺寸: {width}x{height} pixels ({real_width:.2f}m x {real_height:.2f}m)\n")

    def wait_for_map(self):
        """等待地图消息，超时则退出"""
        rospy.loginfo(f"等待来自 {self.map_topic} 的地图消息...")
        
        while not self.map_received and not rospy.is_shutdown():
            if (rospy.Time.now() - self.start_time).to_sec() > self.timeout:
                rospy.logerr(f"等待地图超时 ({self.timeout}秒)")
                rospy.signal_shutdown("等待地图超时")
                return False
                
            rospy.sleep(0.1)
            
        return self.map_received

if __name__ == '__main__':
    try:
        generator = PGMMapGenerator()
        
        # 等待地图消息
        if generator.wait_for_map():
            rospy.spin()  # 保持节点运行直到地图生成完成
        else:
            rospy.logerr("未能接收到地图数据")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("PGM地图生成器被中断")
    except Exception as e:
        rospy.logerr(f"发生未知错误: {str(e)}")
        raise