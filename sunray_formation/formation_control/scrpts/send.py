#!/usr/bin/env python

import rospy
import socket
import struct
import pickle
from threading import Thread

class Msg:
	def __init__(self, target_type=str):
		self.name = None
		self.type = None
		self._inner_obj = None
		self.target_type = target_type

	@property 
	def inner_obj(self):
		return self._inner_obj

	@inner_obj.setter
	def inner_obj(self, value):
		"""赋值时自动转换成 target_type"""
		try:
			self._inner_obj = self.target_type(value)  # 尝试转换
		except (TypeError, ValueError) as e:
			raise ValueError(f"Cannot convert {value} to {self.target_type}: {e}")
		
class ROSMulticastBridge:
	def __init__(self):
		# ROS节点初始化
		rospy.init_node('ros_multicast_bridge', anonymous=True)
		
		# 获取参数
		self.message_type = rospy.get_param('~message_type', 'sunray_msgs/UGVState')
		self.multicast_group = rospy.get_param('~multicast_group', '224.0.0.1')
		self.multicast_port = rospy.get_param('~multicast_port', 8989)
		self.config_file = rospy.get_param('~config_file', "/home/yundrone/Sunray/sunray_formation/formation_control/scrpts/sub.yaml")
		self.ttl = rospy.get_param('~ttl', 2)  # Time To Live for multicast packets
		
		# 设置组播socket
		self.setup_multicast_sender()

		# 订阅ROS话题
		self.subscribers = []
		# self.subscriber = rospy.Subscriber(self.topic_name, self.message_class, self.ros_callback)

		# 初始化存储结构 {topic_name: {'type': msg_type, 'class': msg_class}}
		self.topic_message_map = {}
		# 根据消息类型动态导入
		self.import_message_types_from_config(self.config_file)
		self.msg_data = Msg()
		


	def import_message_types_from_config(self, config_file):
		"""从配置文件动态导入ROS消息类型，以话题名作为唯一索引"""
		try:
			import yaml
			
			# 读取配置文件
			with open(config_file, 'r') as f:
				config = yaml.safe_load(f)
			
			# 获取消息列表配置
			message_configs = config.get('messages', [])
			
			for msg_config in message_configs:
				message_type = msg_config.get('type')
				topic_name = msg_config.get('topic')
				
				if not message_type:
					rospy.logwarn(f"Missing message type in config: {msg_config}")
					continue
					
				if not topic_name:
					rospy.logwarn(f"Missing topic name for message type: {message_type}")
					continue
					
				try:
					# 分割消息类型为包名和类型名
					package, msg_type = message_type.split('/')
					rospy.loginfo(f'Importing {message_type} for topic {topic_name}')
					
					# 动态导入
					module = __import__(f'{package}.msg', fromlist=[msg_type])
					msg_class = getattr(module, msg_type)
					
					# 存储到映射表
					self.topic_message_map[topic_name] = {
						'type': message_type,
						'class': msg_class
					}
					self.subscribers.append(rospy.Subscriber(topic_name, msg_class, self.ros_callback, callback_args=topic_name))
					rospy.loginfo(f'Successfully mapped topic {topic_name} to message {message_type}')
					
				except Exception as e:
					rospy.logerr(f"Failed to import message type {message_type} for topic {topic_name}: {str(e)}")
					continue
					
			return self.topic_message_map
			
		except Exception as e:
			rospy.logerr(f"Failed to load message types from config {config_file}: {str(e)}")
			raise


	def setup_multicast_sender(self):
		"""设置组播发送socket"""
		self.multicast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.multicast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		# self.multicast_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.ttl)
 
	def ros_callback(self, msg, name):
		"""ROS话题回调函数，将消息发送到组播（包含topic_name和msg_type）"""
		try:
			topic_name = name
			msg_type = msg._type

			# 序列化 (topic_name, msg_type, msg) 为一个元组
			data_tuple = (topic_name, msg_type, msg)
			serialized_data = pickle.dumps(data_tuple)
			
			# 发送到组播组
			self.multicast_sock.sendto(serialized_data, (self.multicast_group, self.multicast_port))
			
		except Exception as e:
			rospy.logerr(f"Error sending message via multicast: {str(e)}")
	
	
	def shutdown(self):
		"""清理资源"""
		try:
			self.multicast_sock.close()
		except Exception as e:
			pass

if __name__ == '__main__':
	try:
		bridge = ROSMulticastBridge()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	finally:
		bridge.shutdown()