#!/usr/bin/env python

import rospy
import socket
import struct
import pickle
from threading import Thread

class ROSMulticastBridge:
	def __init__(self):
		# ROS节点初始化
		rospy.init_node('ros_multicast_bridge', anonymous=True)
		
		# 获取参数
		self.multicast_group = rospy.get_param('~multicast_group', '224.0.0.1')
		self.multicast_port = rospy.get_param('~multicast_port', 8989)
		self.config_file = rospy.get_param('~config_file', "/home/yundrone/Sunray/sunray_formation/formation_control/scrpts/pub.yaml")
		self.ttl = rospy.get_param('~ttl', 2)  # Time To Live for multicast packets
		
		# 设置组播socket
		self.setup_multicast_sender()
		
		# 根据消息类型动态导入
		self.publishers = {}
		self.message_type_map = {}
		self.import_message_types_from_config(self.config_file)
		

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
				pub_name = msg_config.get('pub')
				
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
					self.message_type_map[topic_name] = msg_class
					self.publishers[topic_name] = rospy.Publisher(pub_name, msg_class, queue_size=10)
					rospy.loginfo(f'Successfully mapped topic {topic_name} to message {message_type}')
					
				except Exception as e:
					rospy.logerr(f"Failed to import message type {message_type} for topic {topic_name}: {str(e)}")
					continue
					
			return self.message_type_map
			
		except Exception as e:
			rospy.logerr(f"Failed to load message types from config {config_file}: {str(e)}")
			raise
	
	def setup_multicast_sender(self):
		# 设置接收socket
		self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.recv_sock.bind(('', self.multicast_port))
		
		# 加入组播组
		mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
		self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
	
	def receive_multicast_messages(self):
		"""接收组播消息并发布到ROS话题"""
		while not rospy.is_shutdown():
			try:
				data, _ = self.recv_sock.recvfrom(65536)  # 最大接收64KB
				msg = pickle.loads(data)
				topic_name = msg[0]
				# 检查消息类型是否正确
				if isinstance(msg[2], self.message_type_map[topic_name]):
					# 发布到ROS话题
					self.publishers[topic_name].publish(msg[2])
			except Exception as e:
				rospy.logerr(f"Error receiving multicast message: {str(e)}")
	
	def shutdown(self):
		"""清理资源"""
		self.recv_sock.close()

if __name__ == '__main__':
	try:
		bridge = ROSMulticastBridge()
		bridge.receive_multicast_messages()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
	finally:
		bridge.shutdown()