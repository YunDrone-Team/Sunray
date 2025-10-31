#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'pc_frame'},
        {'cloud_topic': 'MS200/point_cloud'},
        {'port_name': '/dev/ttyUSB0'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''

def generate_launch_description():
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      executable='oradar_pointcloud',
      name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'pc_frame'},
        {'cloud_topic': 'MS200/point_cloud'},
        {'port_name': '/dev/ttyUSB0'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  # base_link to pc_frame tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser',
    arguments=['0','0','0.18','0','0','0','base_link','pc_frame']
  )


  # Define LaunchDescription variable
  ord = LaunchDescription()

  ord.add_action(ordlidar_node)
  ord.add_action(base_link_to_laser_tf_node)

  return ord