// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#endif

#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <cmath>
#include "src/ord_lidar_driver.h"
#include <sys/time.h>

using namespace std;
using namespace ordlidar;

#define Degree2Rad(X) ((X)*M_PI / 180.)
#ifdef ROS_FOUND
void publish_msg(ros::Publisher *pub, full_scan_data_st *scan_frame, ros::Time start,
                 double scan_time, std::string frame_id, bool clockwise,
                 double angle_min, double angle_max, double min_range, double max_range)
{
  sensor_msgs::PointCloud pcMsg;
  int point_nums = scan_frame->vailtidy_point_num;
  float range = 0.0, intensity = 0.0, angle = 0.0, rad = 0.0;

  pcMsg.header.stamp = start;
  pcMsg.header.frame_id = frame_id;
  pcMsg.points.resize(point_nums);

  // add an intensity channel to the cloud
  pcMsg.channels.resize(1);
  pcMsg.channels[0].name = "intensities";
  pcMsg.channels[0].values.resize(point_nums);

  for (int i = 0; i < point_nums; i++)
  {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;
    if (!clockwise)
      angle = static_cast<float>(360.f - scan_frame->data[i].angle);
    else
      angle = scan_frame->data[i].angle;

    rad = Degree2Rad(angle);
	
    if ((angle >= angle_min) && (angle <= angle_max))
    {
      if ((range >= min_range) && (range <= max_range))
      {
        pcMsg.points[i].x = range * cos(rad);
        pcMsg.points[i].y = range * sin(rad);
        pcMsg.points[i].z = 0.0;
        pcMsg.channels[0].values[i] = intensity;
      }
    }
  }

  pub->publish(pcMsg);
}

#elif ROS2_FOUND
void publish_msg(rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr &pub, full_scan_data_st *scan_frame, rclcpp::Time start,
                 double scan_time, std::string frame_id, bool clockwise,
                 double angle_min, double angle_max, double min_range, double max_range)
{
  sensor_msgs::msg::PointCloud pcMsg;
  int point_nums = scan_frame->vailtidy_point_num;
  float range = 0.0, intensity = 0.0, angle = 0.0, rad = 0.0;

  pcMsg.header.stamp = start;
  pcMsg.header.frame_id = frame_id;
  pcMsg.points.resize(point_nums);

  // add an intensity channel to the cloud
  pcMsg.channels.resize(1);
  pcMsg.channels[0].name = "intensities";
  pcMsg.channels[0].values.resize(point_nums);

  for (int i = 0; i < point_nums; i++)
  {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;
    if (!clockwise)
      angle = static_cast<float>(360.f - scan_frame->data[i].angle);
    else
      angle = scan_frame->data[i].angle;
      
    rad = Degree2Rad(angle);

    if ((angle >= angle_min) && (angle <= angle_max))
    {
      if ((range >= min_range) && (range <= max_range))
      {
        pcMsg.points[i].x = range * cos(rad);
        pcMsg.points[i].y = range * sin(rad);
        pcMsg.points[i].z = 0.0;
        pcMsg.channels[0].values[i] = intensity;
      }
    }
  }

  pub->publish(pcMsg);
}
#endif

int main(int argc, char **argv)
{
  std::string frame_id, cloud_topic;
  std::string port;
  std::string device_model;

  double min_thr = 0.0, max_thr = 0.0, cur_speed = 0.0;
  int baudrate = 230400;
  int motor_speed = 10;
  double angle_min = 0.0, angle_max = 360.0;
  double min_range = 0.05, max_range = 20.0;
  bool clockwise = false;
  uint8_t type = ORADAR_TYPE_SERIAL;
  int model = ORADAR_MS200;
#ifdef ROS_FOUND
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("port_name", port, "/dev/ttyUSB0");
  nh_private.param<int>("baudrate", baudrate, 230400);
  nh_private.param<double>("angle_max", angle_max, 180.00);
  nh_private.param<double>("angle_min", angle_min, -180.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.05);
  nh_private.param<bool>("clockwise", clockwise, false);
  nh_private.param<int>("motor_speed", motor_speed, 10);
  nh_private.param<std::string>("device_model", device_model, "ms200");
  nh_private.param<std::string>("frame_id", frame_id, "pc_frame");
  nh_private.param<std::string>("cloud_topic", cloud_topic, "point_cloud");
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>(cloud_topic, 50);
  #elif ROS2_FOUND
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("oradar_ros"); // create a ROS2 Node

    // declare ros2 param
  node->declare_parameter<std::string>("port_name", port);
  node->declare_parameter<int>("baudrate", baudrate);
  node->declare_parameter<double>("angle_max", angle_max);
  node->declare_parameter<double>("angle_min", angle_min);
  node->declare_parameter<double>("range_max", max_range);
  node->declare_parameter<double>("range_min", min_range);
  node->declare_parameter<bool>("clockwise", clockwise);
  node->declare_parameter<int>("motor_speed", motor_speed);
  node->declare_parameter<std::string>("device_model", device_model);
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->declare_parameter<std::string>("cloud_topic", cloud_topic);

  // get ros2 param
  node->get_parameter("port_name", port);
  node->get_parameter("baudrate", baudrate);
  node->get_parameter("angle_max", angle_max);
  node->get_parameter("angle_min", angle_min);
  node->get_parameter("range_max", max_range);
  node->get_parameter("range_min", min_range);
  node->get_parameter("clockwise", clockwise);
  node->get_parameter("motor_speed", motor_speed);
  node->get_parameter("device_model", device_model);
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("cloud_topic", cloud_topic);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::PointCloud>(cloud_topic, 10);
  #endif

  OrdlidarDriver device(type, model);
  bool ret = false;

  if (port.empty())
  {
    std::cout << "can't find lidar ms200" << std::endl;
  }
  else
  {
    device.SetSerialPort(port, baudrate);

    std::cout << "get lidar type:"  << device_model.c_str() << std::endl;
    std::cout << "get serial port:"  << port.c_str() << ", baudrate:"  << baudrate << std::endl;
    #ifdef ROS_FOUND
    while (ros::ok())
    #elif ROS2_FOUND
    while (rclcpp::ok())
    #endif
    {
      if (device.isConnected() == true)
      {
        device.Disconnect();
        std::cout << "Disconnect lidar device." << std::endl;
      }

      if (device.Connect())
      {
        std::cout << "lidar device connect succuss." << std::endl;
        break;
      }
      else
      {
        std::cout << "lidar device connecting..." << std::endl;
        sleep(1);
      }
    }

    full_scan_data_st scan_data;
    #ifdef ROS_FOUND
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    #elif ROS2_FOUND
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    #endif
    double scan_duration;

    std::cout << "get lidar scan data" << std::endl;
    std::cout << "ROS topic:" << cloud_topic.c_str() << std::endl;
    
		min_thr = (double)motor_speed - ((double)motor_speed  * 0.1);
		max_thr = (double)motor_speed + ((double)motor_speed  * 0.1);
    cur_speed = device.GetRotationSpeed();
    if(cur_speed < min_thr || cur_speed > max_thr)
    {
      device.SetRotationSpeed(motor_speed);
    }
    

    #ifdef ROS_FOUND
    while (ros::ok())
    #elif ROS2_FOUND
    while (rclcpp::ok())
    #endif
    {
      #ifdef ROS_FOUND
      start_scan_time = ros::Time::now();
      #elif ROS2_FOUND
      start_scan_time = node->now();
      #endif
      ret = device.GrabFullScanBlocking(scan_data, 1000);
      #ifdef ROS_FOUND
      end_scan_time = ros::Time::now();
      scan_duration = (end_scan_time - start_scan_time).toSec();
      #elif ROS2_FOUND
      end_scan_time = node->now();
      scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
      #endif
      

      
      if (ret)
      {
        #ifdef ROS_FOUND
        publish_msg(&cloud_pub, &scan_data, start_scan_time, scan_duration, frame_id,
                    clockwise, angle_min, angle_max, min_range, max_range);
        #elif ROS2_FOUND
        publish_msg(publisher, &scan_data, start_scan_time, scan_duration, frame_id,
            clockwise, angle_min, angle_max, min_range, max_range);
        #endif

      }
    }

    device.Disconnect();
    
  }

  std::cout << "publish node end.." << std::endl;
  #ifdef ROS_FOUND
  ros::shutdown();
  #elif ROS2_FOUND
  rclcpp::shutdown();
  #endif
  
  return 0;
}
