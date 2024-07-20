//头文件
#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>

#include <common/mavlink.h>
#include <printf_utils.h>

#include "generic_port.h"
#include "serial_port.h"
#include "udp_port.h"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

using std::string;
using namespace std;

int	system_id    = 0; // system id
int	component_id = 0; // component id

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void timer_heartbeat_cb(const ros::TimerEvent& e, Generic_Port *port)
{
    int	system_id    = 1; // system id
    int	component_id = 1; // component id

    mavlink_message_t message;
    mavlink_heartbeat_t mavlink_heartbeat = { 0 };
    mavlink_heartbeat.custom_mode = 1;
    mavlink_heartbeat.type = 1;
    mavlink_heartbeat.autopilot = 1;
    mavlink_heartbeat.base_mode = 1;
    mavlink_heartbeat.system_status = 1;
    mavlink_heartbeat.mavlink_version = 1;
    mavlink_msg_heartbeat_encode(system_id, component_id, &message, &mavlink_heartbeat);
    // 发送
    int len = port->write_message(message);

    cout << GREEN << "Set mavlink_heartbeat [" << len << "] to FMT successfully!" << TAIL << endl;
}

void timer_vision_position_estimate_cb(const ros::TimerEvent& e, Generic_Port *port)
{
    int	system_id    = 1; // system id
    int	component_id = 1; // component id

    mavlink_message_t message;
    mavlink_vision_position_estimate_t mavlink_vision_position_estimate = { 0 };
    mavlink_vision_position_estimate.x = 1;
    mavlink_vision_position_estimate.y = 2;
    mavlink_vision_position_estimate.z = 3;
    mavlink_vision_position_estimate.roll = 1;
    mavlink_vision_position_estimate.pitch = 2;
    mavlink_vision_position_estimate.yaw = 3;
    mavlink_msg_vision_position_estimate_encode(system_id, component_id, &message, &mavlink_vision_position_estimate);
    // 发送
    int len = port->write_message(message);

    cout << GREEN << "Set mavlink_vision_position_estimate [" << len << "] to FMT successfully!" << TAIL << endl;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_mavlink_to_fmt");
    ros::NodeHandle nh("~");
    ros::Rate rate(10.0);

    bool use_udp = false;
    // 串口参数
    int baudrate, tx_port, rx_port;
    string uart_name, udp_ip;

    nh.param<bool>("use_udp", use_udp, false);
    nh.param<string>("uart_name", uart_name, "/dev/ttyACM0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<int>("rx_port", rx_port, -1);
    nh.param<int>("tx_port", tx_port, -1);

	const char *uart_name_adr = uart_name.c_str();
	const char *udp_ip_adr = udp_ip.c_str();

	Generic_Port *port;
	if(use_udp)
	{
		port = new UDP_Port(udp_ip_adr, rx_port, tx_port);
	}
	else
	{
		port = new Serial_Port(uart_name_adr, baudrate);
	}

	port->start();

    //至此初始化结束，缺省了关闭串口的初始化

    // 定时发送
    ros::Timer timer_heartbeat = nh.createTimer(ros::Duration(1.0), boost::bind(&timer_heartbeat_cb, _1, port));
 
    // #102 VISION_POSITION_ESTIMATE
    ros::Timer timer_vision_position_estimate = nh.createTimer(ros::Duration(0.02), boost::bind(&timer_vision_position_estimate_cb, _1, port));

    // 读取到的消息
    mavlink_message_t message_received;

    while(ros::ok())
    {
        //回调一次 
        ros::spinOnce();

        // 100 us , 0.0001秒
        usleep(100);
    }

    return 0;

}