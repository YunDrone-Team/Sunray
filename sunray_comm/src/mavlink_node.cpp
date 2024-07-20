#include <ros/ros.h>

#include "mavlink_receiver.h"
#include "mavlink_sender.h"

#include <signal.h>
using std::string;
using namespace std;

void mySigintHandler(int sig)
{
    ROS_INFO("[mavlink_node] exit...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavlink_node");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    bool use_udp = false;
    // 串口参数
    int baudrate, tx_port, rx_port;
    string uart_name, udp_ip;

    string uav_name;
    int uav_id;
    bool flag_printf;
    string px4_or_fmt;

    nh.param<string>("px4_or_fmt", px4_or_fmt, "px4");
    nh.param<bool>("use_udp", use_udp, false);
    nh.param<string>("uart_name", uart_name, "/dev/ttyACM0");
    nh.param<int>("baudrate", baudrate, 115200);
    nh.param<string>("udp_ip", udp_ip, "127.0.0.1");
    nh.param<int>("rx_port", rx_port, -1);
    nh.param<int>("tx_port", tx_port, -1);
    nh.param<bool>("flag_printf", flag_printf, false);



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

    // MAVLink消息接收器
    MavlinkReceiver mavlink_receiver; 
    mavlink_receiver.init(nh, port);            // 初始化设置端口

    if(px4_or_fmt == "fmt")
    {
        // 设置Mavlink消息接收频率（FMT生效，PX4待测试）
        mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_HEARTBEAT, 1);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_SYS_STATUS, 5);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_GPS_RAW_INT, 10);                
        mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_ATTITUDE, 100);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 100);
        mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_RC_CHANNELS, 20);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_ATTITUDE_TARGET, 10);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 10);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_HIGHRES_IMU, 50);
        // mavlink_receiver.set_mavlink_msg_frequency(MAVLINK_MSG_ID_DISTANCE_SENSOR, 10);
    }
    else if (px4_or_fmt == "px4")
    {
        /* code */
    }

    // MAVLink消息发送器
    MavlinkSender mavlink_sender;
    mavlink_sender.init(nh, port);            // 初始化设置端口

    mavlink_message_t message_received;

    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec - 10;

    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        //cout << RED << "Start mavlink_receiver" << TAIL << endl;

        // 延时测试函数

        // 接收Mavlink消息主循环函数
        bool success = port->read_message(message_received);

		if( success )
		{
            // cout << GREEN << "Got MAVLink MSG!!" << TAIL << endl;
            // 处理Mavlink消息
            mavlink_receiver.handle_msg(&message_received);
        }else
        {
            // printf("No message !!! \n");
        }

        //cout << RED << "END mavlink_receiver" << TAIL << endl;

        // 10 us , 0.00001秒
        // 10000 us, 0.01秒
        if(px4_or_fmt == "fmt")
        {
            usleep(10);
        }
        else if (px4_or_fmt == "px4")
        {
            /* code */
        }

        // 定时状态打印
        time_now = ros::Time::now();
        if ((time_now - time_last).toSec() > 1.0 /* 秒 */ && flag_printf)
        {
            mavlink_receiver.printf_debug_info();
            mavlink_sender.printf_debug_info();
            time_last = time_now;
        }
    }

    return 0;
}
