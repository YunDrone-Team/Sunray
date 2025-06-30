#include "data_convert.hpp"
#include "mavlink_control.h"

void pose_callback(const nav_msgs::OdometryPtr &msg){
    mavlink_odometry_t mavlink_odom;
    odometry_msg_convert(msg,mavlink_odom);
    mavlink_save_odometry(mavlink_odom); 
    // mavlink_send_odometry(mavlink_odom);  //直接接收到数据就用串口发送出去  
}

int main(int argc, char **argv){
	ros::init(argc, argv, "mavLink");
    ros::NodeHandle nh("~");
    std::string uart_name;
    int baudrate = 57600;
    nh.param<string>("uart_name", uart_name, "/dev/ttyS0");
    nh.param<int>("baudrate", baudrate, 57600);
    mavlink_init(uart_name.c_str(), baudrate);
    // mavlink_control();
    mavlink_send_odometry_thread();//先保存再在另外的线程里面发送
    ros::Subscriber sub_stereo3_pose = nh.subscribe("/baton/loop/odometry", 2, pose_callback); 
    ros::spin();
    mavlink_deinit();
    return 0;
}
