#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <boost/asio.hpp>
#include <cmath>
#include "c_library_v2/common/mavlink.h"
#include "c_library_v2/common/common.h"

boost::asio::io_service io_s;
boost::asio::ip::udp::socket udp_socket(io_s);
boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 14581); // adjust IP and port accordingly

double position_x = 0.0; // Initialize position_x
double position_y = 0.0; // Initialize position_y
double position_z = 0.0; // Initialize position_z

void trajector_pub()
{
    mavlink_message_t trajectory_msg;
    mavlink_trajectory_representation_bezier_t trajectory_target;
    
    trajectory_target.time_usec = ros::Time::now().toNSec() / 1000; // Convert ros::Time to microseconds

    // set Bezier control points
    trajectory_target.pos_x[0] = position_x;
    trajectory_target.pos_y[0] = position_y;
    trajectory_target.pos_z[0] = position_z;
    trajectory_target.pos_x[1] = position_x + 1;
    trajectory_target.pos_y[1] = position_y + 1;
    trajectory_target.pos_z[1] = position_z + 1;
    trajectory_target.pos_x[2] = position_x + 2;
    trajectory_target.pos_y[2] = position_y + 2;
    trajectory_target.pos_z[2] = position_z + 2;

    // set Bezier time horizon (you can adjust these values accordingly)
    trajectory_target.delta[0] = 1.0;
    trajectory_target.delta[1] = 2.0;
    trajectory_target.delta[2] = 3.0;

    // set yaw angles (you can adjust these values accordingly)
    trajectory_target.pos_yaw[0] = 0.0;
    trajectory_target.pos_yaw[1] = 0.5;
    trajectory_target.pos_yaw[2] = 1.0;

    for (int i = 3; i < 5; i++) {
        trajectory_target.pos_x[i] = NAN;
        trajectory_target.pos_y[i] = NAN;
        trajectory_target.pos_z[i] = NAN;
        trajectory_target.pos_yaw[i] = NAN;
        trajectory_target.delta[i] = NAN;
    }

    // set number of valid control points
    trajectory_target.valid_points = 3;

    uint16_t mas_len = mavlink_msg_trajectory_representation_bezier_encode(1, 84, &trajectory_msg,  &trajectory_target);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &trajectory_msg); // Fix typo: tag_msg -> trajectory_msg
    
    size_t bytes_sent = udp_socket.send_to(boost::asio::buffer(buffer, len), endpoint);
    if (bytes_sent == len) {
        std::cout << "Message sent successfully!" << std::endl;
    } else {
        std::cerr << "Error sending message: " << boost::system::error_code(errno, boost::system::generic_category()).message() << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajector_pub");
    ros::NodeHandle nd;
    ros::Rate loop_rate(10);
    udp_socket.open(boost::asio::ip::udp::v4());

    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        trajector_pub();
    }
    udp_socket.close();

    return 0;
}