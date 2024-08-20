#include <string>
#include <iostream>
#include <thread>
#include "ground_msg.h"
#include "SunrayServer.h"
#include "ros_msg_utils.h"

using namespace std;
class GroundControl {
public:
    GroundControl(){};

    void init(ros::NodeHandle& nh);
    void parseTcpMessage(char *message);
    
    

private:
    uint32_t last_time_stamp;
    int uav_num;
    string tcp_port;
    string udp_port;
    string uav_name; 
    string tcp_ip; 
    string udp_ip; 
    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVState uav_state;
    sunray_msgs::UAVControlCMD uav_cmd;

    std::vector<ros::Publisher> control_cmd_pub;
    std::vector<ros::Publisher> uav_setup_pub;
    std::vector<ros::Subscriber> uav_state_sub;

    std::vector<State_Message> stateMessage;

    ros::Timer recvMsgTimer;
    ros::Timer sendMsgTimer;

    TcpServer tcp_server;
    UDPServer udp_server;

    void recvMsgCb(const ros::TimerEvent &e);
    void sendMsgCb(const ros::TimerEvent &e);
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id);
};

