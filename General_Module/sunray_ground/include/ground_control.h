#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include "ground_msg.h"
#include "SunrayServer.h"
#include "ros_msg_utils.h"
#include "Communication/TCPServer.h"
#include "Communication/CommunicationUDPSocket.h"
#include "Communication/MSG.h"
#include "Communication/Codec.h"

using namespace std;
class GroundControl {
public:
    GroundControl(){};


     ~GroundControl(){tcpServer.Close();};

    void init(ros::NodeHandle& nh);
    
    

private:
    uint32_t last_time_stamp;
    int uav_num;
    int uav_id;
    string tcp_port;
    int udp_port;
    string uav_name; 
    string tcp_ip; 
    string udp_ip; 
    bool HeartbeatState;//心跳包状态
    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVState uav_state;
    sunray_msgs::UAVControlCMD uav_cmd;

    std::vector<ros::Publisher> control_cmd_pub;
    std::vector<ros::Publisher> uav_setup_pub;
    std::vector<ros::Subscriber> uav_state_sub;


    ros::Timer recvMsgTimer;
    ros::Timer sendMsgTimer;
    ros::Timer HeartbeatTimer;

    TcpServer tcp_server;
    // UDPServer udp_server;

    TCPServer tcpServer;
    CommunicationUDPSocket* udpSocket;
    Codec codec;
    unionData udpData[30];

    std::mutex _mutexUDP; //互斥锁
    std::mutex _mutexTCPServer; //互斥锁

    uint8_t getPX4ModeEnum(std::string modeStr);
    void sendMsgCb(const ros::TimerEvent &e);
    void HeartRate(const ros::TimerEvent &e);
    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id);
    void TCPServerCallBack(ReceivedParameter readData);
    void UDPCallBack(ReceivedParameter readData);

};

