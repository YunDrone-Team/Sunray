#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include "ros_msg_utils.h"
#include "Communication/TCPServer.h"
#include "Communication/CommunicationUDPSocket.h"
#include "Communication/MSG.h"
#include "Communication/Codec.h"
#include "sunray_msgs/UGVState.h"
#include "sunray_msgs/UGVControlCMD.h"


#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <unordered_set>
#include <pwd.h>
#include <limits.h>

#define UAVType 0
#define UGVType 1
#define MAX_AGENT_NUM 30

using namespace std;
class communication_bridge
{
public:
    communication_bridge() {};

    ~communication_bridge()
    {
        tcpServer.Close();
        for (auto it = nodeMap.begin(); it != nodeMap.end(); ++it)
        {
            std::cout << "Key: " << it->first << ", Value: " << it->second << std::endl;
            pid_t temp = it->second;
            if(temp<= 0)
                continue;
            if (kill(temp, SIGTERM) != 0)
                perror("kill failed!");
            else
                printf("Sent SIGTERM to child process %d\n", temp);
            
        }
        nodeMap.clear();
    };

    void init(ros::NodeHandle &nh);

private:
    uint32_t last_time_stamp;
    int uav_experiment_num;
    int ugv_experiment_num;
    int uav_simulation_num;
    int uav_id;
    int ugv_id;
    int ugv_simulation_num;

    string tcp_port;
    int udp_port;
    int udp_ground_port;

    string uav_name;
    string ugv_name;

    string tcp_ip;
    string udp_ip;
    pid_t demoPID = -1;
    bool HeartbeatState; // 心跳包状态
    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVState uav_state;
    sunray_msgs::UAVControlCMD uav_cmd;

    std::vector<ros::Publisher> control_cmd_pub;
    std::vector<ros::Publisher> uav_setup_pub;
    std::vector<ros::Subscriber> uav_state_sub;
    std::vector<ros::Subscriber> ugv_state_sub;
    std::map<int,ros::Publisher> uav_state_pub;
    std::map<int,ros::Publisher> ugv_state_pub;
    std::map<int,ros::Publisher> ugv_controlCMD_pub;

    std::vector<ros::Publisher> uav_waypoint_pub;


    ros::Timer recvMsgTimer;
    ros::Timer sendMsgTimer;
    ros::Timer HeartbeatTimer;
    ros::Timer CheckChildProcessTimer;
    ros::Timer InterAircraftTimer;
    ros::Timer SendGroundStationDataTimer;
    ros::Timer InterVehicleTimer;

    // UDPServer udp_server;

    TCPServer tcpServer;
    CommunicationUDPSocket *udpSocket;
    Codec codec;
    unionData uavStateData[MAX_AGENT_NUM];
    unionData ugvStateData[MAX_AGENT_NUM]; 

    std::mutex _mutexUDP;       // 互斥锁
    std::mutex _mutexTCPServer; // 互斥锁
    std::mutex _mutexTCPLinkState; // 互斥锁


    std::map<string, pid_t> nodeMap;
    std::unordered_set<std::string> GSIPHash; // 存储所有已连接的IP地址

    std::string getUserDirectoryPath();
    std::string getCurrentProgramPath();
    std::string getSunrayPath();

    uint8_t getPX4ModeEnum(std::string modeStr);
    void sendMsgCb(const ros::TimerEvent &e);
    void sendHeartbeatPacket(const ros::TimerEvent &e);
    void CheckChildProcessCallBack(const ros::TimerEvent &e);
    void sendInterAircraftStatusInformation(const ros::TimerEvent &e);
    void sendGroundStationData (const ros::TimerEvent &e);
    void sendInterVehicleStatusInformation(const ros::TimerEvent &e);

    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id);
    void ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int robot_id);

    void TCPServerCallBack(ReceivedParameter readData);
    void UDPCallBack(ReceivedParameter readData);
    void executiveDemo(std::string orderStr);
    bool SynchronizationUAVState(StateData Data);
    bool SynchronizationUGVState(UGVStateData Data);
    bool PublishUGVControlTopic(UGVControlData Data);
    void TCPLinkState(bool state,std::string IP);
    pid_t OrderCourse(std::string orderStr);
    pid_t executeScript(std::string scriptStr,std::string filePath);
    pid_t CheckChildProcess(pid_t pid); // 检查子进程是否已经结束
};
