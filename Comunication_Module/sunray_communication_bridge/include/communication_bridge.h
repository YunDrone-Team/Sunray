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
#include "sunray_msgs/UAVState.h"
#include "sunray_msgs/UAVControlCMD.h"
#include "sunray_msgs/Formation.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <unordered_set>
#include <pwd.h>
#include <limits.h>
#include <string>
#include <iostream>
#include <thread>

#define UAVType 0
#define UGVType 1
#define MAX_AGENT_NUM 30

using namespace std;

// Structure to store CPU time statistics
struct CpuData {
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
};


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
    bool is_simulation;
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
    bool station_connected; // 心跳包状态
    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVState uav_state;
    sunray_msgs::UAVControlCMD uav_cmd;

    std::vector<ros::Subscriber> uav_state_sub;
    std::vector<ros::Subscriber> ugv_state_sub;

    std::map<int,ros::Publisher> control_cmd_pub;
    std::map<int,ros::Publisher> uav_setup_pub;
    std::map<int,ros::Publisher> uav_state_pub;
    std::map<int,ros::Publisher> ugv_state_pub;
    std::map<int,ros::Publisher> ugv_controlCMD_pub;
    std::map<int,ros::Publisher> uav_goal_pub;
    std::map<int,ros::Publisher> ugv_goal_pub;


    std::map<int,ros::Publisher> uav_waypoint_pub;
    ros::Subscriber  formation_sub;
    ros::Publisher  formation_pub;

    ros::Timer HeartbeatTimer;
    ros::Timer CheckChildProcessTimer;
    ros::Timer UpdateROSNodeInformationTimer;
    ros::Timer UpdateUDPMulticastTimer;
    ros::Timer UpdateCPUUsageRateTimer;

    CpuData prevData;

    TCPServer tcpServer;
    CommunicationUDPSocket *udpSocket=nullptr;
    Codec codec;
    DataFrame uavStateData[MAX_AGENT_NUM];
    DataFrame ugvStateData[MAX_AGENT_NUM]; 

    DataFrame uavOnlineNodeData[MAX_AGENT_NUM];
    DataFrame ugvOnlineNodeData[MAX_AGENT_NUM]; 

    std::mutex _mutexUDP;       // 互斥锁
    std::mutex _mutexTCPServer; // 互斥锁
    std::mutex _mutexTCPLinkState; // 互斥锁


    std::map<string, pid_t> nodeMap;
    std::unordered_set<std::string> GSIPHash; // 存储所有已连接的IP地址

    std::string getUserDirectoryPath();
    std::string getCurrentProgramPath();
    std::string getSunrayPath();


    uint64_t getCurrentTimestampMs();//本地实现的时间戳生成函数,c++用
    uint64_t calculateMessageDelay(uint64_t msgTimestamp);//计算消息延迟

    uint8_t getPX4ModeEnum(std::string modeStr);
    void sendMsgCb(const ros::TimerEvent &e);
    void sendHeartbeatPacket(const ros::TimerEvent &e);
    void CheckChildProcessCallBack(const ros::TimerEvent &e);
    void UpdateROSNodeInformation(const ros::TimerEvent &e);
    void UpdateComputerStatus(const ros::TimerEvent &e);

    void SendUdpDataToAllOnlineGroundStations(DataFrame data);
    void UpdateUDPMulticast(const ros::TimerEvent &e);


    void uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg, int robot_id);
    void ugv_state_cb(const sunray_msgs::UGVState::ConstPtr &msg, int robot_id);
    void formation_cmd_cb(const sunray_msgs::Formation::ConstPtr &msg);

    void TCPServerCallBack(ReceivedParameter readData);
    void UDPCallBack(ReceivedParameter readData);
    void executiveDemo(std::string orderStr);
    bool SynchronizationUAVState(UAVState Data);
    bool SynchronizationUGVState(UGVState Data);
    void TCPLinkState(bool state,std::string IP);
    pid_t OrderCourse(std::string orderStr);
    pid_t executeScript(std::string scriptStr,std::string filePath);
    pid_t CheckChildProcess(pid_t pid); // 检查子进程是否已经结束


    CpuData readCpuData();// 读取CPU数据
    double calculateCpuUsage(const CpuData& prev, const CpuData& curr);// 计算CPU使用率

    double getMemoryUsage(); // 获取内存使用率
    std::vector<double> getCpuTemperatures(); // 获取CPU温度
};
