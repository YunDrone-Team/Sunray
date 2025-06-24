#ifndef MSG_H
#define MSG_H

#include <cstdint>
#include <string.h>
#include <vector>
#include <string>
#include "MessageEnums.h"

//心跳包 - HeartbeatData（#1）
struct HeartbeatData
{
    uint8_t agentType;
    int32_t count;       /**< @param head message header 心跳包计数，未启用 */

    void init()
    {
        count=0;
        agentType=0;
    }
};

//无人机状态 - UAVState（#2）
struct UAVState
{
    uint8_t uav_id;
    bool    connected;
    bool    armed;
    uint8_t mode;
    uint8_t landed_state;
    float   battery_state;
    float   battery_percentage;
    uint8_t location_source;
    bool    odom_valid;
    float   position[3];
    float   velocity[3];
    float   attitude[3];
    float   attitude_q[4];
    float   attitude_rate[3];
    float   pos_setpoint[3];
    float   vel_setpoint[3];
    float   att_setpoint [3];
    float   thrust_setpoint;
    uint8_t control_mode;
    uint8_t move_mode;
    float   takeoff_height;
    float   home_pos[3];
    float   home_yaw;
    float   hover_pos[3];
    float   hover_yaw;
    float   land_pos[3];
    float   land_yaw;

    void init()
    {
        uav_id=0;
        connected=0;
        armed=0;
        mode=0;
        landed_state=0;
        battery_state=0;
        battery_percentage=0;
        location_source=0;
        odom_valid=0;
        for(int i=0;i<3;++i)
        {
            position[i]=0;
            velocity [i]=0;
            attitude [i]=0;
            attitude_q[i]=0;
            attitude_rate [i]=0;
            pos_setpoint [i]=0;
            vel_setpoint[i]=0;
            att_setpoint [i]=0;
            home_pos[i]=0;
            hover_pos[i]=0;
            land_pos[i]=0;
        }
        attitude_q[3]=0;
        thrust_setpoint=0;
        control_mode=0;
        move_mode=0;

        takeoff_height=0;
        home_yaw=0;
        hover_yaw=0;
        land_yaw=0;
    }

};

//无人车状态 - UGVState（#20）
struct UGVState
{

   uint8_t  ugv_id;
   bool     connected;
   float    battery_state;
   float    battery_percentage;
   uint8_t  location_source;
   bool     odom_valid;
   float    position[2];
   float    velocity[2];
   float    yaw;
   float    attitude[3];
   float    attitude_q[4];
   uint8_t  control_mode;
   float    pos_setpoint[2];
   float    vel_setpoint[2];
   float    yaw_setpoint;
   float    home_pos[2];
   float    home_yaw;
   float    hover_pos[2];
   float    hover_yaw;

   void init()
   {
       ugv_id=0;
       connected=0;
       battery_state=0;
       battery_percentage=0;
       location_source=0;
       odom_valid=0;
       yaw=0;
       control_mode=0;
       yaw_setpoint=0;
       home_yaw=0;
       hover_yaw=0;

       for(int i=0;i<2;++i)
       {
           position[i]=0;
           velocity [i]=0;
           attitude[i]=0;
           attitude_q[i]=0;
           pos_setpoint[i]=0;
           vel_setpoint[i]=0;
           home_pos[i]=0;
           hover_pos[i]=0;
       }
       attitude[2]=0;
       attitude_q[2]=0;
       attitude_q[3]=0;
   }

};

//机载电脑ROS节点 - NodeData（#30）
struct NodeData
{
    uint16_t nodeCount;
    uint16_t nodeID;
    uint16_t nodeSize;
    char nodeStr[300];
    void init()
    {

        nodeCount=0;
        nodeID=0;
        nodeSize=0;
    }
};

//智能体电脑状态 -AgentComputerStatus（#31）
struct AgentComputerStatus
{
    double cpuLoad;         // CPU 负载百分比（0.0-100.0）
    double memoryUsage;     // 内存使用率（0.0-100.0）
    double cpuTemperature;  // CPU 温度（摄氏度，保留小数精度）
    void init()
    {
        cpuLoad=0;
        memoryUsage=0;
        cpuTemperature=0;
    }

};

//编队切换 - Formation（#40）
struct Formation
{
    uint8_t cmd;
    uint8_t formation_type;
    uint8_t leader_id;
    uint16_t nameSize;
    char name[300];
    void init()
    {
       leader_id=0;
       cmd=0;
       formation_type=0;
       nameSize=0;
    }
};

//无人机控制指令 - UAVControlCMD（#102）
struct UAVControlCMD
{
    uint8_t cmd;
    float desired_pos[3];
    float desired_vel[3];
    float desired_acc[3];
    float desired_jerk[3];
    float desired_att[3];
    float desired_thrust;
    float desired_yaw;
    float desired_yaw_rate;
    float latitude;
    float longitude;
    float altitude;

    void init()
    {
        cmd=0;
        for(int i=0;i<3;++i)
        {
            desired_pos[i]=0;
            desired_vel[i]=0;
            desired_acc[i]=0;
            desired_jerk[i]=0;
            desired_att[i]=0;
        }
        desired_thrust=0;
        desired_yaw=0;
        desired_yaw_rate=0;
        latitude=0;
        longitude=0;
        altitude=0;
    }

};

//无人机设置指令 - UAVSetup（#103）
struct UAVSetup
{
    uint8_t cmd;
    uint8_t px4_mode;
    uint8_t control_mode;

    void init()
    {
        cmd=0;
        px4_mode=0;
        control_mode=0;
    }
};

//航点 - WaypointData（#104）
struct WaypointData
{
    uint8_t wp_num;
    uint8_t wp_type;
    uint8_t wp_end_type;
    bool wp_takeoff;
    uint8_t wp_yaw_type;
    float wp_move_vel;
    float wp_vel_p;
    float z_height;
    double wp_point_1[4];
    double wp_point_2[4];
    double wp_point_3[4];
    double wp_point_4[4];
    double wp_point_5[4];
    double wp_point_6[4];
    double wp_point_7[4];
    double wp_point_8[4];
    double wp_point_9[4];
    double wp_point_10[4];
    double wp_circle_point[2];//环绕点 xy 或 经纬

    void init()
    {
        wp_num=0;
        wp_type=0;
        wp_end_type=0;
        wp_takeoff=true;
        wp_yaw_type=1;
        wp_move_vel=1;
        wp_vel_p=1;
        z_height=1;
        for(int i=0;i<4;i++)
        {
            wp_point_1[i]=0;
            wp_point_2[i]=0;
            wp_point_3[i]=0;
            wp_point_4[i]=0;
            wp_point_5[i]=0;
            wp_point_6[i]=0;
            wp_point_7[i]=0;
            wp_point_8[i]=0;
            wp_point_9[i]=0;
            wp_point_10[i]=0;
        }
       wp_circle_point[0]=0;
       wp_circle_point[1]=0;

    }
};

//无人车控制指令 - UGVControlCMD（#120）
struct UGVControlCMD
{

    uint8_t cmd;
    uint8_t yaw_type;
    float desired_pos[2];
    float desired_vel[2];
    float desired_yaw;
    float angular_vel;

    void init()
    {
        cmd=0;
        yaw_type=0;
        for(int i=0;i<2;++i)
        {
            desired_pos[i]=0;
            desired_vel[i]=0;
        }
        desired_yaw=0;
        angular_vel=0;
    }

};

//搜索在线智能体 - SearchData（#200）
struct SearchData
{
    uint64_t port;

    void init()
    {

        port=0;
    }
};

//智能体应答 - ACKData（#201）
struct ACKData
{
    uint8_t agentType;
    uint8_t ID;
    uint16_t port;

    void init()
    {
        agentType=0;
        ID=0;
        port=0;
    }
};

//智能体demo - DemoData（#202）
struct DemoData
{
    bool demoState;
    uint16_t demoSize;
//    std::string demoStr;//std::string类型为非平凡类型，无法作为联合体成员
    char demoStr[250];
    void init()
    {
        demoState=false;
        demoSize=0;

    }
};

//功能脚本 - ScriptData（#203）
struct ScriptData
{
    uint8_t scripType;
    bool scriptState;
    uint16_t scriptSize;
    char scriptStr[250];
    void init()
    {
        scripType=0;
        scriptState=false;
        scriptSize=0;

    }
};

//规划点 - Goal（#204）
struct Goal
{

    double positionX;
    double positionY;
    double positionZ;

    void init()
    {
        positionX=0;
        positionY=0;
        positionZ=0;

    }
};

// 有效数据部分联合体，用于传递有效数据
union Payload
{
    HeartbeatData heartbeat;            // 无人机心跳包 - HeartbeatData（#1）
    UAVState uavState;                  // 无人机状态 - UAVState（#2）
    UAVControlCMD uavControlCMD;        // 无人机控制指令 - UAVControlCMD（#102）
    UAVSetup uavSetup;                  // 无人机设置指令 - UAVSetup（#103）
    WaypointData waypointData;          // 无人机航点 - WaypointData（#104）
    SearchData search;                  // 搜索在线智能体 - SearchData（#200）
    ACKData ack;                        // 智能体应答 - ACKData（#201）
    DemoData demo;                      // 无人机demo - DemoData（#202）
    ScriptData agentScrip;              // 功能脚本 - ScriptData（#203）
    UGVState ugvState;                  // 无人车状态 - UGVState（#20）
    UGVControlCMD ugvControlCMD;        // 无人车控制指令 - UGVControlCMD（#120）
    NodeData nodeInformation;           // 机载电脑ROS节点 - NodeData（#30）
    Formation formation;                // 编队切换 - Formation（#40）
    Goal goal;                          // 规划点- Goal（#204）
    AgentComputerStatus computerStatus; // 智能体电脑状态 -AgentComputerStatus（#31）
};

//整个数据帧
struct DataFrame
{
    uint16_t head;
    uint32_t length;
    uint8_t  seq;
    uint8_t  robot_ID;
    uint64_t timestamp;
    Payload  data;
    uint16_t check;
};


//接收到的数据参数结构体
struct ReceivedParameter
{
    DataFrame dataFrame;
    std::string ip;
    uint8_t communicationType;//通信类型
    unsigned short port;
};

//设备参数
struct DeviceData
{
    int agentType;
    int ID;
    std::string ip;
    unsigned short port;
};

//连接状态
struct CommunicationState
{
    int sock;
    std::string ip;
    int state;
    unsigned short port;
};


#endif // MSG_H
