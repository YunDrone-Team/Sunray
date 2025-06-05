#ifndef MSG_H
#define MSG_H

#include <cstdint>
#include <string.h>
#include <vector>
#include <string>


// 控制命令宏定义 - 同 UAVControlCMD.msg 中的控制命令枚举
enum ControlType
{
    XyzPos                      =1,           //XYZ位置 不带偏航角
    XyzVel                      =2,           //XYZ速度 不带偏航角
    XyVelZPos                   =3,           //XY速度 Z位置 不带偏航角
    XyzPosYaw                   =4,           //XYZ位置 带偏航角
    XyzPosYawrate               =5,           //XYZ位置 带偏航角速率
    XyzVelYaw                   =6,           //XYZ速度 带偏航角
    XyzVelYawrate               =7,           //XYZ速度 带偏航角速率
    XyVelZPosYaw                =8,           //XY速度 Z位置 带偏航角
    XyVelZPosYawrate            =9,           //XY速度 Z位置 带偏航角速率
    XyzPosVelYaw                =10,          //XYZ位置速度 带偏航角
    XyzPosVelYawrate            =11,          //XYZ位置速度 带偏航角速率          地面站不需要支持（待定）
    PosVelAccYaw                =12,          //XYZ位置速度加速度 带偏航角        地面站不需要支持
    PosVelAccYawrate            =13,          //XYZ位置速度加速度 带偏航角速率     地面站不需要支持
    XyzPosYawBody               =14,          //XYZ位置 带偏航角 机体坐标系
    XyzVelYawBody               =15,          //XYZ速度 带偏航角 机体坐标系
    XyVelZPosYawBody            =16,          //XY速度 Z位置 带偏航角 机体坐标系
    GlobalPos                   =17,          //全局坐标(绝对坐标系下的经纬度)

    Point                       =30,          //规划点
    CTRLXyzPos                  =50,          //姿态控制,惯性系定点控制 带偏航角
    TakeoffControlType                     =100,         //起飞
    LandControlType                        =101,         //降落
    HoverControlType                       =102,         //悬停
    WaypointControlType                    =103,         //航点        特殊模式 需要传入多个航点 后续再适配
    ReturnControlType                      =104,         //返航
};


enum ControlMode // 无人机控制模式
{
  INIT              = 0,     // 初始模式
  RC_CONTROL        = 1,     // 遥控器控制模式
  CMD_CONTROL       = 2,     // 外部指令控制模式
  LAND_CONTROL      = 3,     // 降落
  WITHOUT_CONTROL   = 4,     // 无控制
};

enum class UGVControlMode // 无人车控制模式
{
  INIT              = 0,     // 初始模式
  HOLD              = 1,     //
  POS_CONTROL       = 2,     //
  POS_CONTROL_BODY  = 3,     //
  VEL_CONTROL       = 4,     //
  VEL_CONTROL_BODY  = 5,
  Planner_Control   = 6,

};



//stop
enum ModelType
{
    OffBoardModelType      = 1,
    PositionModelType      = 2,
    HoldModelType          = 3,
    StabilizedModelType    = 4,
    ManualModelType        = 5,
};




//Arm解锁 Vehicle control type
enum UAVSetupType
{
    DisarmControlType       = 0,
    ArmControlType          = 1,
    SetPX4ModeControlType   = 2,
    RebootPX4ControlType    = 3,
    SetControlMode          = 4,
    KillControlType         = 5,
};


//MESSAGE ID
enum MessageID
{
    HeartbeatMessageID          = 1,
    UAVStateMessageID           = 2,
    UGVStateMessageID           = 20,
    NodeMessageID               = 30,
//    TakeoffMessageID    = 101,
    UAVControlCMDMessageID      = 102,
    UAVSetupMessageID           = 103,
    WaypointMessageID           = 104,
    UGVControlCMDMessageID      = 120,
    SearchMessageID             = 200,
    ACKMessageID                = 201,
    DemoMessageID               = 202,
    ScriptMessageID             = 203,

};




//1.INIT；2.RC_CONTROL；3.CMD_CONTROL；4.LAND_CONTROL
enum ControlState
{
    InitControlState        = 1,
    RcControlControlState   = 2,
    CMDControlControlState  = 3,
    LandControlControlState = 4,
};

enum PX4ModeType
{
    ManualType          =1,
    StabilizedType      =2,
    AcroType            =3,
    RattitudeType       =4,
    AltitudeType        =5,
    OffboardType        =6,
    PositionType        =7,
    HoldType            =8,
    MissionType         =9,
    ReturnType          =10,
    FollowMeType        =11,
    PrecisionLandType   =12,
};

enum TCPClientState
{
    ConnectionSuccessful    =1,
    ConnectionFail          =2,
    ConnectionBreak         =3,
    ConnectionTimeout       =4,
};



enum WaypointType
{
    NEDType     =0,           //
    LonLatType  =1,
};

enum WaypointEndType
{
    HoverEndType     =1,           //
    LandEndType      =2,
    ReturnFlight     =3,
};

enum WaypointYawType
{
    FixedValue      =1,
    NextWaypoint    =2,
    CirclePoint     =3,
};



//  心跳包
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

struct SearchData
{
    uint64_t port;

    void init()
    {

        port=0;
    }
};

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
            wp_point_10[2]=0;
        }
       wp_circle_point[0]=0;
       wp_circle_point[1]=0;

    }
};



//通信数据包
struct CommunicationPack
{
    std::vector<char> sendData; //数据
    std::string targetIp;       //目标ip
    uint16_t targetPort;        //目标端口，TCP可不填
    uint8_t communicationType;  //通信类型
};

//通信类型
enum CommunicationType
{
    TCPServerCommunicationType      = 1,//TCPServer
    TCPClientCommunicationType      = 2,//TCPClient
    UDPUnicastCommunicationType     = 3,//UDP单播
    UDPBroadcastCommunicationType   = 4,//UDP广播
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

// 有效数据部分联合体，用于传递有效数据
union Payload
{
    HeartbeatData heartbeat;        // 无人机心跳包 - HeartbeatData（#1）
    UAVState uavState;              // 无人机状态 - UAVState（#2）
    UAVControlCMD uavControlCMD;    // 无人机控制指令 - UAVControlCMD（#102）
    UAVSetup uavSetup;              // 无人机设置指令 - UAVSetup（#103）
    WaypointData waypointData;      // 无人机航点 - WaypointData（#104）
    SearchData search;              // 搜索在线智能体 - SearchData（#200）
    ACKData ack;                    // 智能体应答 - ACKData（#201）
    DemoData demo;                  // 无人机demo - DemoData（#202）
    ScriptData agentScrip;          // 功能脚本 - ScriptData（#203）
    UGVState ugvState;              // 无人车状态 - UGVState（#20）
    UGVControlCMD ugvControlCMD;    // 无人车控制指令 - UGVControlCMD（#120）
    NodeData nodeInformation;       // 机载电脑ROS节点 - NodeData（#30）
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


struct CommunicationState
{
    int sock;
    std::string ip;
    int state;
    unsigned short port;
};


#endif // MSG_H
