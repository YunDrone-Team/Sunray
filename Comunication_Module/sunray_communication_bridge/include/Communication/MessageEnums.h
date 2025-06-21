#ifndef MESSAGEENUMS_H
#define MESSAGEENUMS_H

//MESSAGE ID
enum MessageID
{
    HeartbeatMessageID              = 1,
    UAVStateMessageID               = 2,
    UGVStateMessageID               = 20,
    NodeMessageID                   = 30,
    AgentComputerStatusMessageID    = 31,
    FormationMessageID              = 40,
    GroundFormationMessageID        = 41,
//    TakeoffMessageID    = 101,
    UAVControlCMDMessageID          = 102,
    UAVSetupMessageID               = 103,
    WaypointMessageID               = 104,
    UGVControlCMDMessageID          = 120,
    SearchMessageID                 = 200,
    ACKMessageID                    = 201,
    DemoMessageID                   = 202,
    ScriptMessageID                 = 203,
    GoalMessageID                   = 204,

};

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

// 无人机控制模式
enum ControlMode
{
  INIT              = 0,     // 初始模式
  RC_CONTROL        = 1,     // 遥控器控制模式
  CMD_CONTROL       = 2,     // 外部指令控制模式
  LAND_CONTROL      = 3,     // 降落
  WITHOUT_CONTROL   = 4,     // 无控制
};

// 无人车控制模式
enum class UGVControlMode
{
  INIT              = 0,     // 初始模式
  HOLD              = 1,     //
  POS_CONTROL       = 2,     //
  POS_CONTROL_BODY  = 3,     //
  VEL_CONTROL       = 4,     //
  VEL_CONTROL_BODY  = 5,
  Planner_Control   = 6,

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
    LandSetupType         = 6,
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
    NEDType     =0,
    LonLatType  =1,
};

enum WaypointEndType
{
    HoverEndType     =1,
    LandEndType      =2,
    ReturnFlight     =3,
};

enum WaypointYawType
{
    FixedValue      =1,
    NextWaypoint    =2,
    CirclePoint     =3,
};

enum FormationCMD
{
     FormationINIT=0 ,
     FormationTAKEOFF=1 ,    // 起飞
     FormationLAND=2,        // 降落
     FormationHOVER=3,       // 悬停
     FormationFORMATION=4,   // 设置编队
     FormationSET_HOME=5,    // 设置home点
     FormationRETURN_HOME=6, // 返回home点
};

enum FormationType
{
    FormationGOAL=0,
    FormationSTATIC=1,
    FormationDYNAMIC=2,
    FormationLEADER=3,
    FormationFLLOW=4,
};

//通信类型
enum CommunicationType
{
    TCPServerCommunicationType      = 1,//TCPServer
    TCPClientCommunicationType      = 2,//TCPClient
    UDPUnicastCommunicationType     = 3,//UDP单播
    UDPBroadcastCommunicationType   = 4,//UDP广播
};

#endif // MESSAGEENUMS_H
