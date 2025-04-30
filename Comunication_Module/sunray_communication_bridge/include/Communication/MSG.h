#ifndef MSG_H
#define MSG_H

#include <cstdint>
#include <string.h>
#include <vector>
#include <string>


//
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
enum VehicleControlType
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
    HeartbeatMessageID    = 1,
    StateMessageID        = 2,
    UGVStateMessageID     = 20,
//    TakeoffMessageID    = 101,
    ControlMessageID      = 102,
    VehicleMessageID      = 103,
    WaypointMessageID     = 104,
    UGVControlMessageID   = 120,
    SearchMessageID       = 200,
    ACKMessageID          = 201,
    DemoMessageID         = 202,
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



//
//struct TakeoffData     //停用
//{
//    uint8_t robotID;    /**< @param robot_id robot ID */
//    uint8_t msgType;
//    uint64_t timestamp; /**< @param time_stamp timestamp */
//    uint8_t takeoff;     /**< @param takeoff takeoff flag */
//};



struct VehicleData
{

    uint8_t robotID;    /**< @param robot_id robot ID */
    uint8_t msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t sunray_mode;        /**< @param type vehicle control type */
    uint8_t px4_mode;

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        sunray_mode=0;
        px4_mode=0;
    }
};

struct SpaceCoordinates
{
    float x;//r
    float y;//p
    float z;//y
};

struct FloatPair {
    float x;
    float y;
};

struct QuaternionData
{
    float w;
    float x;
    float y;
    float z;
};

struct ControlData
{
    uint8_t robotID;    /**< @param robot_id robot ID */
    uint8_t msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t controlMode;        /**< @param type control type */
    SpaceCoordinates position;      // [m]
    SpaceCoordinates velocity;      // [m/s]
    SpaceCoordinates accelerometer;
    float yaw;           /**< @param yaw yaw angle */
    float roll;          /**< @param roll roll angle */
    float pitch;         /**< @param pitch pitch angle */
    float latitude;      //经度
    float longitude;     //纬度
    float altitude;      //海拔
    float yawRate;       /**< @param yaw_rate yaw rate flag */
    //    uint8_t frame;       /**< @param frame reference frame */

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        controlMode=0;
        position.x=0;
        position.y=0;
        position.z=0;
        velocity.x=0;
        velocity.y=0;
        velocity.z=0;
        accelerometer.x=0;
        accelerometer.y=0;
        accelerometer.z=0;
        yaw=0;
        roll=0;
        pitch=0;
        latitude=0;
        longitude=0;
        altitude=0;
        yawRate=0;
    }
};

struct UGVControlData
{
    uint8_t robotID;    /**< @param robot_id robot ID */
    uint8_t msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t controlMode;        /**< @param type control type */
    uint8_t yawType;        // 偏航角类型 0 角速度 1 角度（需要有定位支持）,地面站默认是1

    FloatPair desiredPos;      // [m]
    FloatPair desiredVel;      // [m/s]

    float desiredYaw;
    float angularVel;          //[rad/s] 地面站不接入，默认给0


    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        controlMode=0;
        yawType=1;
        desiredPos.x=0;
        desiredPos.y=0;
        desiredVel.x=0;
        desiredVel.y=0;
        desiredYaw=0;
        angularVel=0;

    }
};

struct StateData
{
    uint8_t robotID;  /**< @param robot_id robot ID */
    uint8_t msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t uavID;   /**< @param uav_id UAV ID */
    bool connected;   /**< @param connected connection status */
    bool armed;       /**< @param armed arm status */
    uint8_t mode; /**< @param mode flight mode lenght 1
                    1.INIT；2.RC_CONTROL；3.CMD_CONTROL；4.LAND_CONTROL*/

    uint8_t locationSource;          // 0: Mocap, 1: RTK, 2: Gazebo, 3:
    bool odom_valid;                  // 0: invalid, 1: valid
    SpaceCoordinates position;      // [m]
    SpaceCoordinates velocity;      // [m/s]
    QuaternionData attitudeQuaternion; //四元数
    SpaceCoordinates attitude;      // [rad]
    SpaceCoordinates attitudeRate; // [rad/s]
    SpaceCoordinates posSetpoint;      // [m]
    SpaceCoordinates velSetpoint;      // [m/s]
    SpaceCoordinates attSetpoint;      // [rad]

    float batteryState;      // [V]
    float batteryPercentage; // [0-1]

    uint8_t controlMode;
    uint8_t moveMode;

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;

        uavID=0;
        connected=false;
        armed=false;
        mode=0;

        locationSource=0;
        odom_valid=false;
        position.x=0;
        position.y=0;
        position.z=0;
        velocity.x=0;
        velocity.y=0;
        velocity.z=0;
        attitude.x=0;
        attitude.y=0;
        attitude.z=0;
        attitudeRate.x=0;
        attitudeRate.y=0;
        attitudeRate.z=0;
        posSetpoint.x=0;
        posSetpoint.y=0;
        posSetpoint.z=0;
        velSetpoint.x=0;
        velSetpoint.y=0;
        velSetpoint.z=0;
        attSetpoint.x=0;
        attSetpoint.y=0;
        attSetpoint.z=0;
        attitudeQuaternion.w=0;
        attitudeQuaternion.x=0;
        attitudeQuaternion.y=0;
        attitudeQuaternion.z=0;

        batteryState=0;
        batteryPercentage=0;

        controlMode=0;
        moveMode=0;
    }

};


struct UGVStateData
{
    uint8_t  robotID;  /**< @param robot_id robot ID */
    uint8_t  msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t  ugvID;   /**< @param uav_id UGV ID */
    uint8_t locationSource;          // 0: Mocap, 1: RTK, 2: Gazebo, 3:
    bool connected;   /**< @param connected connection status */
    bool odom_valid;                  // 0: invalid, 1: valid
    FloatPair position;      // [m]
    FloatPair velocity;      // [m/s]
    float yaw;

    FloatPair posSetpoint;      // [m]
    FloatPair velSetpoint;      // [m/s]
    float yawSetpoint;

    float batteryState;      // [V]
    float batteryPercentage; // [0-1]

    uint8_t controlMode;


    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;

        ugvID=0;
        connected=false;
        locationSource=0;
        odom_valid=false;

        position.x=0;
        position.y=0;
        velocity.x=0;
        velocity.y=0;
        yaw=0;

        posSetpoint.x=0;
        posSetpoint.y=0;
        velSetpoint.x=0;
        velSetpoint.y=0;
        yawSetpoint=0;

        batteryState=0;
        batteryPercentage=0;

        controlMode=0;
    }

};

// Message structures 心跳包
struct HeartbeatData
{
    uint8_t robotID;  /**< @param robot_id robot ID */
    uint8_t msgType;
    uint64_t timestamp;
    uint8_t agentType;
    int32_t count;       /**< @param head message header 心跳包计数，未启用 */

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        count=0;
        agentType=0;
    }
};

struct SearchData
{
    uint8_t robotID;
    uint8_t msgType;
    uint64_t timestamp;
    uint64_t port;

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        port=0;
    }
};

struct ACKData
{
    uint8_t robotID;
    uint8_t msgType;
    uint64_t timestamp;
    uint8_t agentType;
    uint8_t ID;
    uint16_t port;

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        agentType=0;
        ID=0;
        port=0;
    }
};

struct DemoData
{
    uint8_t robotID;
    uint8_t msgType;
    uint64_t timestamp; /**< @param head message header */
    uint8_t uavID;
    bool demoState;
    uint16_t demoSize;
//    std::string demoStr;
    char demoStr[250];
    void init()
    {
        robotID=0;
        msgType=0;
        uavID=0;
        demoState=false;
        demoSize=0;

    }
};

struct WaypointSingleData
{
    double X;//经度Lon
    double Y;//纬度Lat
    double Z;//高
    double Yaw;
};

struct WaypointData
{
    uint8_t robotID;
    uint8_t msgType;
    uint64_t timestamp; /**< @param time_stamp timestamp */
    uint8_t uavID;    /**< @param robot_id robot ID */
    uint8_t wpType;
    uint8_t wpNum;
    uint8_t wpEndType;
    bool wpTakeoff;
    uint8_t wpYawType;
    float wpMoveVel;
    float wpVelP;
    float wpHeight;
    WaypointSingleData Waypoint1;
    WaypointSingleData Waypoint2;
    WaypointSingleData Waypoint3;
    WaypointSingleData Waypoint4;
    WaypointSingleData Waypoint5;
    WaypointSingleData Waypoint6;
    WaypointSingleData Waypoint7;
    WaypointSingleData Waypoint8;
    WaypointSingleData Waypoint9;
    WaypointSingleData Waypoint10;
    double wpCirclePointX;//环绕点X或经度
    double wpCirclePointY;//环绕点Y或纬度

    void init()
    {
        robotID=0;
        msgType=0;
        timestamp=0;
        uavID=0;
        wpType=0;
        wpNum=0;
        wpEndType=0;
        wpTakeoff=true;
        wpYawType=1;
        wpMoveVel=1;
        wpVelP=1;
        wpHeight=1;
        Waypoint1.X=0;
        Waypoint2.X=0;
        Waypoint3.X=0;
        Waypoint4.X=0;
        Waypoint5.X=0;
        Waypoint6.X=0;
        Waypoint7.X=0;
        Waypoint8.X=0;
        Waypoint9.X=0;
        Waypoint10.X=0;
        Waypoint1.Y=0;
        Waypoint2.Y=0;
        Waypoint3.Y=0;
        Waypoint4.Y=0;
        Waypoint5.Y=0;
        Waypoint6.Y=0;
        Waypoint7.Y=0;
        Waypoint8.Y=0;
        Waypoint9.Y=0;
        Waypoint10.Y=0;
        Waypoint1.Z=0;
        Waypoint2.Z=0;
        Waypoint3.Z=0;
        Waypoint4.Z=0;
        Waypoint5.Z=0;
        Waypoint6.Z=0;
        Waypoint7.Z=0;
        Waypoint8.Z=0;
        Waypoint9.Z=0;
        Waypoint10.Z=0;
        Waypoint1.Yaw=0;
        Waypoint2.Yaw=0;
        Waypoint3.Yaw=0;
        Waypoint4.Yaw=0;
        Waypoint5.Yaw=0;
        Waypoint6.Yaw=0;
        Waypoint7.Yaw=0;
        Waypoint8.Yaw=0;
        Waypoint9.Yaw=0;
        Waypoint10.Yaw=0;
        wpCirclePointX=0;
        wpCirclePointY=0;
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

//不同类型数据联合体，用于传递数据
union unionData
{
    HeartbeatData heartbeat;
    VehicleData vehicle;
    ControlData control;
    StateData state;
    SearchData search;
    ACKData ack;
    DemoData demo;
    WaypointData waypointData;
    UGVStateData ugvState;
    UGVControlData ugvControl;
};

//接受到的数据参数结构体
struct ReceivedParameter
{
    int messageID;
    unionData data;
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
