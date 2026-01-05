#include "ros_msg_utils.h"
#include "type_mask.h"
#include "pos_controller_pid.h"

#define FLIP_ANGLE M_PI/3
#define RC_DEADZONE 0.2

class UAVControl
{
private:
    std::string node_name;  // 节点名称
    int uav_id;             // 【参数】无人机ID
    std::string uav_name;   // 【参数】无人机名称

    // 无人机飞行相关参数
    struct FlightParams
    {
        float takeoff_height;                        // 【参数】默认起飞高度
        int land_type = 0;                           // 【参数】降落类型【0:到达指定高度后锁桨 1:使用px4 auto.land】
        float land_speed;                            // 【参数】降落速度
        float land_end_time;                         // 【参数】降落最后一阶段时间
        float land_end_speed;                        // 【参数】降落最后一阶段速度
        bool set_home = false;                       // 起飞点是否设置
        Eigen::Vector3d home_pos{0.0, 0.0, 0.0};     // 【参数】起飞点
        float home_yaw = 0.0;                        // 起飞点航向
        Eigen::Vector3d hover_pos{0.0, 0.0, 0.0};    // 悬停点
        float hover_yaw = 0.0;                       // 无人机目标航向
        bool set_land_pos{false};                    // 降落点是否设置
        Eigen::Vector3d land_pos{0.0, 0.0, 0.0};     // 降落点
        float land_yaw = 0.0;                        // 降落点航向
        Eigen::Vector3d relative_pos{0.0, 0.0, 0.0}; // 相对位置
    };
    FlightParams flight_params;

    // 无人机地理围栏 - 超出围栏后无人机自动降落
    struct geo_fence
    {
        float x_min = 5.0;
        float x_max = 5.0;
        float y_min = 5.0;
        float y_max = 5.0;
        float z_min = -0.1;
        float z_max = 2.0;
    };
    geo_fence uav_geo_fence;

    // 无人机系统参数
    struct SystemParams
    {
        bool allow_lock;          // 【参数】允许临时解锁 特殊模式下允许跳过解锁检查保持在CMD_CONTROL模式
        bool use_rc;              // 【参数】是否使用遥控器控制
        bool get_rc_signal;       // 是否收到遥控器的信号
        bool check_flip;          // 【参数】是否使用翻转上锁，默认启动
        bool use_offset;          // 【参数】是否使用位置偏移，TODO
        bool check_cmd_timeout;   // 【参数】是否检查无人机控制指令超时
        float cmd_timeout;        // 指令超时时间
        int control_mode;         // 无人机控制模式
        int last_control_mode;    // 上一时刻无人机控制模式
        uint16_t type_mask;       // 控制指令类型
        int safety_state;         // 安全标志，0代表正常，其他代表不正常
        ros::Time last_land_time; // 进入降落最后一阶段的时间戳
        ros::Time last_rc_time;   // 上一个rc控制时间点
        ros::Time low_velocity_start_time; // 低速检测开始时间
    };
    SystemParams system_params;

    // 无人机控制状态机
    enum Control_Mode
    {
        INIT = 0,           // 初始模式
        RC_CONTROL = 1,     // 遥控器控制模式
        CMD_CONTROL = 2,    // 外部指令控制模式
        LAND_CONTROL = 3,   // 降落模式
        WITHOUT_CONTROL = 4 // 无控制模式
    };

    struct RCControlParams
    {
        float max_vel_xy = 1.0;  // 最大水平速度
        float max_vel_z = 1.0;   // 最大垂直速度
        float max_vel_yaw = 1.5; // 最大偏航速度
    };
    RCControlParams rc_control_params;

    // 添加无人机控制模式的string的映射（用于打印）
    std::map<int, std::string> modeMap =
    {
        {int(Control_Mode::INIT), "INIT"},
        {int(Control_Mode::RC_CONTROL), "RC_CONTROL"},
        {int(Control_Mode::CMD_CONTROL), "CMD_CONTROL"},
        {int(Control_Mode::LAND_CONTROL), "LAND_CONTROL"},
        {int(Control_Mode::WITHOUT_CONTROL), "WITHOUT_CONTROL"}
    };

    // 添加每个移动模式对应的string的映射（用于打印）
    std::map<int, std::string> moveModeMapStr =
    {
        {sunray_msgs::UAVControlCMD::XyzPos, "XyzPos"},
        {sunray_msgs::UAVControlCMD::XyzVel, "XyzVel"},
        {sunray_msgs::UAVControlCMD::XyVelZPos, "XyVelZPos"},
        {sunray_msgs::UAVControlCMD::XyzPosYaw, "XyzPosYaw"},
        {sunray_msgs::UAVControlCMD::XyzPosYawrate, "XyzPosYawrate"},
        {sunray_msgs::UAVControlCMD::XyzVelYaw, "XyzVelYaw"},
        {sunray_msgs::UAVControlCMD::XyzVelYawrate, "XyzVelYawrate"},
        {sunray_msgs::UAVControlCMD::XyVelZPosYaw, "XyVelZPosYaw"},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawrate, "XyVelZPosYawrate"},
        {sunray_msgs::UAVControlCMD::XyzPosVelYaw, "XyzPosVelYaw"},
        {sunray_msgs::UAVControlCMD::XyzPosVelYawrate, "XyzPosVelYawrate"},
        {sunray_msgs::UAVControlCMD::PosVelAccYaw, "PosVelAccYaw"},
        {sunray_msgs::UAVControlCMD::PosVelAccYawrate, "PosVelAccYawrate"},
        {sunray_msgs::UAVControlCMD::XyzPosYawBody, "XyzPosYawBody"},
        {sunray_msgs::UAVControlCMD::XyzVelYawBody, "XyzVelYawBody"},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawBody, "XyVelZPosYawBody"},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawrateBody, "XyVelZPosYawrateBody"},
        {sunray_msgs::UAVControlCMD::XyzAcc, "XyzAcc"},
        {sunray_msgs::UAVControlCMD::XyzAccYaw, "XyzAccYaw"},
        {sunray_msgs::UAVControlCMD::XyzAccYawrate, "XyzAccYawrate"},
        {sunray_msgs::UAVControlCMD::GlobalPos, "GlobalPos"},
        {sunray_msgs::UAVControlCMD::Point, "Point"},
        {sunray_msgs::UAVControlCMD::CTRL_XyzPos, "CTRL_XyzPos"},
        {sunray_msgs::UAVControlCMD::CTRL_Traj, "CTRL_Traj"},
        {sunray_msgs::UAVControlCMD::Takeoff, "Takeoff"},
        {sunray_msgs::UAVControlCMD::Land, "Land"},
        {sunray_msgs::UAVControlCMD::Hover, "Hover"},
        {sunray_msgs::UAVControlCMD::Waypoint, "Waypoint"},
        {sunray_msgs::UAVControlCMD::Return, "Return"}
    };
    
    // 添加每个基础移动模式对应的typemask的映射（用于发布mavros控制话题）
    std::map<int, uint16_t> moveModeMap =
    {
        {sunray_msgs::UAVControlCMD::XyzPos, TypeMask::XYZ_POS},
        {sunray_msgs::UAVControlCMD::XyzVel, TypeMask::XYZ_VEL},
        {sunray_msgs::UAVControlCMD::XyVelZPos, TypeMask::XY_VEL_Z_POS},
        {sunray_msgs::UAVControlCMD::XyzPosYaw, TypeMask::XYZ_POS_YAW},
        {sunray_msgs::UAVControlCMD::XyzPosYawrate, TypeMask::XYZ_POS_YAWRATE},
        {sunray_msgs::UAVControlCMD::XyzVelYaw, TypeMask::XYZ_VEL_YAW},
        {sunray_msgs::UAVControlCMD::XyzVelYawrate, TypeMask::XYZ_VEL_YAWRATE},
        {sunray_msgs::UAVControlCMD::XyVelZPosYaw, TypeMask::XY_VEL_Z_POS_YAW},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawrate, TypeMask::XY_VEL_Z_POS_YAWRATE},
        {sunray_msgs::UAVControlCMD::XyzPosVelYaw, TypeMask::XYZ_POS_VEL_YAW},
        {sunray_msgs::UAVControlCMD::XyzPosVelYawrate, TypeMask::XYZ_POS_VEL_YAWRATE},
        {sunray_msgs::UAVControlCMD::PosVelAccYaw, TypeMask::POS_VEL_ACC_YAW},
        {sunray_msgs::UAVControlCMD::PosVelAccYawrate, TypeMask::POS_VEL_ACC_YAWRATE},
        {sunray_msgs::UAVControlCMD::XyzPosYawBody, TypeMask::XYZ_POS_YAW},
        {sunray_msgs::UAVControlCMD::XyzVelYawBody, TypeMask::XYZ_VEL_YAW},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawBody, TypeMask::XY_VEL_Z_POS_YAW},
        {sunray_msgs::UAVControlCMD::XyVelZPosYawrateBody, TypeMask::XY_VEL_Z_POS_YAWRATE},
        {sunray_msgs::UAVControlCMD::XyzAcc, TypeMask::XYZ_ACC},
        {sunray_msgs::UAVControlCMD::XyzAccYaw, TypeMask::XYZ_ACC_YAW},
        {sunray_msgs::UAVControlCMD::XyzAccYawrate, TypeMask::XYZ_ACC_YAWRATE}
    };

    // 绑定特殊指令对应的实现函数（用于执行不同的特殊指令）
    std::map<int, std::function<void()>> advancedModeFuncMap;

    // 订阅句柄
    ros::Subscriber px4_state_sub;    // 【订阅】无人机状态订阅 来自externalFusion节点
    ros::Subscriber control_cmd_sub;  // 【订阅】控制指令订阅
    ros::Subscriber setup_sub;        // 【订阅】无人机模式设置
    ros::Subscriber rc_state_sub;     // 【订阅】rc_control状态订阅
    ros::Subscriber uav_waypoint_sub; // 【订阅】无人机航点订阅

    // 发布句柄
    ros::Publisher uav_state_pub;             // 【发布】无人机状态发布
    ros::Publisher px4_setpoint_local_pub;    // 【发布】无人机指令发布 NED
    ros::Publisher px4_setpoint_global_pub;   // 【发布】无人机指令发布 经纬度+海拔
    ros::Publisher px4_setpoint_attitude_pub; // 【发布】无人机指令发布 姿态+推力
    ros::Publisher goal_pub;                  // 【发布】发布一个目标点 来自外部控制指令
    
    // 服务句柄
    ros::ServiceClient px4_arming_client;    // 【服务】px4解锁
    ros::ServiceClient px4_set_mode_client;  // 【服务】px4模式设置
    ros::ServiceClient px4_reboot_client;    // 【服务】px4重启
    ros::ServiceClient px4_emergency_client; // 【服务】px4紧急停止

    void printf_params();
    int safetyCheck();     // 安全检查
    void setArm(bool arm); // 设置解锁 0:上锁 1:解锁
    void reboot_px4();     // 重启
    void emergencyStop();  // 紧急停止出来
    void set_px4_flight_mode(std::string mode);
    void send_attitude_setpoint(Eigen::Vector4d &u_att);                                      // 设置模式
    void setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint);        // 【发布】发送控制指令
    void pub_setpoint_raw_global(mavros_msgs::GlobalPositionTarget setpoint); // 【发布】发送控制指令
    void handle_cmd_control();                                                                // CMD_CONTROL模式下获取期望值
    void handle_rc_control();                                                                 // RC_CONTROL模式下获取期望值
    void handle_land_control();                                                               // LAND_CONTROL模式下获取期望值
    void set_desired_from_hover();                                                            // Hover模式下获取期望值
    void check_state();                                                                       // 安全检查 + 发布状态
    void set_hover_pos();                                                                     // 设置悬停位置
    void set_default_local_setpoint();                                                        // 设置默认期望值
    void set_default_global_setpoint();                                                       // 设置默认期望值
    void body2enu(double body_frame[2], double enu_frame[2], double yaw);                     // body坐标系转ned坐标系
    void set_offboard_control(int mode);                                                      // 设置offboard控制模式
    void return_to_home();                                                                    // 返航模式实现
    void waypoint_mission();                                                                  // 航点任务实现
    void set_takeoff();                                                                       // 起飞模式实现
    void set_land();                                                                          // 降落模式实现
    void publish_goal();                                                                      // 发布规划点
    void pos_controller();
    void check_flip();
    // 回调函数
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void uav_setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg);
    void px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg);
    void rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg);                        // 遥控器状态回调函数

    void pub_setpoint_raw_local(mavros_msgs::PositionTarget setpoint);

public:
    UAVControl() {};
    ~UAVControl() {};

    PosControlPID pos_controller_pid;

    sunray_msgs::UAVControlCMD control_cmd;               // 当前时刻无人机控制指令（来自任务节点）
    sunray_msgs::UAVControlCMD last_control_cmd;          // 上一时刻无人机控制指令（来自任务节点）
    sunray_msgs::PX4State px4_state;                      // 当前时刻无人机状态（来自external_fusion_node）
    sunray_msgs::UAVState uav_state;                      // 当前时刻无人机状态（本节点发布）
    sunray_msgs::RcState rc_state;                        // 无人机遥控器状态（来自遥控器输入节点）
    mavros_msgs::PositionTarget local_setpoint;           // PX4的本地位置设定点（待发布）
    mavros_msgs::GlobalPositionTarget global_setpoint;    // PX4的全局位置设定点（待发布）
    mavros_msgs::AttitudeTarget att_setpoint;             // PX4的姿态设定点（待发布）

    void mainLoop();
    void show_ctrl_state(); // 打印状态
    void init(ros::NodeHandle &nh);
};