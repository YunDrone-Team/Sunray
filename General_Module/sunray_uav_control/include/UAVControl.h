#include "ros_msg_utils.h"
#include "type_mask.h"
#include "printf_format.h"
#include "pos_controller_pid.h"
#include "geometry_utils.h"
#include "math_utils.h"

using namespace sunray_logger;

class UAVControl
{
private:
    std::string uav_ns;   // 节点名称
    int uav_id;           // 无人机ID
    std::string uav_name; // 无人机名称

    // 无人机飞行相关参数
    struct FlightParams
    {
        float takeoff_height;                        // 起飞高度
        int land_type = 0;                           // 降落类型 【0:到达指定高度后锁桨 1:使用px4 auto.land】
        float disarm_height;                         // 锁桨高度
        float land_speed;                            // 降落速度
        Eigen::Vector3d land_pos{0.0, 0.0, 0.0};     // 降落点
        float land_yaw = 0.0;                        // 降落点航向
        float land_end_time;                         // 降落最后一段时间
        float land_end_speed;                        // 降落最后一段速度
        bool set_home = false;                       // 起飞点是否设置
        Eigen::Vector3d home_pos{0.0, 0.0, 0.0};     // 起飞点
        float home_yaw = 0.0;                        // 起飞点航向
        Eigen::Vector3d hover_pos{0.0, 0.0, 0.0};    // 悬停点
        float hover_yaw = 0.0;                       // 无人机目标航向
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
        int control_mode;         // 无人机控制模式
        int last_control_mode;    // 上一时刻无人机控制模式
        uint16_t type_mask;       // 控制指令类型
        int safety_state;         // 安全标志，0代表正常，其他代表不正常
        bool get_rc_signal;       // 是否收到遥控器的信号
        bool check_cmd_timeout;   // 是否检查指令超时
        float cmd_timeout;        // 指令超时时间
        bool use_rc;              // 是否使用遥控器
        bool use_offset;          // 是否添加偏移
        ros::Time last_land_time; // 进入降落最后一阶段的时间戳
        ros::Time last_rc_time;   // 上一个rc控制时间点
    };
    SystemParams system_params;

    sunray_msgs::UAVControlCMD control_cmd;               // 当前时刻无人机控制指令（来自任务节点）
    sunray_msgs::UAVControlCMD last_control_cmd;          // 上一时刻无人机控制指令（来自任务节点）
    sunray_msgs::UAVState uav_state;                      // 当前时刻无人机状态（本节点发布）
    sunray_msgs::RcState rc_state;                        // 无人机遥控器状态（来自遥控器输入节点）
    mavros_msgs::PositionTarget local_setpoint;           // PX4的本地位置设定点（待发布）
    mavros_msgs::GlobalPositionTarget global_setpoint;    // PX4的全局位置设定点（待发布）
    mavros_msgs::AttitudeTarget att_setpoint;             // PX4的姿态设定点（待发布）
    float default_home_x, default_home_y, default_home_z; // 默认home点？

    bool rcState_cb; // 遥控器状态回调
    bool allow_lock; // 允许临时解锁 特殊模式下允许跳过解锁检查保持在CMD_CONTROL模式

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

    struct Waypoint_Params
    {
        bool wp_init = false;                         // 是否初始化
        bool wp_takeoff = false;                      // 是否起飞
        int wp_num = 0;                               // 航点数量 【最大数量10】
        int wp_type = 0;                              // 航点类型 【0：NED 1：经纬】
        int wp_end_type = 3;                          // 航点结束类型 【1: 悬停 2: 降落 3: 返航】
        int wp_yaw_type = 2;                          // 航点航向类型 【1: 固定值 2: 朝向下一个航点 3: 指向环绕点】
        int wp_index = 0;                             // 当前航点索引
        int wp_state = 0;                             // 航点状态 【1：解锁 2：起飞中 3：航点执行中 4:返航中 5:降落中 6: 结束】
        float wp_move_vel = 0.5;                      // 最大水平速度
        float z_height = 1.0;                         // 起飞和返航高度
        float wp_x_vel = 0.0;                         // 水平速度
        float wp_y_vel = 0.0;                         // 水平速度
        float wp_vel_p = 1;                           // 速度比例
        double wp_point_takeoff[3] = {0.0, 0.0, 0.0}; // 起飞点
        double wp_point_return[3] = {0.0, 0.0, 0.0};  // 返航点
        std::map<int, double[3]> wp_points;           // 航点
        ros::Time start_wp_time;                      // 上一个动作时间戳
    };
    Waypoint_Params wp_params;

    // 添加每个基础移动模式对应的 typemask的映射
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
            {sunray_msgs::UAVControlCMD::XyVelZPosYawBody, TypeMask::XY_VEL_Z_POS_YAW}};

    // 添加string的映射
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
            {sunray_msgs::UAVControlCMD::GlobalPos, "GlobalPos"},
            {sunray_msgs::UAVControlCMD::CTRL_XyzPos, "CTRL_XyzPos"},
            {sunray_msgs::UAVControlCMD::CTRL_Traj, "CTRL_Traj"},
            {sunray_msgs::UAVControlCMD::Takeoff, "Takeoff"},
            {sunray_msgs::UAVControlCMD::Land, "Land"},
            {sunray_msgs::UAVControlCMD::Hover, "Hover"},
            {sunray_msgs::UAVControlCMD::Waypoint, "Waypoint"},
            {sunray_msgs::UAVControlCMD::Return, "Return"}};

    // 添加string的映射
    std::map<int, std::string> modeMap =
        {
            {int(Control_Mode::INIT), "INIT"},
            {int(Control_Mode::RC_CONTROL), "RC_CONTROL"},
            {int(Control_Mode::CMD_CONTROL), "CMD_CONTROL"},
            {int(Control_Mode::LAND_CONTROL), "LAND_CONTROL"},
            {int(Control_Mode::WITHOUT_CONTROL), "WITHOUT_CONTROL"}};

    // 绑定高级模式对应的实现函数
    std::map<int, std::function<void()>> advancedModeFuncMap;

    // 订阅节点句柄
    ros::Subscriber setup_sub;        // 【订阅】无人机模式设置
    ros::Subscriber control_cmd_sub;  // 【订阅】控制指令订阅
    ros::Subscriber odom_state_sub;   // 【订阅】无人机位置状态订阅
    ros::Subscriber rc_state_sub;     // 【订阅】rc_control状态订阅
    ros::Subscriber px4_state_sub;    // 【订阅】无人机状态订阅 来自externalFusion节点
    ros::Subscriber uav_waypoint_sub; // 【订阅】无人机航点订阅

    // 发布节点
    ros::Publisher uav_control_pub;           // 【发布】控制指令发布
    ros::Publisher uav_state_pub;             // 【发布】无人机状态发布
    ros::Publisher px4_setpoint_local_pub;    // 【发布】无人机指令发布 NED
    ros::Publisher px4_setpoint_global_pub;   // 【发布】无人机指令发布 经纬度+海拔
    ros::Publisher px4_setpoint_attitude_pub; // 【发布】无人机指令发布 姿态+推力
    ros::Publisher goal_pub;                  // 【发布】发布一个目标点 来自外部控制指令
    // 服务节点
    ros::ServiceClient px4_arming_client;    // 【服务】px4解锁
    ros::ServiceClient px4_set_mode_client;  // 【服务】px4模式设置
    ros::ServiceClient px4_reboot_client;    // 【服务】px4重启
    ros::ServiceClient px4_emergency_client; // 【服务】px4紧急停止

    void printf_params();
    int safetyCheck();     // 安全检查
    void setArm(bool arm); // 设置解锁 0:上锁 1:解锁
    void set_auto_land();  // 调用px4 auto.land
    void reboot_px4();     // 重启
    void emergencyStop();  // 紧急停止出来
    void set_px4_flight_mode(std::string mode);
    void send_attitude_setpoint(Eigen::Vector4d &u_att);                                      // 设置模式
    void setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint);        // 【发布】发送控制指令
    void setpoint_global_pub(uint16_t type_mask, mavros_msgs::GlobalPositionTarget setpoint); // 【发布】发送控制指令
    void handle_cmd_control();                                                                // CMD_CONTROL模式下获取期望值
    void handle_rc_control();                                                                 // RC_CONTROL模式下获取期望值
    void handle_land_control();                                                               // LAND_CONTROL模式下获取期望值
    void set_desired_from_hover();                                                            // Hover模式下获取期望值
    void check_state();                                                                       // 安全检查 + 发布状态
    void set_hover_pos();                                                                     // 设置悬停位置
    void set_default_local_setpoint();                                                        // 设置默认期望值
    void set_default_global_setpoint();                                                       // 设置默认期望值
    void set_offboard_mode();                                                                 // 设置offboard模式
    void body2enu(double body_frame[2], double enu_frame[2], double yaw);                     // body坐标系转ned坐标系
    void rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg);                        // 遥控器状态回调函数
    void set_offboard_control(int mode);                                                      // 设置offboard控制模式
    void return_to_home();                                                                    // 返航模式实现
    void waypoint_mission();                                                                  // 航点任务实现
    void set_takeoff();                                                                       // 起飞模式实现
    void set_land();                                                                          // 降落模式实现
    float get_yaw_from_waypoint(int type, float point_x, float point_y);                      // 获取航点航向
    float get_vel_from_waypoint(float point_x, float point_y);                                // 获取航点速度
    void publish_goal();                                                                      // 发布规划点
    void pos_controller();
    // 回调函数
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void uav_setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg);
    void px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg);
    void waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg);

public:
    UAVControl() {};
    ~UAVControl() {};

    sunray_msgs::PX4State px4_state; // 当前时刻无人机状态（来自external_fusion_node）

    PosControlPID pos_controller_pid;

    void mainLoop();
    void show_ctrl_state(); // 打印状态
    void init(ros::NodeHandle &nh);
};