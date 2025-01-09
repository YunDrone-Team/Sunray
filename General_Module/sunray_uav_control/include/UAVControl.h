#include "ros_msg_utils.h"
#include "type_mask.h"
#include "printf_format.h"

using namespace sunray_logger;

class UAVControl
{
private:
    int uav_id;                                        // 无人机ID
    int control_mode;                                  // 控制模式
    int last_control_mode;                             // 控制模式
    int safety_state;                                  // 安全标志
    float takeoff_height;                              // 起飞高度
    float disarm_height;                               // 锁桨高度
    float land_speed;                                  // 降落速度
    float cmd_timeout;                                 // 指令超时时间
    float land_end_time;                               // 降落最后一段时间
    float land_end_speed;                              // 降落最后一段速度
    float odom_valid_timeout;                          // 外部定位超时时间
    float odom_valid_warming_time;                     // 外部定位超时警告
    bool check_cmd_timeout;                            // 是否检查指令超时
    bool odom_valid;                                   // 外部定位是否有效
    bool use_rc;                                       // 是否使用遥控器
    bool rcState_cb;                                   // 遥控器状态回调
    std::string uav_name;                              // 无人机名称
    std::string uav_ns;                                // 节点命名空间
    std::string topic_prefix;                          // 话题前缀
    std::string uav_prefix;                            // 无人机前缀
    sunray_msgs::UAVControlCMD control_cmd;            // 外部控制指令
    sunray_msgs::UAVControlCMD last_control_cmd;       // 上一刻控制指令
    sunray_msgs::UAVState uav_state;                   // 无人机状态
    sunray_msgs::UAVState uav_state_last;              // 上一刻无人机状态
    sunray_msgs::RcState rc_state;                     // rc_control状态
    mavros_msgs::PositionTarget local_setpoint;        // px4目标指令
    mavros_msgs::GlobalPositionTarget global_setpoint; // px4目标指令
    mavros_msgs::AttitudeTarget att_setpoint;          // px4目标指令
    ros::Time odom_valid_time;                         // 外部定位状态订阅时间
    // 订阅节点
    ros::Subscriber setup_sub;          // 【订阅】无人机模式设置
    ros::Subscriber control_cmd_sub;    // 【订阅】控制指令订阅
    ros::Subscriber odom_state_sub;     // 【订阅】无人机位置状态订阅
    ros::Subscriber rc_state_sub;       // 【订阅】rc_control状态订阅
    ros::Subscriber px4_state_sub;      // 【订阅】无人机状态订阅
    ros::Subscriber px4_battery_sub;    // 【订阅】无人机电池状态订阅
    ros::Subscriber px4_odom_sub;       // 【订阅】无人机里程计订阅
    ros::Subscriber px4_att_sub;        // 【订阅】无人机imu姿态订阅
    ros::Subscriber px4_pos_target_sub; // 【订阅】px4目标订阅 位置 速度加 速度
    ros::Subscriber px4_att_target_sub; // 【订阅】无人机姿态订阅
    ros::Subscriber uav_waypoint_sub;   // 【订阅】无人机航点订阅

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
    // 定时器
    ros::Timer cmd_timer;   // 【定时器】指令定时器
    ros::Timer task_timer;  // 【定时器】任务定时器
    ros::Timer print_timer; // 【定时器】状态定时器

    struct geo_fence // 地理围栏
    {
        float x_min = 5.0;
        float x_max = 5.0;
        float y_min = 5.0;
        float y_max = 5.0;
        float z_min = -0.1;
        float z_max = 2.0;
    };
    geo_fence uav_geo_fence;

    struct PX4State
    {
        bool connected = false;                           // 无人机连接状态
        bool armed = false;                               // 无人机解锁状态
        std::string mode{"unknown"};                      // 无人机模式
        float batt_volt = 0.0;                            // 无人机电池电压
        float batt_perc = 0.0;                            // 无人机电池电量
        float target_thrust = 0.0;                        // 无人机目标推力
        Eigen::Vector3d pos{-0.01, -0.01, -0.01};         // 无人机当前位置
        Eigen::Vector3d vel{-0.01, -0.01, -0.01};         // 无人机当前速度
        Eigen::Vector3d att{0.0, 0.0, 0.0};               // 无人机当前姿态 角度
        Eigen::Vector4d att_q{0.0, 0.0, 0.0, 0.0};        // 无人机当前姿态 四元数xyzw
        Eigen::Vector3d target_pos{0.0, 0.0, 0.0};        // 无人机目标位置
        Eigen::Vector3d target_vel{0.0, 0.0, 0.0};        // 无人机目标速度
        Eigen::Vector3d target_att{0.0, 0.0, 0.0};        // 无人机目标姿态 角度
        Eigen::Vector4d target_att_q{0.0, 0.0, 0.0, 0.0}; // 无人机目标姿态 四元数
    };
    PX4State px4_state;

    struct FlightParams
    {
        uint16_t type_mask = 0;                   // 控制指令类型
        float home_yaw = 0.0;                     // 起飞点航向
        float hover_yaw = 0.0;                    // 无人机目标航向
        float land_yaw = 0.0;                     // 降落点航向
        float max_vel_xy = 1.0;                   // 最大水平速度
        float max_vel_z = 1.0;                    // 最大垂直速度
        float max_vel_yaw = 1.5;                  // 最大偏航速度
        bool home_set = false;                    // 起飞点是否设置
        Eigen::Vector3d home_pos{0.0, 0.0, 0.0};  // 起飞点
        Eigen::Vector3d hover_pos{0.0, 0.0, 0.0}; // 悬停点
        Eigen::Vector3d land_pos{0.0, 0.0, 0.0};  // 降落点
        ros::Time last_land_time;                 // 最后停止时间
        ros::Time last_rc_time;                   // 上一个rc控制时间点
    };
    FlightParams flight_params;

    struct Waypoint_Params
    {
        bool wp_init = false;                         // 是否初始化
        bool wp_takeoff = false;                      // 是否起飞
        int wp_num = 0;                               // 航点数量 【最大数量10】
        int wp_type = 0;                              // 航点类型 【0：NED 1：经纬】
        int wp_end_type = 0;                          // 航点结束类型 【1: 悬停 2: 降落 3: 返航】
        int wp_yaw_type = 0;                          // 航点航向类型 【1: 固定值 1: 朝向下一个航点 2: 指向环绕点】
        int wp_index = 0;                             // 当前航点索引
        int wp_state = 0;                             // 航点状态 【1：解锁 2：起飞中 3：航点执行中 4:返航中 5:降落中 6: 结束】
        float wp_move_vel = 0.5;                      // 最大水平速度
        float z_height = 1.0;                         // 起飞和返航高度
        double wp_point_takeoff[3] = {0.0, 0.0, 0.0}; // 起飞点
        double wp_point_return[3] = {0.0, 0.0, 0.0};  // 返航点
        std::map<int, double[3]> wp_points;           // 航点
        float wp_x_vel = 0.0;                         // 水平速度
        float wp_y_vel = 0.0;                         // 水平速度
        float wp_vel_p = 1;                           // 速度比例
        ros::Time start_wp_time;                      // 上一个动作时间点
    };
    Waypoint_Params wp_params;

    enum Control_Mode // 无人机控制模式
    {
        INIT = 0,           // 初始模式
        RC_CONTROL = 1,     // 遥控器控制模式
        CMD_CONTROL = 2,    // 外部指令控制模式
        LAND_CONTROL = 3,   // 降落
        WITHOUT_CONTROL = 4 // 无控制
    };

    enum BaseMoveMode // 基础移动模式
    {
        XyzPos = 1,
        XyzVel,
        XyVelZPos,
        XyzPosYaw,
        XyzPosYawrate = 5,
        XyzVelYaw,
        XyzVelYawrate,
        XyVelZPosYaw,
        XyVelZPosYawrate,
        XyzPosVelYaw = 10,
        XyzPosVelYawrate,
        PosVelAccYaw,
        PosVelAccYawrate,
        XyzPosYawBody,
        XyzVelYawBody = 15,
        XyVelZPosYawBody,
        GlobalPos,
        Att = 18
    };

    enum AdvanceMoveMode // 高级移动模式
    {
        Takeoff = 100,
        Land = 101,
        Hover = 102,
        Waypoint = 103,
        Return = 104,
    };
    
    enum OtherMode // 其他模式
    {
        Point = 30,
    };

    // 添加每个基础移动模式对应的 typemask的映射
    std::map<int, uint16_t> moveModeMap =
        {
            {XyzPos, TypeMask::XYZ_POS},
            {XyzVel, TypeMask::XYZ_VEL},
            {XyVelZPos, TypeMask::XY_VEL_Z_POS},
            {XyzPosYaw, TypeMask::XYZ_POS_YAW},
            {XyzPosYawrate, TypeMask::XYZ_POS_YAWRATE},
            {XyzVelYaw, TypeMask::XYZ_VEL_YAW},
            {XyzVelYawrate, TypeMask::XYZ_VEL_YAWRATE},
            {XyVelZPosYaw, TypeMask::XY_VEL_Z_POS_YAW},
            {XyVelZPosYawrate, TypeMask::XY_VEL_Z_POS_YAWRATE},
            {XyzPosVelYaw, TypeMask::XYZ_POS_VEL_YAW},
            {XyzPosVelYawrate, TypeMask::XYZ_POS_VEL_YAWRATE},
            {PosVelAccYaw, TypeMask::POS_VEL_ACC_YAW},
            {PosVelAccYawrate, TypeMask::POS_VEL_ACC_YAWRATE},
            {XyzPosYawBody, TypeMask::XYZ_POS_YAW},
            {XyzVelYawBody, TypeMask::XYZ_VEL_YAW},
            {XyVelZPosYawBody, TypeMask::XY_VEL_Z_POS_YAW}};

    // 添加string的映射
    std::map<int, std::string> moveModeMapStr =
        {
            {XyzPos, "XyzPos"},
            {XyzVel, "XyzVel"},
            {XyVelZPos, "XyVelZPos"},
            {XyzPosYaw, "XyzPosYaw"},
            {XyzPosYawrate, "XyzPosYawrate"},
            {XyzVelYaw, "XyzVelYaw"},
            {XyzVelYawrate, "XyzVelYawrate"},
            {XyVelZPosYaw, "XyVelZPosYaw"},
            {XyVelZPosYawrate, "XyVelZPosYawrate"},
            {XyzPosVelYaw, "XyzPosVelYaw"},
            {XyzPosVelYawrate, "XyzPosVelYawrate"},
            {PosVelAccYaw, "PosVelAccYaw"},
            {PosVelAccYawrate, "PosVelAccYawrate"},
            {XyzPosYawBody, "XyzPosYawBody"},
            {XyzVelYawBody, "XyzVelYawBody"},
            {XyVelZPosYawBody, "XyVelZPosYawBody"},
            {GlobalPos, "GlobalPos"},
            {Att, "Att"},
            {Takeoff, "Takeoff"},
            {Land, "Land"},
            {Hover, "Hover"},
            {Waypoint, "Waypoint"},
            {Return, "Return"}};

    // 添加string的映射
    std::map<int, std::string> modeMap =
        {
            {int(Control_Mode::INIT), "INIT"},
            {int(Control_Mode::RC_CONTROL), "RC_CONTROL"},
            {int(Control_Mode::CMD_CONTROL), "CMD_CONTROL"},
            {int(Control_Mode::LAND_CONTROL), "LAND_CONTROL"},
            {int(Control_Mode::WITHOUT_CONTROL), "WITHOUT_CONTROL"}};

    std::map<int, std::function<void()>> advancedModeFuncMap;

    int safetyCheck();                                                                 // 安全检查
    void setArm(bool arm);                                                             // 设置解锁 0:上锁 1:解锁
    void reboot();                                                                     // 重启
    void emergencyStop();                                                              // 紧急停止出来
    void setMode(std::string mode);                                                    // 设置模式
    void setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint); // 【发布】发送控制指令
    void set_desired_from_cmd();                                                       // CMD_CONTROL模式下获取期望值
    void set_desired_from_rc();                                                        // RC_CONTROL模式下获取期望值
    void set_desired_from_land();                                                      // LAND_CONTROL模式下获取期望值
    void set_desired_from_hover();                                                     // Hover模式下获取期望值
    void print_state(const ros::TimerEvent &event);                                    // 打印状态
    void task_timer_callback(const ros::TimerEvent &event);                            // 任务定时器回调函数
    void set_hover_pos();                                                              // 设置悬停位置
    void set_default_setpoint();                                                       // 设置默认期望值
    void set_offboard_mode();                                                          // 设置offboard模式
    void body2ned(double body_xy[2], double ned_xy[2], double yaw);                    // body坐标系转ned坐标系
    void rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg);                 // 遥控器状态回调函数
    void set_offboard_control(int mode);                                               // 设置offboard控制模式
    void return_to_home();                                                             // 返航模式实现
    void waypoint_mission();                                                           // 航点任务实现
    void set_takeoff();                                                                // 起飞模式实现
    void set_land();                                                                   // 降落模式实现
    float get_yaw_from_waypoint(int type, float point_x, float point_y);               // 获取航点航向
    float get_vel_from_waypoint(float point_x, float point_y);                         // 获取航点速度
    void publish_goal();                                                               // 发布规划点
    // 回调函数
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg);
    void odom_state_callback(const sunray_msgs::ExternalOdom::ConstPtr &msg);
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);
    void waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg);

public:
    UAVControl() {};
    ~UAVControl() {};

    void mainLoop();
    void init(ros::NodeHandle &nh);
};