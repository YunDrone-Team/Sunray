#include "ros_msg_utils.h"
#include "type_mask.h"
#include "printf_format.h"

using namespace sunray_logger;

class mavros_control
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
    ros::Subscriber setup_sub;          // 无人机模式设置
    ros::Subscriber control_cmd_sub;    // 控制指令订阅
    ros::Subscriber odom_state_sub;     // 无人机位置状态订阅
    ros::Subscriber rc_state_sub;       // rc_control状态订阅
    ros::Subscriber px4_state_sub;      // 无人机状态订阅
    ros::Subscriber px4_battery_sub;    // 无人机电池状态订阅
    ros::Subscriber px4_odom_sub;       // 无人机里程计订阅
    ros::Subscriber px4_att_sub;        // 无人机imu姿态订阅
    ros::Subscriber px4_pos_target_sub; // px4目标订阅 位置 速度加 速度
    ros::Subscriber px4_att_target_sub; // 无人机姿态订阅

    // 发布节点
    ros::Publisher uav_control_pub;           // 控制指令发布
    ros::Publisher uav_state_pub;             // 无人机状态发布
    ros::Publisher px4_setpoint_local_pub;    // 无人机指令发布 NED
    ros::Publisher px4_setpoint_global_pub;   // 无人机指令发布 经纬度+海拔
    ros::Publisher px4_setpoint_attitude_pub; // 无人机指令发布 姿态+推力
    // 服务节点
    ros::ServiceClient px4_arming_client;    // px4解锁
    ros::ServiceClient px4_set_mode_client;  // px4模式设置
    ros::ServiceClient px4_reboot_client;    // px4重启
    ros::ServiceClient px4_emergency_client; // px4紧急停止
    // 定时器
    ros::Timer cmd_timer;   // 指令定时器
    ros::Timer task_timer;  // 任务定时器
    ros::Timer print_timer; // 状态定时器

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
        Eigen::Vector4d att_q{0.0, 0.0, 0.0, 0.0};        // 无人机当前姿态 四元数
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
        XyzPosYaw ,
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
        Waypoint,
        Return
    };

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

    std::map<int, std::string> modeMap =
        {
            {int(Control_Mode::INIT), "INIT"},
            {int(Control_Mode::RC_CONTROL), "RC_CONTROL"},
            {int(Control_Mode::CMD_CONTROL), "CMD_CONTROL"},
            {int(Control_Mode::LAND_CONTROL), "LAND_CONTROL"},
            {int(Control_Mode::WITHOUT_CONTROL), "WITHOUT_CONTROL"}};

    std::map<int, std::function<void()>> advancedModeFuncMap;

    int safetyCheck();              // 安全检查
    void setArm(bool arm);          // 设置解锁 0:上锁 1:解锁
    void reboot();                  // 重启
    void emergencyStop();           // 紧急停止
    void setMode(std::string mode); // 设置模式
    void setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint);
    void set_desired_from_cmd();
    void set_desired_from_rc();
    void set_desired_from_land();
    void set_desired_from_hover();
    void print_state(const ros::TimerEvent &event);
    void task_timer_callback(const ros::TimerEvent &event);
    void set_hover_pos();
    void set_default_setpoint();
    void set_offboard_mode();
    void body2ned(double body_xy[2], double ned_xy[2], double yaw);
    void rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg);
    void setset_offboard_control(int mode);
    void return_to_home();
    void waypoint_mission();
    void set_takeoff();
    void set_land();
    // 回调函数
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg);
    void odom_state_callback(const std_msgs::Bool::ConstPtr &msg);
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void px4_att_target_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg);

public:
    mavros_control() {};
    ~mavros_control() {};

    void mainLoop();
    void init(ros::NodeHandle &nh);
};