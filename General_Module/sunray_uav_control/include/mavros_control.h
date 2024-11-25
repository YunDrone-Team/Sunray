#include "ros_msg_utils.h"
#include "type_mask.h"

class mavros_control
{
private:
    int uav_id;
    int rc_mode;                                 // 遥控器模式 0:无 1:法可 2:自动
    float takeoff_height;                        // 起飞高度
    float disarm_height;                         // 锁桨高度
    float land_speed;                            // 降落速度
    float cmd_timeout;                           // 指令超时时间
    bool check_cmd_timeout;                      // 检查指令超时
    std::string uav_name;                        // 无人机名称
    std::string uav_ns;                          // 节点命名空间
    std::string topic_prefix;                    // 话题前缀
    sunray_msgs::UAVControlCMD control_cmd;      // 外部控制指令
    sunray_msgs::UAVControlCMD last_control_cmd; // 上一刻控制指令
    sunray_msgs::UAVState uav_state;             // 无人机状态
    sunray_msgs::UAVState uav_state_last;        // 上一刻无人机状态
    // 订阅节点
    ros::Subscriber setup_sub;       // 无人机模式设置
    ros::Subscriber control_cmd_sub; // 控制指令订阅
    ros::Subscriber px4_target_sub;  // px4目标订阅
    ros::Subscriber px4_state_sub;   // 无人机状态订阅
    ros::Subscriber px4_battery_sub; // 无人机电池状态订阅
    ros::Subscriber px4_odom_sub;    // 无人机里程计订阅
    ros::Subscriber px4_att_sub;     // 无人机姿态订阅
    // 发布节点
    ros::Publisher uav_control_pub;  // 控制指令发布
    ros::Publisher uav_state_pub;    // 无人机状态发布
    ros::Publisher px4_setpoint_pub; // 无人机指令发布
    // 服务节点
    ros::ServiceClient px4_arming_client;    // px4解锁
    ros::ServiceClient px4_set_mode_client;  // px4模式设置
    ros::ServiceClient px4_reboot_client;    // px4重启
    ros::ServiceClient px4_emergency_client; // px4紧急停止
    // 定时器
    ros::Timer cmd_timer; // 指令定时器

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
        bool connect = false;  // 无人机连接状态
        bool armed = false;    // 无人机解锁状态
        std::string mode;      // 无人机模式
        float batt_volt = 0.0; // 无人机电池电压
        float batt_perc = 0.0; // 无人机电池电量
        // 无人机当前位置
        float px = 0.0;
        float py = 0.0;
        float pz = 0.0;
        // 无人机当前速度
        float vx = 0.0;
        float vy = 0.0;
        float vz = 0.0;
        // 无人机当前姿态 角度
        float roll = 0.0;
        float pitch = 0.0;
        float yaw = 0.0;
        // 无人机当前姿态 四元数
        float qx = 0.0;
        float qy = 0.0;
        float qz = 0.0;
        float qw = 0.0;
        // 无人机目标位置
        float target_px = 0.0;
        float target_py = 0.0;
        float target_pz = 0.0;
        // 无人机目标速度
        float target_vx = 0.0;
        float target_vy = 0.0;
        float target_vz = 0.0;
        // 无人机目标姿态 角度
        float target_roll = 0.0;
        float target_pitch = 0.0;
        float target_yaw = 0.0;
        // 无人机目标姿态 四元数
        float target_qx = 0.0;
        float target_qy = 0.0;
        float target_qz = 0.0;
        float target_qw = 0.0;
        // 无人机目标推力
        float target_thrust = 0.0;
    };
    PX4State px4_state;

    enum Control_Mode // 无人机控制模式
    {
        INIT = 0,        // 初始模式
        RC_CONTROL = 1,  // 遥控器控制模式
        CMD_CONTROL = 2, // 外部指令控制模式
        LAND_CONTROL = 3 // 降落
    };

    enum moveMode // 无人机移动模式
    {
        TAKEOFF,
        LAND,
        HOVER,
        WAYPOINT,
        XYZ_POS_YAW = 1,
        XYZ_POS_YAWRATE,
        XYZ_POS_YAW_YAWRATE,
        XYZ_VEL_YAW,
        XYZ_VEL_YAWRATE,
        XYZ_VEL_YAW_YAWRATE,
        XY_VEL_Z_POS_YAW,
        XY_VEL_Z_POS_YAWRATE,
        XY_VEL_Z_POS_YAW_YAWRATE,
        XYZ_POS_VEL_YAW,
        XYZ_POS_VEL_YAWRATE,
        XYZ_POS_VEL_YAW_YAWRATE,
        POS_VEL_ACC_YAW,
        POS_VEL_ACC_YAWRATE,
        POS_VEL_ACC_YAW_YAWRATE,
        POS_VEL_ACC_YAW_YAWRATE,
        GLOBAL_POS,
        ATT
    }

    int
    saftyCheck();              // 安全检查
    void setArm(bool arm);     // 设置解锁 0:上锁 1:解锁
    void reboot();             // 重启
    void emergencyStop();      // 紧急停止
    void setMode(string mode); // 设置模式
    void setpoint_pub(uint16_t type_mask, geometry_msgs::PoseStamped setpoint);
    // 回调函数
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void setup_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);

public:
    mavros_control() {};
    ~mavros_control() {};

    void mainLoop();
    void init();
};

int mavros_control::saftyCheck()
{
    if (uav_state.position[0] < uav_geo_fence.x_min || uav_state.position[0] > uav_geo_fence.x_max ||
        uav_state.position[1] < uav_geo_fence.y_min || uav_state.position[1] > uav_geo_fence.y_max ||
        uav_state.position[2] < uav_geo_fence.z_min || uav_state.position[2] > uav_geo_fence.z_max)
    {
        return 1;
    }
    else if (!uav_state.odom_valid)
    {
        return 2;
    }
    else
    {
        return 0;
    }
}

void mavros_control::px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    px4_state.px = msg->pose.pose.position.x;
    px4_state.py = msg->pose.pose.position.y;
    px4_state.pz = msg->pose.pose.position.z;
    px4_state.vx = msg->twist.twist.linear.x;
    px4_state.vy = msg->twist.twist.linear.y;
    px4_state.vz = msg->twist.twist.linear.z;
}

void mavros_control::px4_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    px4_state.connected = msg->connected;
    px4_state.armed = msg->armed;
    px4_state.mode = msg->mode;
}

void mavros_control::px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    px4_state.batt_volt = msg->voltage;
    px4_state.batt_perc = msg->percentage * 100;
}

void mavros_control::px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    px4_state.qx = msg->orientation.x;
    px4_state.qy = msg->orientation.y;
    px4_state.qz = msg->orientation.z;
    px4_state.qw = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_state.roll = roll;
    px4_state.pitch = pitch;
    px4_state.yaw = yaw;
}

void mavros_control::setMode(string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}

void mavros_control::emergencyStop()
{
    mavros_msgs::CommandLong emergency_srv;
    emergency_srv.request.broadcast = false;
    emergency_srv.request.command = 400;
    emergency_srv.request.confirmation = 0;
    emergency_srv.request.param1 = 0.0;
    emergency_srv.request.param2 = 21196;
    emergency_srv.request.param3 = 0.0;
    emergency_srv.request.param4 = 0.0;
    emergency_srv.request.param5 = 0.0;
    emergency_srv.request.param6 = 0.0;
    emergency_srv.request.param7 = 0.0;
    px4_emergency_client.call(emergency_srv);
}

void mavros_control::reboot()
{
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;    // Reboot autopilot
    reboot_srv.request.param2 = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;
    px4_reboot_client.call(reboot_srv);
}

void mavros_control::setArm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    px4_arm_client.call(arm_cmd);
}

void mavros_control::control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg)
{
    control_cmd = *msg;
}

void mavros_control::setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg)
{
    if (msg->cmd == sunray_msgs::UAVSetup::ARM)
    {
        setArm(true);
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::DISARM)
    {
        setArm(false);
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::SET_PX4_MODE)
    {
        setMode(msg->px4_mode);
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::REBOOT_PX4)
    {
        reboot();
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::SET_CONTROL_MODE)
    {
        // 需要无遥控器控制一定要注释掉（ && uav_state.armed ），否则无法进入OFFBOARD模式
        // if (msg->control_state == "CMD_CONTROL" && uav_state.armed)
        if (msg->control_state == "CMD_CONTROL")
        {
            control_mode = Control_Mode::CMD_CONTROL;
            check_off = true;
        }
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::EMERGENCY_KILL)
    {
        enable_emergency_func();
        control_mode = Control_Mode::INIT;
        // 控制命令初始化,不初始化将影响setup接口切换command_control模式
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        set_landing_des = false;
    }
}

