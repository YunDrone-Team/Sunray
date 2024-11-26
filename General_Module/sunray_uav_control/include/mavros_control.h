#include "ros_msg_utils.h"
#include "type_mask.h"
#include "printf_format.h"

using namespace sunray_logger;

class mavros_control
{
private:
    int uav_id;                                        // 无人机ID
    int control_mode;                                  // 控制模式
    float takeoff_height;                              // 起飞高度
    float disarm_height;                               // 锁桨高度
    float land_speed;                                  // 降落速度
    float cmd_timeout;                                 // 指令超时时间
    bool check_cmd_timeout;                            // 检查指令超时
    std::string uav_name;                              // 无人机名称
    std::string uav_ns;                                // 节点命名空间
    std::string topic_prefix;                          // 话题前缀
    sunray_msgs::UAVControlCMD control_cmd;            // 外部控制指令
    sunray_msgs::UAVControlCMD last_control_cmd;       // 上一刻控制指令
    sunray_msgs::UAVState uav_state;                   // 无人机状态
    sunray_msgs::UAVState uav_state_last;              // 上一刻无人机状态
    mavros_msgs::PositionTarget local_setpoint;        // px4目标指令
    mavros_msgs::GlobalPositionTarget global_setpoint; // px4目标指令
    mavros_msgs::AttitudeTarget att_setpoint;          // px4目标指令
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
    ros::Timer cmd_timer;   // 指令定时器
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
        bool connected = false;       // 无人机连接状态
        bool armed = false;           // 无人机解锁状态
        std::string mode;             // 无人机模式
        float batt_volt = 0.0;        // 无人机电池电压
        float batt_perc = 0.0;        // 无人机电池电量
        float target_thrust = 0.0;    // 无人机目标推力
        Eigen::Vector3d pos;          // 无人机当前位置
        Eigen::Vector3d vel;          // 无人机当前速度
        Eigen::Vector3d att;          // 无人机当前姿态 角度
        Eigen::Vector4d att_q;        // 无人机当前姿态 四元数
        Eigen::Vector3d target_pos;   // 无人机目标位置
        Eigen::Vector3d target_vel;   // 无人机目标速度
        Eigen::Vector3d target_att;   // 无人机目标姿态 角度
        Eigen::Vector4d target_att_q; // 无人机目标姿态 四元数
    };
    PX4State px4_state;

    struct FlightParams
    {
        uint16_t type_mask = 0;    // 控制指令类型
        float home_yaw = 0.0;      // 起飞点航向
        float hover_yaw = 0.0;     // 无人机目标航向
        Eigen::Vector3d home_pos;  // 起飞点
        Eigen::Vector3d hover_pos; // 悬停点
    };
    FlightParams flight_params;

    TypeMask Mask;

    enum Control_Mode // 无人机控制模式
    {
        INIT = 0,           // 初始模式
        RC_CONTROL = 1,     // 遥控器控制模式
        CMD_CONTROL = 2,    // 外部指令控制模式
        LAND_CONTROL = 3,   // 降落
        WITHOUT_CONTROL = 4 // 无控制
    };

    enum moveMode // 无人机移动模式
    {
        Takeoff = 1,
        Land,
        Hover,
        Waypoint,
        XyzPosYaw,
        XyzPosYawrate,
        XyzPosYawYawrate,
        XyzVelYaw,
        XyzVelYawrate,
        XyzVelYawYawrate,
        XyVelZPosYaw,
        XyVelZPosYawrate,
        XyVelZPosYawYawrate,
        XyzPosVelYaw,
        XyzPosVelYawrate,
        XyzPosVelYawYawrate,
        PosVelAccYaw,
        PosVelAccYawrate,
        PosVelAccYawYawrate,
        GlobalPos,
        Att
    };

    int
    saftyCheck();                   // 安全检查
    void setArm(bool arm);          // 设置解锁 0:上锁 1:解锁
    void reboot();                  // 重启
    void emergencyStop();           // 紧急停止
    void setMode(std::string mode); // 设置模式
    void setpoint_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint);
    void set_desired_from_cmd();
    void set_desired_from_rc();
    void set_desired_from_land();
    void set_desired_from_hover();
    void print_state(const ros::TimerEvent &event);
    void set_hover_pos();
    void set_default_setpoint();
    void set_offboard_mode();
    // 回调函数
    void control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg);
    void setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg);
    void px4_state_callback(const mavros_msgs::State::ConstPtr &msg);
    void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void px4_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg);

public:
    mavros_control() {};
    ~mavros_control() {};

    void mainLoop();
    void init(ros::NodeHandle &nh);
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
    px4_state.pos[0] = msg->pose.pose.position.x;
    px4_state.pos[1] = msg->pose.pose.position.y;
    px4_state.pos[2] = msg->pose.pose.position.z;
    px4_state.vel[0] = msg->twist.twist.linear.x;
    px4_state.vel[0] = msg->twist.twist.linear.y;
    px4_state.vel[0] = msg->twist.twist.linear.z;
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
    px4_state.att_q[0] = msg->orientation.x;
    px4_state.att_q[2] = msg->orientation.y;
    px4_state.att_q[2] = msg->orientation.z;
    px4_state.att_q[3] = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    px4_state.att[0] = roll;
    px4_state.att[1] = pitch;
    px4_state.att[2] = yaw;
}

void mavros_control::px4_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_state.target_pos[0] = msg->position.x;
    px4_state.target_pos[1] = msg->position.y;
    px4_state.target_pos[2] = msg->position.z;
    px4_state.target_vel[0] = msg->velocity.x;
    px4_state.target_vel[1] = msg->velocity.y;
    px4_state.target_vel[2] = msg->velocity.z;
}

void mavros_control::setMode(std::string mode)
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
    px4_arming_client.call(arm_cmd);
    if (arm)
    {

        if (arm_cmd.response.success)
        {
            std::cout << "Arming success!" << std::endl;
        }
        else
        {
            std::cout << "Arming failed!" << std::endl;
        }
    }
    else
    {
        if (arm_cmd.response.success)
        {
            std::cout << "Disarming success!" << std::endl;
        }
        else
        {
            std::cout << "Disarming failed!" << std::endl;
        }
    }
    arm_cmd.request.value = arm;
}

void mavros_control::control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg)
{
    control_cmd = *msg;
    std::cout << "control_cmd: " << control_cmd << std::endl;
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
        if (msg->control_state == "INIT")
        {
            control_mode = Control_Mode::INIT;
        }
        else if (msg->control_state == "CMD_CONTROL")
        {
            set_offboard_mode();
            control_mode = Control_Mode::CMD_CONTROL;
        }
        else if (msg->control_state == "RC_CONTROL")
        {
            control_mode = Control_Mode::RC_CONTROL;
        }
        else if (msg->control_state == "LAND_CONTROL")
        {
            control_mode = Control_Mode::LAND_CONTROL;
        }
        else if (msg->control_state == "WITHOUT_CONTROL")
        {
            control_mode = Control_Mode::WITHOUT_CONTROL;
        }
        else
        {
            std::cout << "Unknown control state!" << std::endl;
        }
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::EMERGENCY_KILL)
    {
        emergencyStop();
        control_mode = Control_Mode::INIT;
    }
}

void mavros_control::set_desired_from_cmd()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp == last_control_cmd.header.stamp;
    if (control_cmd.cmd == Takeoff)
    {
        if (new_cmd)
        {
            set_default_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = flight_params.home_pos[2] + takeoff_height;
            local_setpoint.yaw = flight_params.home_yaw;
            flight_params.type_mask = Mask.XYZ_POS_YAW;
        }
    }
    else if (control_cmd.cmd == Land)
    {
    }
    else if (control_cmd.cmd == Hover)
    {
        if (new_cmd)
        {
            set_default_setpoint();
            set_hover_pos();
            local_setpoint.position.x = flight_params.hover_pos[0];
            local_setpoint.position.y = flight_params.hover_pos[1];
            local_setpoint.position.z = flight_params.hover_pos[2];
            local_setpoint.yaw = flight_params.hover_yaw;
            flight_params.type_mask = Mask.XYZ_POS_YAW;
        }
    }
    else if (control_cmd.cmd == XyzPosYaw)
    {
        if (new_cmd)
        {
            set_default_setpoint();
            local_setpoint.position.x = control_cmd.desired_pos[0];
            local_setpoint.position.y = control_cmd.desired_pos[1];
            local_setpoint.position.z = control_cmd.desired_pos[2];
            local_setpoint.yaw = control_cmd.desired_yaw;
            flight_params.type_mask = Mask.XYZ_POS_YAW;
        }
    }
    else
    {
        std::cout << "Unknown command!" << std::endl;
        if (new_cmd)
        {
            set_default_setpoint();
            set_hover_pos();
            local_setpoint.position.x = flight_params.hover_pos[0];
            local_setpoint.position.y = flight_params.hover_pos[1];
            local_setpoint.position.z = flight_params.hover_pos[2];
            local_setpoint.yaw = flight_params.hover_yaw;
            flight_params.type_mask = Mask.XYZ_POS_YAW;
        }
    }
    setpoint_pub(flight_params.type_mask, local_setpoint);
    last_control_cmd = control_cmd;
}

void mavros_control::set_desired_from_rc()
{
}

void mavros_control::set_desired_from_land()
{
}

void mavros_control::set_desired_from_hover()
{
}

void mavros_control::print_state(const ros::TimerEvent &event)
{
    Logger::print_color(int(LogColor::blue), "Control Mode", control_mode);
}

void mavros_control::set_hover_pos()
{
    flight_params.hover_pos = px4_state.pos;
    flight_params.hover_yaw = px4_state.att[2];
}

void mavros_control::setpoint_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    px4_setpoint_pub.publish(setpoint);
}

void mavros_control::set_default_setpoint()
{
    local_setpoint.header.stamp = ros::Time::now();
    local_setpoint.header.frame_id = "map";
    local_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_setpoint.type_mask = Mask.NONE_TYPE;
    local_setpoint.position.x = 0;
    local_setpoint.position.y = 0;
    local_setpoint.position.z = 0;
    local_setpoint.velocity.x = 0;
    local_setpoint.velocity.y = 0;
    local_setpoint.velocity.z = 0;
    local_setpoint.acceleration_or_force.x = 0;
    local_setpoint.acceleration_or_force.y = 0;
    local_setpoint.acceleration_or_force.z = 0;
    local_setpoint.yaw = 0;
    local_setpoint.yaw_rate = 0;
}

void mavros_control::set_offboard_mode()
{
    set_default_setpoint();
    local_setpoint.velocity.x = 0.0;
    local_setpoint.velocity.y = 0.0;
    local_setpoint.velocity.z = 0.0;
    setpoint_pub(Mask.XYZ_VEL, local_setpoint);
    control_cmd.cmd = Hover;
    control_cmd.header.stamp = ros::Time::now();
    setMode("OFFBOARD");
}