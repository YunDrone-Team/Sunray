#include "UAVControl.h"

void UAVControl::init(ros::NodeHandle &nh)
{
    uav_ns = ros::this_node::getName();
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀
    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/" + uav_name + std::to_string(uav_id);
    // 【参数】飞行相关参数
    nh.param<float>("flight_param/Takeoff_height", flight_params.takeoff_height, 1.0); // 【参数】默认起飞高度
    nh.param<float>("flight_param/Disarm_height", flight_params.disarm_height, 0.2);   // 【参数】降落时自动上锁高度
    nh.param<float>("flight_param/Land_speed", flight_params.land_speed, 0.2);         // 【参数】降落速度
    nh.param<float>("flight_param/land_end_time", flight_params.land_end_time, 1.0);   // 【参数】降落最后一阶段时间
    nh.param<float>("flight_param/land_end_speed", flight_params.land_end_speed, 0.3); // 【参数】降落最后一阶段速度
    nh.param<int>("flight_param/land_type", flight_params.land_type, 0);               // 【参数】降落类型 【0:到达指定高度后锁桨 1:使用px4 auto.land】
    nh.param<float>("flight_param/home_x", default_home_x, 0.0);                       // 【参数】默认home点 在起飞后运行程序时需要
    nh.param<float>("flight_param/home_y", default_home_y, 0.0);                       // 【参数】默认home点 在起飞后运行程序时需要
    nh.param<float>("flight_param/home_z", default_home_z, 0.0);                       // 【参数】默认home点 在起飞后运行程序时需要
    // 【参数】地理围栏
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0); // 【参数】地理围栏最小x坐标
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);  // 【参数】地理围栏最大x坐标
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0); // 【参数】地理围栏最小y坐标
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);  // 【参数】地理围栏最大y坐标
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);  // 【参数】地理围栏最小z坐标
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);   // 【参数】地理围栏最大z坐标
    // 【参数】运行系统相关参数
    nh.param<bool>("system_params/check_cmd_timeout", system_params.check_cmd_timeout, true); // 【参数】是否检查命令超时
    nh.param<float>("system_params/cmd_timeout", system_params.cmd_timeout, 2.0);             // 【参数】命令超时时间
    nh.param<bool>("system_params/use_rc_control", system_params.use_rc, true);               // 【参数】是否使用遥控器控制
    nh.param<bool>("system_params/use_offset", system_params.use_offset, false);              // 【参数】是否使用位置偏移

    // 【订阅】无人机PX4模式 - 飞控 -> mavros -> 本节点
    px4_state_sub = nh.subscribe<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state",
                                                        10, &UAVControl::px4_state_callback, this);
    // 【订阅】无人机控制指令 - 外部节点 -> 本节点
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd",
                                                               10, &UAVControl::control_cmd_callback, this);
    // 【订阅】无人机设置指令 - 外部节点 -> 本节点
    setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup",
                                                    1, &UAVControl::uav_setup_callback, this);
    // 【订阅】遥控器数据 -- 飞控 -> mavros -> rc_input -> 本节点
    rc_state_sub = nh.subscribe<sunray_msgs::RcState>(uav_name + "/sunray/rc_state", 1,
                                                      &UAVControl::rc_state_callback, this);
    // 【订阅】无人机航点数据 -- 外部节点 -> 本节点
    uav_waypoint_sub = nh.subscribe<sunray_msgs::UAVWayPoint>(uav_name + "/sunray/uav_waypoint", 1,
                                                              &UAVControl::waypoint_callback, this);
    // 【发布】PX4位置环控制指令（包括期望位置、速度、加速度等接口，坐标系:ENU系） - 本节点 -> mavros -> 飞控
    px4_setpoint_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 1);
    // 【发布】PX4全局位置控制指令（包括期望经纬度等接口 坐标系:WGS84坐标系）- 本节点 -> mavros -> 飞控
    px4_setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(uav_name + "/mavros/setpoint_raw/global", 1);
    // 【发布】PX4姿态环控制指令（包括期望姿态等接口）- 本节点 -> mavros -> 飞控
    px4_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 1);
    // 【发布】无人机状态（接收到vision_pose节点的无人机状态，加上无人机控制模式，重新发布出去）- 本节点 -> 其他控制&任务节点
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 1);
    // 【发布】 一个geometry_msgs::PoseStamped类型的消息，用于指定规划目标位置，与控制节点无关 - 本节点 -> 其他控制&任务节点
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_" + std::to_string(uav_id), 1);

    // 【服务】PX4解锁/上锁指令 -- 本节点 -> mavros -> 飞控
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
    // 【服务】PX4修改PX4飞行模式指令 -- 本节点 -> mavros -> 飞控
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    // 【服务】PX4紧急上锁服务(KILL) -- 本节点 -> mavros -> 飞控
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_name + "/mavros/cmd/command");
    // 【服务】重启PX4飞控 -- 本节点 -> mavros -> 飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_name + "/mavros/cmd/command");

    // 【定时器】定时打印无人机状态
    print_timer = nh.createTimer(ros::Duration(1), &UAVControl::print_state, this);

    // 初始化各个部分参数
    system_params.control_mode = Control_Mode::INIT;
    system_params.last_control_mode = Control_Mode::INIT;
    system_params.type_mask = 0;
    system_params.safety_state = -1;
    rcState_cb = false;
    allow_lock = false;
    flight_params.home_pos[0] = default_home_x;
    flight_params.home_pos[1] = default_home_y;
    flight_params.home_pos[2] = default_home_z;

    // 绑定高级模式对应的实现函数
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Takeoff] = std::bind(&UAVControl::set_takeoff, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Land] = std::bind(&UAVControl::set_land, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Hover] = std::bind(&UAVControl::set_desired_from_hover, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Waypoint] = std::bind(&UAVControl::waypoint_mission, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Return] = std::bind(&UAVControl::return_to_home, this);
}

// 检查当前模式 并进入对应的处理函数中
void UAVControl::mainLoop()
{
    check_state();
    // 无人机控制状态机：控制模式由遥控器话题（遥控器拨杆）进行切换
    switch (system_params.control_mode)
    {
    // 初始模式：保持PX4在定点模式，此时PX4不接收来自机载电脑的任何指令
    case Control_Mode::INIT:
        // 无人机在未解锁状态且处于非定点模式时切换到定点模式
        if (!px4_state.armed && uav_state.mode != "POSCTL")
        {
            set_px4_flight_mode("POSCTL");
        }
        break;

    // 遥控器控制模式（RC_CONTROL）
    case Control_Mode::RC_CONTROL:
        // 在RC_CONTROL模式下，控制程序根据遥控器的摇杆来控制无人机移动
        // 类似于PX4的定点模式，只不过此时PX4为OFFBOARD模式（PX4的控制指令来自机载电脑）
        handle_rc_control();
        break;

    // CMD控制模式（CMD_CONTROL）：根据"/sunray/uav_control_cmd"话题的控制指令来控制无人机移动（二次开发一般使用这个模式）
    case Control_Mode::CMD_CONTROL:
        handle_cmd_control();
        break;

    // 降落控制模式（LAND_CONTROL）：当前位置原地降落，降落后会自动上锁
    case Control_Mode::LAND_CONTROL:
        handle_land_control();
        break;

    // 无控制模式（WITHOUT_CONTROL）：do nothing
    case Control_Mode::WITHOUT_CONTROL:
        break;

    default:
        set_desired_from_hover();
        setpoint_local_pub(system_params.type_mask, local_setpoint);
        break;
    }
    system_params.last_control_mode = system_params.control_mode;
}

// 安全检查 是否超出地理围栏 外部定位是否有效
int UAVControl::safetyCheck()
{
    // 如果超出地理围栏，则返回1
    if (px4_state.position[0] < uav_geo_fence.x_min ||
        px4_state.position[0] > uav_geo_fence.x_max ||
        px4_state.position[1] < uav_geo_fence.y_min ||
        px4_state.position[1] > uav_geo_fence.y_max ||
        px4_state.position[2] < uav_geo_fence.z_min ||
        px4_state.position[2] > uav_geo_fence.z_max)
    {
        return 1;
    }
    // 如果外部定位失效，则返回2
    if (!system_params.odom_valid)
    {
        return 2;
    }
    return 0;
}

// 无人机状态回调
void UAVControl::px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg)
{
    system_params.location_source = msg->external_odom.external_source;
    system_params.odom_valid = msg->external_odom.odom_valid;
    if (!px4_state.armed && msg->armed)
    {
        // 每一次解锁时，将无人机的解锁位置设置为home点
        flight_params.home_pos[0] = px4_state.position[0];
        flight_params.home_pos[1] = px4_state.position[1];
        flight_params.home_pos[2] = px4_state.position[2];
        flight_params.home_yaw = px4_state.attitude[2];
        flight_params.set_home = true;
        Logger::info("Home position set to: ", flight_params.home_pos[0], flight_params.home_pos[1], flight_params.home_pos[2]);
    }

    px4_state.connected = msg->connected;
    px4_state.armed = msg->armed;
    px4_state.mode = msg->mode;

    if (flight_params.set_home && !px4_state.armed)
    {
        flight_params.set_home = false;
    }

    // 更新无人机状态
    px4_state = *msg; 
    // 如果使用相对坐标，则将当前位置转换为相对坐标 仅显示用
    if(system_params.use_offset)
    {
        flight_params.relative_pos[0] = px4_state.position[0] - flight_params.home_pos[0];
        flight_params.relative_pos[1] = px4_state.position[1] - flight_params.home_pos[1];
        flight_params.relative_pos[2] = px4_state.position[2] - flight_params.home_pos[2];
    }
}

// 遥控器状态回调
void UAVControl::rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg)
{
    rcState_cb = true;
    rc_state = *msg;
    // 解锁与上锁通道判断
    if (rc_state.arm_state == 2)
    {
        setArm(true);
    }
    else if (rc_state.arm_state == 1)
    {
        setArm(false);
    }
    // 状态机模式通道判断 1 - INIT 2 - RC_CONTROL 3 - CMD_CONTROL
    if (rc_state.mode_state == 1)
    {
        system_params.control_mode = Control_Mode::INIT;
        Logger::warning("Switch to INIT mode with rc");
    }
    else if (rc_state.mode_state == 2)
    {
        Logger::warning("Switch to RC_CONTROL mode with rc");
        set_offboard_control(Control_Mode::RC_CONTROL);
    }
    else if (rc_state.mode_state == 3)
    {
        Logger::warning("Switch to CMD_CONTROL mode with rc");
        set_offboard_control(Control_Mode::CMD_CONTROL);
    }
    // 降落通道判断
    if (rc_state.land_state == 1)
    {
        Logger::warning("Switch to LAND_CONTROL mode with rc");
        set_land();
        // set_offboard_control(Control_Mode::LAND_CONTROL);
    }
    // 紧急停止通道判断
    if (rc_state.kill_state == 1)
    {
        Logger::error("Emergency Stop with rc");
        emergencyStop();
    }
}

// 设置PX4飞行模式
void UAVControl::set_px4_flight_mode(std::string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}

// KILL PX4
void UAVControl::emergencyStop()
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

    system_params.control_mode = Control_Mode::INIT; // 紧急停止后，切换到初始化模式
    Logger::error("Emergency Stop!");
}

// 重启PX4
void UAVControl::reboot_px4()
{
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;    // Reboot autopilot
    reboot_srv.request.param2 = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;
    px4_reboot_client.call(reboot_srv);
    Logger::error("Reboot!");
}

// PX4解锁/上锁
void UAVControl::setArm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    px4_arming_client.call(arm_cmd);
    if (arm)
    {
        // 解锁
        if (arm_cmd.response.success)
        {
            Logger::warning("Arming success!");
        }
        else
        {
            Logger::warning("Arming failed!");
        }
    }
    else
    {
        // 上锁
        if (arm_cmd.response.success)
        {
            Logger::warning("Disarming success!");
        }
        else
        {
            Logger::warning("Disarming failed!");
        }
    }
    arm_cmd.request.value = arm;
}

void UAVControl::set_auto_land()
{
    // 进入px4 auto.land模式
    set_px4_flight_mode("AUTO.LAND");
}

// 控制指令回调
void UAVControl::control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg)
{
    control_cmd = *msg;
    control_cmd.header.stamp = ros::Time::now();
    // 特殊模式单独判断 （如：紧急停止 航点任务）
    // 航点模式直接自动切换到CMD_CONTROL模式
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Waypoint)
    {
        // 允许就算没有遥控器的情况下临时自动解锁
        allow_lock = true;
        system_params.control_mode = Control_Mode::CMD_CONTROL;
    }
    else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Point)
    {
        // 发布规划点 与程序本身状态无关
        publish_goal();
        // 发布完成后要切换回之前的状态
        control_cmd = last_control_cmd;
    }
    else
    {
        // 其他模式需要手动解锁
        allow_lock = false; 
    }
}

// 回调函数：接收无人机设置指令
void UAVControl::uav_setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg)
{
    switch (msg->cmd)
    {
    // 设置指令：解锁无人机
    case sunray_msgs::UAVSetup::ARM:
        setArm(true);
        break;
    // 设置指令：无人机上锁
    case sunray_msgs::UAVSetup::DISARM:
        setArm(false);
        break;
    // 设置指令：设置PX4模式，需要配合px4_mode进行设置
    case sunray_msgs::UAVSetup::SET_PX4_MODE:
        set_px4_flight_mode(msg->px4_mode);
        break;
    // 设置指令：重启PX4飞控
    case sunray_msgs::UAVSetup::REBOOT_PX4:
        reboot_px4();
        break;
    // 设置指令： 无人机紧急上锁
    case sunray_msgs::UAVSetup::EMERGENCY_KILL:
        emergencyStop();
        system_params.control_mode = Control_Mode::INIT;
        break;
    // 设置指令：设置无人机控制模式，需要配合control_mode进行设置
    case sunray_msgs::UAVSetup::SET_CONTROL_MODE:
        if (msg->control_mode == "INIT")
        {
            system_params.control_mode = Control_Mode::INIT;
            Logger::warning("Switch to INIT mode with cmd");
        }
        else if (msg->control_mode == "RC_CONTROL")
        {
            if (system_params.safety_state == 0)
            {
                Logger::warning("Switch to RC_CONTROL mode with cmd");
                set_offboard_control(Control_Mode::RC_CONTROL);
            }
            else if (system_params.safety_state != 0)
            {
                Logger::error("Safety state error, cannot switch to RC_CONTROL mode!");
            }
        }
        else if (msg->control_mode == "CMD_CONTROL")
        {
            if (system_params.safety_state == 0)
            {
                set_offboard_control(Control_Mode::CMD_CONTROL);
                Logger::warning("Switch to CMD_CONTROL mode with cmd");
            }
            else if (system_params.safety_state != 0)
            {
                Logger::error("Safety state error, cannot switch to CMD_CONTROL mode");
            }
        }
        else if (msg->control_mode == "LAND_CONTROL")
        {
            set_land();
        }
        else if (msg->control_mode == "WITHOUT_CONTROL")
        {
            system_params.control_mode = Control_Mode::WITHOUT_CONTROL;
        }
        else
        {
            Logger::error("Unknown control state!");
        }
        break;
    default:
        break;
    }
}

void UAVControl::waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg)
{
    // 如果当前模式处于航点模式则不允许赋值
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Waypoint)
    {
        Logger::error("Waypoint mode is not allowed to be set when the current mode is waypoint mode!");
        return;
    }
    wp_params.wp_takeoff = msg->wp_takeoff;
    wp_params.wp_move_vel = msg->wp_move_vel;
    wp_params.wp_vel_p = msg->wp_vel_p;
    wp_params.z_height = msg->z_height;
    wp_params.wp_type = msg->wp_type;
    wp_params.wp_end_type = msg->wp_end_type;
    wp_params.wp_yaw_type = msg->wp_yaw_type;
    wp_params.wp_num = msg->wp_num;
    wp_params.wp_points[0][0] = msg->wp_point_1[0];
    wp_params.wp_points[0][1] = msg->wp_point_1[1];
    wp_params.wp_points[0][2] = msg->wp_point_1[2];
    wp_params.wp_points[0][3] = msg->wp_point_1[3];
    wp_params.wp_points[1][0] = msg->wp_point_2[0];
    wp_params.wp_points[1][1] = msg->wp_point_2[1];
    wp_params.wp_points[1][2] = msg->wp_point_2[2];
    wp_params.wp_points[1][3] = msg->wp_point_2[3];
    wp_params.wp_points[2][0] = msg->wp_point_3[0];
    wp_params.wp_points[2][1] = msg->wp_point_3[1];
    wp_params.wp_points[2][2] = msg->wp_point_3[2];
    wp_params.wp_points[2][3] = msg->wp_point_3[3];
    wp_params.wp_points[3][0] = msg->wp_point_4[0];
    wp_params.wp_points[3][1] = msg->wp_point_4[1];
    wp_params.wp_points[3][2] = msg->wp_point_4[2];
    wp_params.wp_points[3][3] = msg->wp_point_4[3];
    wp_params.wp_points[4][0] = msg->wp_point_5[0];
    wp_params.wp_points[4][1] = msg->wp_point_5[1];
    wp_params.wp_points[4][2] = msg->wp_point_5[2];
    wp_params.wp_points[4][3] = msg->wp_point_5[3];
    wp_params.wp_points[5][0] = msg->wp_point_6[0];
    wp_params.wp_points[5][1] = msg->wp_point_6[1];
    wp_params.wp_points[5][2] = msg->wp_point_6[2];
    wp_params.wp_points[5][3] = msg->wp_point_6[3];
    wp_params.wp_points[6][0] = msg->wp_point_7[0];
    wp_params.wp_points[6][1] = msg->wp_point_7[1];
    wp_params.wp_points[6][2] = msg->wp_point_7[2];
    wp_params.wp_points[6][3] = msg->wp_point_7[3];
    wp_params.wp_points[7][0] = msg->wp_point_8[0];
    wp_params.wp_points[7][1] = msg->wp_point_8[1];
    wp_params.wp_points[7][2] = msg->wp_point_8[2];
    wp_params.wp_points[7][3] = msg->wp_point_8[3];
    wp_params.wp_points[8][0] = msg->wp_point_9[0];
    wp_params.wp_points[8][1] = msg->wp_point_9[1];
    wp_params.wp_points[8][2] = msg->wp_point_9[2];
    wp_params.wp_points[8][3] = msg->wp_point_9[3];
    wp_params.wp_points[9][0] = msg->wp_point_10[0];
    wp_params.wp_points[9][1] = msg->wp_point_10[1];
    wp_params.wp_points[9][2] = msg->wp_point_10[2];
    wp_params.wp_points[9][3] = msg->wp_point_10[3];
    // 环绕点的值在最后一位获取
    wp_params.wp_points[10][0] = msg->wp_circle_point[0];
    wp_params.wp_points[10][1] = msg->wp_circle_point[1];

    wp_params.wp_init = true;
    Logger::warning("Waypoint setup success!");
}

// 打印状态
void UAVControl::print_state(const ros::TimerEvent &event)
{
    Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_name, "<<<<<<<<<<<<<<<");
    if (px4_state.connected)
    {
        Logger::print_color(int(LogColor::green), "CONNECTED:", "TRUE");
        if (px4_state.armed)
            Logger::print_color(int(LogColor::green), "MODE:", LOG_BLUE, px4_state.mode, "ARMED");
        else
            Logger::print_color(int(LogColor::green), "MODE:", LOG_BLUE, px4_state.mode, LOG_RED, "DISARMED");
        Logger::print_color(int(LogColor::green), "BATTERY:", px4_state.battery_state, "[V]", px4_state.battery_percentage, "[%]");
        // PX4回调位置 ekf数据
        Logger::print_color(int(LogColor::blue), "PX4 POS(receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.position[0],
                            px4_state.position[1],
                            px4_state.position[2],
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            px4_state.velocity[0],
                            px4_state.velocity[1],
                            px4_state.velocity[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            px4_state.attitude[0] / M_PI * 180,
                            px4_state.attitude[1] / M_PI * 180,
                            px4_state.attitude[2] / M_PI * 180,
                            "[deg]");
    }
    else
        Logger::print_color(int(LogColor::red), "CONNECTED:", "FALSE");
    Logger::color_no_del(int(LogColor::green), "Control Mode [", LOG_BLUE, modeMap[system_params.control_mode], LOG_GREEN, "]");
    if (system_params.control_mode == Control_Mode::CMD_CONTROL || system_params.control_mode == Control_Mode::RC_CONTROL)
    {
        Logger::color_no_del(int(LogColor::green), "Move Mode [", LOG_BLUE, moveModeMapStr[control_cmd.cmd], LOG_GREEN, "]");
        Logger::print_color(int(LogColor::blue), "PX4 TARGET (receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.pos_setpoint[0],
                            px4_state.pos_setpoint[1],
                            px4_state.pos_setpoint[2],
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            px4_state.vel_setpoint[0],
                            px4_state.vel_setpoint[1],
                            px4_state.vel_setpoint[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            px4_state.att_setpoint[0],
                            px4_state.att_setpoint[1],
                            px4_state.att_setpoint[2],
                            "[m/s]");

        if (system_params.use_offset)
        {
            // 打印相对位置
            Logger::print_color(int(LogColor::blue), "POS(relative to home)");
            Logger::print_color(int(LogColor::green), "Relative POS[X Y Z]:",
                                flight_params.relative_pos[0],
                                flight_params.relative_pos[1],
                                flight_params.relative_pos[2],
                                "[m]");
            Logger::print_color(int(LogColor::blue), "PX4 TARGET (relative to home)");
            Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                                px4_state.pos_setpoint[0] - flight_params.home_pos[0],
                                px4_state.pos_setpoint[1] - flight_params.home_pos[1],
                                px4_state.pos_setpoint[2] - flight_params.home_pos[2],
                                "[m]");
        }
    }
}

// 设置悬停位置
void UAVControl::set_hover_pos()
{
    // 将当前位置设置为悬停位置
    flight_params.hover_pos[0] = px4_state.position[0];
    flight_params.hover_pos[1] = px4_state.position[1];
    flight_params.hover_pos[2] = px4_state.position[2];
    flight_params.hover_yaw = px4_state.attitude[2];
}

// 发布惯性系下的目标点
void UAVControl::setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    px4_setpoint_local_pub.publish(setpoint);
}

// 发送经纬度以及高度期望值至飞控(输入,期望lat/lon/alt,期望yaw)
void UAVControl::setpoint_global_pub(uint16_t type_mask, mavros_msgs::GlobalPositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    px4_setpoint_global_pub.publish(setpoint);
}

// 设置默认目标点 用于清除过去指令的影响
void UAVControl::set_default_local_setpoint()
{
    local_setpoint.header.stamp = ros::Time::now();
    local_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_setpoint.type_mask = TypeMask::NONE_TYPE;
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

// 设置默认目标点 用于清除过去指令的影响
void UAVControl::set_default_global_setpoint()
{
    global_setpoint.header.stamp = ros::Time::now();
    global_setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT; // 相对高度
    global_setpoint.type_mask = TypeMask::GLOBAL_POSITION;
    global_setpoint.latitude = 0;
    global_setpoint.longitude = 0;
    global_setpoint.altitude = 0;
    global_setpoint.velocity.x = 0;
    global_setpoint.velocity.y = 0;
    global_setpoint.velocity.z = 0;
    global_setpoint.acceleration_or_force.x = 0;
    global_setpoint.acceleration_or_force.y = 0;
    global_setpoint.acceleration_or_force.z = 0;
    global_setpoint.yaw = 0;
    global_setpoint.yaw_rate = 0;
}

// 检查进入offboard模式
void UAVControl::set_offboard_control(int mode)
{
    // 如果不使用遥控器不允许进入RC_CONTROL模式
    if (mode == Control_Mode::RC_CONTROL && !system_params.use_rc)
    {
        Logger::error("RC not enabled, cannot enter RC_CONTROL mode!");
        return;
    }
    // 如果没有收到遥控器的回调，不允许进入RC_CONTROL模式
    if (mode == Control_Mode::RC_CONTROL && !rcState_cb)
    {
        Logger::error("RC callback error, cannot enter RC_CONTROL mode!");
        return;
    }
    // 如果无人机未解锁，不允许进入OFFBOARD模式
    if (!px4_state.armed && system_params.use_rc)
    {
        Logger::error("UAV not armed, cannot enter OFFBOARD mode!");
        return;
    }
    // 进入OFFBOARD模式前，先设置默认目标点
    set_hover_pos();
    control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd.header.stamp = ros::Time::now();
    if (px4_state.mode != "OFFBOARD")
    {
        set_offboard_mode();
    }
    system_params.control_mode = mode;
}

// 设置offboard模式
void UAVControl::set_offboard_mode()
{
    // 无人机未解锁时，不允许进入OFFBOARD模式
    if (!px4_state.armed && system_params.use_rc)
    {
        Logger::error("UAV not armed, cannot enter OFFBOARD mode!");
        return;
    }

    // 设置默认目标点+设置PX4为OFFBOARD模式（PX4进入OFFBOARD模式，需要发送指令才可进入，此处发送0指令）
    set_default_local_setpoint();
    local_setpoint.velocity.x = 0.0;
    local_setpoint.velocity.y = 0.0;
    local_setpoint.velocity.z = 0.0;
    setpoint_local_pub(TypeMask::XYZ_VEL, local_setpoint);
    set_px4_flight_mode("OFFBOARD");

    control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    set_hover_pos();
    control_cmd.header.stamp = ros::Time::now();
}

// 安全检查 + 发布状态
void UAVControl::check_state()
{
    // 安全检查
    system_params.safety_state = safetyCheck();
    if (system_params.safety_state == 1)
    {
        // 超出安全范围 进入降落模式
        if (px4_state.armed && system_params.control_mode != Control_Mode::LAND_CONTROL)
        {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("Out of safe range, landing...");
        }
    }
    else if (system_params.safety_state == 2) // 定位数据失效
    {
        // 定位失效要要进入降落模式
        if (system_params.control_mode == Control_Mode::RC_CONTROL || system_params.control_mode == Control_Mode::CMD_CONTROL)
        {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("Lost odom, landing...");
        }
    }
    // 发布状态
    uav_state.uav_id = uav_id;
    uav_state.header.stamp = ros::Time::now();
    uav_state.connected = px4_state.connected;
    uav_state.armed = px4_state.armed;
    uav_state.mode = px4_state.mode;
    uav_state.location_source = system_params.location_source;
    uav_state.odom_valid = system_params.odom_valid;
    for (int i = 0; i < 3; i++)
    {
        uav_state.position[i] = px4_state.position[i];
        uav_state.velocity[i] = px4_state.velocity[i];
        uav_state.attitude[i] = px4_state.attitude[i];
        uav_state.pos_setpoint[i] = px4_state.pos_setpoint[i];
        uav_state.vel_setpoint[i] = px4_state.vel_setpoint[i];
        uav_state.att_setpoint[i] = px4_state.att_setpoint[i];
    }
    uav_state.attitude_q = px4_state.attitude_q;
    uav_state.battery_state = px4_state.battery_state;
    uav_state.battery_percentage = px4_state.battery_percentage;
    uav_state.control_mode = system_params.control_mode;
    uav_state.move_mode = control_cmd.cmd;
    uav_state_pub.publish(uav_state);
}

// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系,yaw_angle是当前偏航角[rad]
void UAVControl::body2enu(double body_frame[2], double enu_frame[2], double yaw)
{
    enu_frame[0] = cos(yaw) * body_frame[0] - sin(yaw) * body_frame[1];
    enu_frame[1] = sin(yaw) * body_frame[0] + cos(yaw) * body_frame[1];
}

// 从指令中获取期望位置
void UAVControl::handle_cmd_control()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    // 如果需要检查指令超时：判断是否是新的指令，只有不是新的指令才会超时，悬停和起飞模式是不进行超时判断的
    if (system_params.check_cmd_timeout && !new_cmd && control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover && control_cmd.cmd != sunray_msgs::UAVControlCMD::Takeoff)
    {
        if ((ros::Time::now() - last_control_cmd.header.stamp).toSec() > system_params.cmd_timeout)
        {
            Logger::error("Command timeout, change to hover mode");
            // 超时会切换到悬停模式
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            control_cmd.header.stamp = ros::Time::now();
            set_desired_from_hover();
            return;
        }
    }
    // 在 use_rc 为 true 的情况下 如果无人机未解锁则不执行
    if (!allow_lock && !px4_state.armed && system_params.use_rc)
    {
        if (new_cmd)
        {
            Logger::error("UAV not armed, can't set desired frome cmd");
            last_control_cmd = control_cmd;
        }
        // 切换回INIT
        system_params.control_mode = Control_Mode::INIT;

        return;
    }

    // 特殊指令单独判断，执行对应的特殊指令处理函数
    if (advancedModeFuncMap.find(control_cmd.cmd) != advancedModeFuncMap.end())
    {
        // 调用对应的函数
        // std::cout<<"advancedMode"<<std::endl;
        // 目标点赋值
        advancedModeFuncMap[control_cmd.cmd]();
        // 发布PX4指令
        setpoint_local_pub(system_params.type_mask, local_setpoint);
    }
    else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::GlobalPos)
    {
        // 经纬度海拔控制模式
        // std::cout<<"globalPos"<<std::endl;
        set_default_global_setpoint();
        global_setpoint.latitude = control_cmd.latitude;
        global_setpoint.longitude = control_cmd.longitude;
        global_setpoint.altitude = control_cmd.altitude;
        global_setpoint.yaw = control_cmd.desired_yaw;
        system_params.type_mask = TypeMask::GLOBAL_POSITION;

        setpoint_global_pub(system_params.type_mask, global_setpoint);
    }
    // else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Att)
    // {
    //     // 姿态控制模式
    // }
    else
    {
        // std::cout<<"baseMode"<<std::endl;
        // 基础控制模式
        if (new_cmd)
        {
            // 判断指令是否存在
            auto it = moveModeMap.find(control_cmd.cmd);
            if (it != moveModeMap.end())
            {
                system_params.type_mask = it->second;
                // 机体系需要单独做转换
                if (control_cmd.cmd == sunray_msgs::UAVControlCMD::XyzPosYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyzVelYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyVelZPosYawBody)
                {
                    // Body系的需要转换到NED下】
                    // 清除过去数据 更新时间戳
                    set_default_local_setpoint();
                    double body_pos[2] = {control_cmd.desired_pos[0], control_cmd.desired_pos[1]};
                    double enu_pos[2] = {0.0, 0.0};
                    UAVControl::body2enu(body_pos, enu_pos, px4_state.attitude[2]); // 偏航角 px4_state.attitude[2]

                    local_setpoint.position.x = px4_state.position[0] + enu_pos[0];
                    local_setpoint.position.y = px4_state.position[1] + enu_pos[1];
                    local_setpoint.position.z = px4_state.position[2] + control_cmd.desired_pos[2];

                    // Body 系速度向量到 NED 系的转换
                    double body_vel[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                    double enu_vel[2] = {0.0, 0.0};
                    UAVControl::body2enu(body_vel, enu_vel, px4_state.attitude[2]);

                    local_setpoint.velocity.x = enu_vel[0];
                    local_setpoint.velocity.y = enu_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];

                    local_setpoint.yaw = control_cmd.desired_yaw + px4_state.attitude[2];

                    // 设置控制模式
                    system_params.type_mask = moveModeMap[control_cmd.cmd];
                }
                // 惯性系下的控制将直接赋值
                else
                {
                    // 清除过去数据 更新时间戳
                    set_default_local_setpoint();
                    local_setpoint.position.x = control_cmd.desired_pos[0];
                    local_setpoint.position.y = control_cmd.desired_pos[1];
                    local_setpoint.position.z = control_cmd.desired_pos[2];
                    if (system_params.use_offset)
                    {
                        local_setpoint.position.x = control_cmd.desired_pos[0] + flight_params.home_pos[0];
                        local_setpoint.position.y = control_cmd.desired_pos[1] + flight_params.home_pos[1];
                        local_setpoint.position.z = control_cmd.desired_pos[2] + flight_params.home_pos[2];
                    }
                    local_setpoint.velocity.x = control_cmd.desired_vel[0];
                    local_setpoint.velocity.y = control_cmd.desired_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];
                    local_setpoint.acceleration_or_force.x = control_cmd.desired_acc[0];
                    local_setpoint.acceleration_or_force.y = control_cmd.desired_acc[1];
                    local_setpoint.acceleration_or_force.z = control_cmd.desired_acc[2];
                    local_setpoint.yaw = control_cmd.desired_yaw;
                    local_setpoint.yaw_rate = control_cmd.desired_yaw_rate;
                    system_params.type_mask = moveModeMap[control_cmd.cmd];
                }
            }
            else
            {
                Logger::error("Unknown command!");
                if (new_cmd)
                {
                    set_desired_from_hover();
                }
            }
        }
        setpoint_local_pub(system_params.type_mask, local_setpoint);
    }

    last_control_cmd = control_cmd;
}

// 从遥控器状态中获取并计算期望值
void UAVControl::handle_rc_control()
{
    if ((ros::Time::now() - rc_state.header.stamp).toSec() > 1.5)
    {
        Logger::error("RC timeout!");
        return;
    }

    if (system_params.last_control_mode != system_params.control_mode)
    {
        system_params.last_rc_time = ros::Time::now();
    }

    double delta_t = (ros::Time::now() - system_params.last_rc_time).toSec();
    system_params.last_rc_time = ros::Time::now();

    // 遥控器的指令为机体系，因此需要将指令转换为惯性系
    double body_xy[2], enu_xy[2], body_z, body_yaw;
    // 允许一定误差(0.2 也就是1400 到 1600)保持悬停
    body_xy[0] = ((rc_state.channel[1] >= -0.2 && rc_state.channel[1] <= 0.2) ? 0 : rc_state.channel[1]) * rc_control_params.max_vel_xy * delta_t;
    body_xy[1] = -((rc_state.channel[0] >= -0.2 && rc_state.channel[0] <= 0.2) ? 0 : rc_state.channel[0]) * rc_control_params.max_vel_xy * delta_t;
    body_z = ((rc_state.channel[2] >= -0.2 && rc_state.channel[2] <= 0.2) ? 0 : rc_state.channel[2]) * rc_control_params.max_vel_z * delta_t;
    body_yaw = -rc_state.channel[3] * rc_control_params.max_vel_yaw * delta_t;
    body2enu(body_xy, enu_xy, px4_state.attitude[2]);

    // 定点悬停位置 = 前一个悬停位置 + 遥控器数值[-1,1] * 速度限幅 * delta_t
    // 因为这是一个积分系统，所以即使停杆了，无人机也还会继续移动一段距离
    flight_params.hover_pos[0] += enu_xy[0];
    flight_params.hover_pos[1] += enu_xy[1];
    flight_params.hover_pos[2] += body_z;
    flight_params.hover_yaw += body_yaw;

    // 如果低于起飞点，则悬停点的高度为起飞点高度 这对非回中的遥控控制很重要
    if (flight_params.hover_pos[2] < flight_params.home_pos[2] + 0.2)
        flight_params.hover_pos[2] = flight_params.home_pos[2] + 0.2;

    // 发布PX4控制指令
    local_setpoint.position.x = flight_params.hover_pos[0];
    local_setpoint.position.y = flight_params.hover_pos[1];
    local_setpoint.position.z = flight_params.hover_pos[2];
    local_setpoint.yaw = flight_params.hover_yaw;
    system_params.type_mask = TypeMask::XYZ_POS_YAW;
    setpoint_local_pub(system_params.type_mask, local_setpoint);
}

// 计算降落的期望值
void UAVControl::handle_land_control()
{
    // 如果无人机已经上锁，代表已经降落结束，切换控制状态机为INIT模式
    if (!px4_state.armed)
    {
        system_params.control_mode = Control_Mode::INIT;
        Logger::warning("Landing finished!");
    }

    // 使用AUTO.LAND飞行模式进行降落
    if (flight_params.land_type == 1)
    {
        if(px4_state.mode != "AUTO.LAND")
        {
            set_auto_land();
        }else
        {
            return;
        }
        
    }

    bool new_cmd = system_params.control_mode != system_params.last_control_mode ||
                   (control_cmd.cmd == sunray_msgs::UAVControlCMD::Land && (control_cmd.header.stamp != last_control_cmd.header.stamp));
    if (new_cmd)
    {
        system_params.last_land_time = ros::Time(0);
        set_default_local_setpoint();
        flight_params.land_pos[0] = px4_state.position[0];
        flight_params.land_pos[1] = px4_state.position[1];
        flight_params.land_pos[2] = flight_params.home_pos[2];
        flight_params.land_yaw = px4_state.attitude[2];
    }

    local_setpoint.position.x = flight_params.land_pos[0];
    local_setpoint.position.y = flight_params.land_pos[1];
    local_setpoint.position.z = flight_params.land_pos[2];
    local_setpoint.velocity.z = -flight_params.land_speed;
    local_setpoint.yaw = flight_params.land_yaw;
    system_params.type_mask = TypeMask::XYZ_POS_VEL_YAW;

    // 当无人机位置低于指定高度时，自动上锁
    if (px4_state.position[2] < flight_params.home_pos[2] + flight_params.disarm_height)
    {
        if (system_params.last_land_time == ros::Time(0))
        {
            system_params.last_land_time = ros::Time::now();
        }
        // 到达制定高度后向下移动land_end_time 防止其直接锁桨掉落
        set_default_local_setpoint();
        if ((ros::Time::now() - system_params.last_land_time).toSec() < flight_params.land_end_time)
        {
            local_setpoint.velocity.z = -flight_params.land_end_speed;
            local_setpoint.yaw = flight_params.land_yaw;
            system_params.type_mask = TypeMask::XYZ_VEL_YAW;
        }
        else
        {
            // 停桨降落完成
            emergencyStop();
            system_params.control_mode = Control_Mode::INIT;
        }
    }

    setpoint_local_pub(system_params.type_mask, local_setpoint);
}

// 获取悬停的期望值
void UAVControl::set_desired_from_hover()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if ((new_cmd && last_control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover) || control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover)
    {
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        control_cmd.header.stamp = ros::Time::now();
        set_default_local_setpoint();
        set_hover_pos();
    }
    local_setpoint.position.x = flight_params.hover_pos[0];
    local_setpoint.position.y = flight_params.hover_pos[1];
    local_setpoint.position.z = flight_params.hover_pos[2];
    local_setpoint.yaw = flight_params.hover_yaw;
    system_params.type_mask = TypeMask::XYZ_POS_YAW;
}

// 高级模式-返航模式的实现函数
void UAVControl::return_to_home()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        // 如果未设置home点，则无法进入返航模式
        if (!flight_params.set_home)
        {
            Logger::error("Home position not set! Cannot return to home!");
            set_desired_from_hover();
            return;
        }
        else
        {
            set_default_local_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = px4_state.position[2];
            local_setpoint.yaw = flight_params.home_yaw;
            system_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
    // 达到home点上方后，且速度降低后开始降落
    if ((px4_state.position[0] - flight_params.home_pos[0]) < 0.15 &&
        (px4_state.position[1] - flight_params.home_pos[1]) < 0.15 &&
        abs(px4_state.velocity[0]) < 0.1 &&
        abs(px4_state.velocity[1]) < 0.1 &&
        abs(px4_state.velocity[2]) < 0.1)

    {
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
        control_cmd.header.stamp = ros::Time::now();
    }
}

// 计算航点需要的yaw值
float UAVControl::get_yaw_from_waypoint(int type, float point_x, float point_y)
{
    // 朝向下一个点
    if (type == 2)
    {
        float yaw = atan2(point_y - px4_state.position[1],
                          point_x - px4_state.position[0]);
        // 如果航点距离较近，则不改变yaw值
        if ((abs(px4_state.position[0] - point_x) < 0.4) &&
            (abs(px4_state.position[1] - point_y) < 0.4) && wp_params.wp_state != 4)
        {
            yaw = px4_state.attitude[2];
        }
        return yaw;
    }
    else if (type == 3)
    {
        // 指向环绕点
        return atan2(wp_params.wp_points[10][1] - px4_state.position[1],
                     wp_params.wp_points[10][0] - px4_state.position[0]);
    }
    else
    {
        // 如果航点距离较近，则不改变yaw值
        if ((abs(px4_state.position[0] - point_x) < 0.4) &&
            (abs(px4_state.position[1] - point_y) < 0.4))
        {
            return px4_state.attitude[2];
        }
        return wp_params.wp_points[wp_params.wp_index][3];
    }
}

// 计算航点需要的速度
float UAVControl::get_vel_from_waypoint(float point_x, float point_y)
{
    wp_params.wp_x_vel = (point_x - px4_state.position[0]) * wp_params.wp_vel_p;
    wp_params.wp_y_vel = (point_y - px4_state.position[1]) * wp_params.wp_vel_p;
    // 如果合速度大于最大速度，则重新计算为最大速度
    if (sqrt(wp_params.wp_x_vel * wp_params.wp_x_vel + wp_params.wp_y_vel * wp_params.wp_y_vel) > wp_params.wp_move_vel * wp_params.wp_move_vel)
    {
        float ratio = sqrt(wp_params.wp_move_vel * wp_params.wp_move_vel / (wp_params.wp_x_vel * wp_params.wp_x_vel + wp_params.wp_y_vel * wp_params.wp_y_vel));
        wp_params.wp_x_vel *= ratio;
        wp_params.wp_y_vel *= ratio;
    }
}

// 高级模式-航点模式的实现函数
void UAVControl::waypoint_mission()
{
    // 检查是否已经上传航点
    if (!wp_params.wp_init)
    {
        Logger::error("Waypoint not uploaded! Cannot start waypoint mission!");
        set_desired_from_hover();
        return;
    }
    // 进入航点模式先重置航点索引
    if (last_control_cmd.cmd != control_cmd.cmd)
    {
        Logger::warning("Waypoint mission started!");
        wp_params.wp_index = 0;
        wp_params.wp_state = 0;
        wp_params.start_wp_time = ros::Time::now();
        wp_params.wp_point_takeoff[0] = px4_state.position[0];
        wp_params.wp_point_takeoff[1] = px4_state.position[1];
    }
    // 如果过程包含起飞过程，则先解锁起飞
    if (wp_params.wp_takeoff && (wp_params.wp_state == 0 || wp_params.wp_state == 1))
    {
        // 判断是否已经解锁 未解锁则先解锁
        if (!px4_state.armed && wp_params.wp_state == 0)
        {
            setArm(true);
            wp_params.wp_state = 1;
        }
        else
        {
            if ((ros::Time::now() - wp_params.start_wp_time).toSec() > 10)
            {
                // 解锁超时
                Logger::error("Takeoff timeout! Cannot start waypoint mission!");
                set_desired_from_hover();
            }
            if (px4_state.armed)
            {
                // 切入指令模式
                set_offboard_control(Control_Mode::CMD_CONTROL);
                // set_offboard_control会重置状态 所以需要手动重新赋值为Waypoint
                control_cmd.cmd = sunray_msgs::UAVControlCMD::Waypoint;
                // 更新为新状态
                wp_params.wp_state = 2;
            }
        }
    }
    else
    {
        if (!wp_params.wp_takeoff && !px4_state.armed)
        {
            Logger::error("UAV not armed! Cannot start waypoint mission!");
            set_desired_from_hover();
            return;
        }
        wp_params.start_wp_time = ros::Time::now();
        if (!wp_params.wp_takeoff && (wp_params.wp_state == 0 || wp_params.wp_state == 1))
        {
            wp_params.wp_state = 3;
            Logger::warning("next point:", wp_params.wp_points[wp_params.wp_index][0], wp_params.wp_points[wp_params.wp_index][1], wp_params.z_height);
        }
    }

    switch (wp_params.wp_state)
    {
    case 0:
        break;
    case 2:
        // 判断是否到达起飞点
        if ((abs(px4_state.position[0] - wp_params.wp_point_takeoff[0]) < 0.15) &&
            (abs(px4_state.position[1] - wp_params.wp_point_takeoff[1]) < 0.15) &&
            (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
        {
            wp_params.wp_state = 3;
            Logger::warning("Reached takeoff point!");
            Logger::warning("next point:", wp_params.wp_points[wp_params.wp_index][0], wp_params.wp_points[wp_params.wp_index][1], wp_params.z_height);
        }
        else
        {
            set_default_local_setpoint();
            local_setpoint.position.x = wp_params.wp_point_takeoff[0];
            local_setpoint.position.y = wp_params.wp_point_takeoff[1];
            local_setpoint.position.z = wp_params.z_height;
            system_params.type_mask = TypeMask::XYZ_POS;
        }
        break;
    case 3:
        // 判断是否达到航点
        if ((abs(px4_state.position[0] - wp_params.wp_points[wp_params.wp_index][0]) < 0.15) &&
            (abs(px4_state.position[1] - wp_params.wp_points[wp_params.wp_index][1]) < 0.15) &&
            (abs(px4_state.position[2] - wp_params.wp_points[wp_params.wp_index][2]) < 0.15))
        {
            wp_params.wp_index += 1;

            // 如果到达最后一个航点，判断是否需要返航, 不返航且需要降落则降落
            if (wp_params.wp_index >= wp_params.wp_num)
            {
                wp_params.wp_state = 4;
                break;
            }
            Logger::warning("next point: ", wp_params.wp_points[wp_params.wp_index][0], wp_params.wp_points[wp_params.wp_index][1],
                            wp_params.wp_points[wp_params.wp_index][2]);
        }
        else
        {
            // 如果未到达航点，则设置航点
            if (wp_params.wp_type == 0)
            {
                set_default_local_setpoint();
                get_vel_from_waypoint(wp_params.wp_points[wp_params.wp_index][0], wp_params.wp_points[wp_params.wp_index][1]);
                // local_setpoint.position.x = wp_params.wp_points[wp_params.wp_index][0];
                // local_setpoint.position.y = wp_params.wp_points[wp_params.wp_index][1];
                local_setpoint.position.z = wp_params.wp_points[wp_params.wp_index][2];
                local_setpoint.velocity.x = wp_params.wp_x_vel;
                local_setpoint.velocity.y = wp_params.wp_y_vel;

                local_setpoint.yaw = get_yaw_from_waypoint(wp_params.wp_yaw_type,
                                                           wp_params.wp_points[wp_params.wp_index][0],
                                                           wp_params.wp_points[wp_params.wp_index][1]);
                // system_params.type_mask = TypeMask::XYZ_POS_YAW;
                system_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
            }
        }
        break;
    case 4:
        // 如果航点结束需要返航
        if (wp_params.wp_end_type == 3)
        {
            // 到达返航点后降落
            if ((abs(px4_state.position[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.position[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
            {
                wp_params.wp_state = 5;
            }
            else
            {
                set_default_local_setpoint();
                get_vel_from_waypoint(flight_params.home_pos[0], flight_params.home_pos[1]);
                // local_setpoint.position.x = flight_params.home_pos[0];
                // local_setpoint.position.y = flight_params.home_pos[1];
                local_setpoint.velocity.x = wp_params.wp_x_vel;
                local_setpoint.velocity.y = wp_params.wp_y_vel;
                local_setpoint.position.z = wp_params.z_height;
                local_setpoint.yaw = get_yaw_from_waypoint(wp_params.wp_yaw_type,
                                                           flight_params.home_pos[0],
                                                           flight_params.home_pos[1]);
                system_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
            }
        }
        else
        {
            wp_params.wp_state = 5;
        }

        break;

    case 5:
        // 降落 如果是返航降落 还需要保证偏航角
        if (wp_params.wp_end_type == 3)
        {
            // 偏航角正负0.0872弧度，即5度
            if ((abs(px4_state.attitude[2] - flight_params.home_yaw) < 0.0872) &&
                (abs(px4_state.position[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.position[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
            {
                Logger::warning("Mission completed! waiting for landing");
                control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
                control_cmd.header.stamp = ros::Time::now();
                wp_params.wp_state = 6;
                break;
            }
            else
            {
                set_default_local_setpoint();
                local_setpoint.position.x = flight_params.home_pos[0];
                local_setpoint.position.y = flight_params.home_pos[1];
                local_setpoint.position.z = wp_params.z_height;
                local_setpoint.yaw = flight_params.home_yaw;
                system_params.type_mask = TypeMask::XYZ_POS_YAW;
            }
        }
        else if (wp_params.wp_end_type == 2)
        {
            // 原地降落
            Logger::warning("Mission completed! waiting for landing");
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd.header.stamp = ros::Time::now();
            wp_params.wp_state = 6;
        }
        else
        {
            // 航点任务结束
            Logger::warning("Mission completed! setting to hover");
            wp_params.wp_state = 6;
            set_hover_pos();
            set_desired_from_hover();
        }
        break;
    }
}

// 设置起飞的期望值
void UAVControl::set_takeoff()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        // 如果未设置home点，则无法起飞
        if (!flight_params.set_home)
        {
            Logger::error("Home position not set! Cannot takeoff!");
            set_desired_from_hover();
            return;
        }
        // 如果已经起飞，则不执行
        if ((px4_state.position[0] - flight_params.home_pos[0]) > 0.1 ||
            (px4_state.position[1] - flight_params.home_pos[1]) > 0.1 ||
            (px4_state.position[2] - flight_params.home_pos[2]) > 0.1)
        {
            Logger::warning("UAV already takeoff!");
        }
        else
        {
            set_default_local_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = flight_params.home_pos[2] + flight_params.takeoff_height;
            local_setpoint.yaw = flight_params.home_yaw;
            system_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
}

// 进入降落模式
void UAVControl::set_land()
{
    // 当前模式不是降落模式，且要处于RC_CONTROL或CMD_CONTROL模式才会进入降落模式
    if (system_params.control_mode != Control_Mode::LAND_CONTROL && (system_params.control_mode == Control_Mode::RC_CONTROL || system_params.control_mode == Control_Mode::CMD_CONTROL))
    {
        system_params.control_mode = Control_Mode::LAND_CONTROL;
        handle_land_control();
    }
}

// 发布goal话题
void UAVControl::publish_goal()
{
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = control_cmd.desired_pos[0];
    goal.pose.position.y = control_cmd.desired_pos[1];
    goal.pose.position.z = control_cmd.desired_pos[2];
    // tf欧拉角转四元素
    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, control_cmd.desired_yaw);
    goal.pose.orientation = q;
    // 发布goal话题
    goal_pub.publish(goal);
}