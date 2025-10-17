/*
本程序功能：
    1、管理无人机状态机 INIT RC_CONTROL CMD_CONTROL LAND_CONTROL WITHOUT_CONTROL等
    2、订阅外部控制话题 sunray/uav_control_cmd 用于接收和执行相应动作指令 XyzPos、XyzVel、Return等
    3、订阅外部话题 sunray/setup 用于控制程序状态机模式状态
    4、发布对应控制指令到mavros-px4
    5、发布无人机状态uav_state
*/

#include "UAVControl.h"

void UAVControl::init(ros::NodeHandle &nh)
{
    node_name = ros::this_node::getName();
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀
    // 无人机名字 = 无人机名字前缀 + 无人机ID
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 【参数】飞行相关参数
    nh.param<float>("flight_params/Takeoff_height", flight_params.takeoff_height, 1.0); // 【参数】默认起飞高度
    nh.param<int>("flight_params/land_type", flight_params.land_type, 0);               // 【参数】降落类型 【0:到达指定高度后锁桨 1:使用px4 auto.land】
    nh.param<float>("flight_params/Disarm_height", flight_params.disarm_height, 0.2);   // 【参数】降落时自动上锁高度
    nh.param<float>("flight_params/Land_speed", flight_params.land_speed, 0.2);         // 【参数】降落速度
    nh.param<float>("flight_params/land_end_time", flight_params.land_end_time, 1.0);   // 【参数】降落最后一阶段时间
    nh.param<float>("flight_params/land_end_speed", flight_params.land_end_speed, 0.3); // 【参数】降落最后一阶段速度
    nh.param<double>("flight_params/home_x", flight_params.home_pos[0], 0.0);            // 【参数】默认home点 在起飞后运行程序时需要
    nh.param<double>("flight_params/home_y", flight_params.home_pos[1], 0.0);            // 【参数】默认home点 在起飞后运行程序时需要
    nh.param<double>("flight_params/home_z", flight_params.home_pos[2], 0.0);            // 【参数】默认home点 在起飞后运行程序时需要
    flight_params.set_home = true;
    flight_params.home_yaw = 0.0;
    flight_params.hover_pos.setZero();
    flight_params.hover_yaw = 0.0;
    flight_params.land_pos.setZero();
    flight_params.land_yaw = 0.0;
    flight_params.relative_pos.setZero();

    // 【参数】无人机地理围栏 - 超出围栏后无人机自动降落
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0); // 【参数】地理围栏最小x坐标
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);  // 【参数】地理围栏最大x坐标
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0); // 【参数】地理围栏最小y坐标
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);  // 【参数】地理围栏最大y坐标
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);  // 【参数】地理围栏最小z坐标
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);   // 【参数】地理围栏最大z坐标

    // 【参数】无人机系统参数
    nh.param<bool>("system_params/use_rc_control", system_params.use_rc, true);                // 【参数】是否使用遥控器控制
    nh.param<bool>("system_params/check_flip", system_params.check_flip, true);                // 【参数】是否使用翻转上锁，默认启动
    nh.param<bool>("system_params/use_offset", system_params.use_offset, false);               // 【参数】是否使用位置偏移
    nh.param<bool>("system_params/check_cmd_timeout", system_params.check_cmd_timeout, false); // 【参数】是否检查无人机控制指令超时
    nh.param<float>("system_params/cmd_timeout", system_params.cmd_timeout, 2.0);              // 【参数】无人机控制指令超时阈值
    system_params.allow_lock = false;
    system_params.get_rc_signal = false;
    system_params.control_mode = Control_Mode::INIT;
    system_params.last_control_mode = Control_Mode::INIT;
    system_params.type_mask = 0;
    system_params.safety_state = -1;

    printf_params();

    // 【订阅】PX4无人机综合状态 - 飞控 -> mavros -> external_fusion_node -> 本节点
    px4_state_sub = nh.subscribe<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state", 10, &UAVControl::px4_state_callback, this);
    // 【订阅】无人机控制指令 - 外部节点 -> 本节点
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 10, &UAVControl::control_cmd_callback, this);
    // 【订阅】无人机设置指令 - 外部节点 -> 本节点
    setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 10, &UAVControl::uav_setup_callback, this);
    // 【订阅】遥控器数据 -- 飞控 -> mavros -> rc_input_node -> 本节点
    rc_state_sub = nh.subscribe<sunray_msgs::RcState>(uav_name + "/sunray/rc_state", 10, &UAVControl::rc_state_callback, this);

    // 【发布】无人机状态（接收到vision_pose节点的无人机状态，加上无人机控制模式，重新发布出去）- 本节点 -> 其他控制&任务节点/地面站
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 1);
    // 【发布】PX4位置环控制指令（包括期望位置、速度、加速度等接口，坐标系:ENU系） - 本节点 -> mavros -> 飞控
    px4_setpoint_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 1);
    // 【发布】PX4全局位置控制指令（包括期望经纬度等接口 坐标系:WGS84坐标系）- 本节点 -> mavros -> 飞控
    px4_setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(uav_name + "/mavros/setpoint_raw/global", 1);
    // 【发布】PX4姿态环控制指令（包括期望姿态等接口）- 本节点 -> mavros -> 飞控
    px4_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 1);
    // 【发布】 一个geometry_msgs::PoseStamped类型的消息，用于指定规划目标位置，与控制节点无关 - 本节点 -> 其他控制&任务节点
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_name + "/sunray/goal", 1);

    // 【服务】PX4解锁/上锁指令 -- 本节点 -> mavros -> 飞控
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
    // 【服务】PX4修改PX4飞行模式指令 -- 本节点 -> mavros -> 飞控
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    // 【服务】PX4紧急上锁服务(KILL) -- 本节点 -> mavros -> 飞控
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_name + "/mavros/cmd/command");
    // 【服务】重启PX4飞控 -- 本节点 -> mavros -> 飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_name + "/mavros/cmd/command");

    // 【控制器】PID控制器初始化
    pos_controller_pid.init(nh);

    // 绑定特殊指令对应的实现函数
    // 起飞：将期望目标位置local_setpoint设置为home_pos上方（起飞高度参数：takeoff_height）,未发布local_setpoint
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Takeoff] = std::bind(&UAVControl::set_takeoff, this);
    // 降落：将system_params.control_mode修改为LAND_CONTROL
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Land] = std::bind(&UAVControl::set_land, this);
    // 悬停：将期望目标位置local_setpoint设置为当前点,未发布local_setpoint
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Hover] = std::bind(&UAVControl::set_desired_from_hover, this);
    // 航点：TODO，未发布local_setpoint
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Waypoint] = std::bind(&UAVControl::waypoint_mission, this);
    // 返航：将期望目标位置local_setpoint设置为home点上方（高度为当前高度），返航后降落，未发布local_setpoint
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Return] = std::bind(&UAVControl::return_to_home, this);
}

// 检查当前模式 并进入对应的处理函数中
void UAVControl::mainLoop()
{
    // 安全检查 + 发布状态话题 ~/sunray/uav_state
    check_state();

    // 检测到无人机倾覆，直接上锁（实现：能够手动翻转上锁）
    if(system_params.check_flip)
    {
        check_flip();
    }

    // 无人机控制状态机：控制模式由遥控器话题（遥控器拨杆）进行切换
    switch (system_params.control_mode)
    {
    // 初始模式：保持PX4在定点模式，此时PX4不接收来自机载电脑的任何指令
    case Control_Mode::INIT:
        // 无人机在未解锁状态且处于非定点模式时切换到定点模式
        // 注意：无人机在POSCTL模式下，如果飞控没有接入遥控器是不允许解锁的
        if (!px4_state.armed && uav_state.mode != "POSCTL" && system_params.safety_state == 0)
        {
            set_px4_flight_mode("POSCTL");
            ros::Duration(1.0).sleep();
        }
        break;

    // 遥控器控制模式（RC_CONTROL）
    // 两种进入方式：1、通过"~/sunray/setup"话题；2、通过遥控器设置进入
    case Control_Mode::RC_CONTROL:
        // 在RC_CONTROL模式下，控制程序根据遥控器的摇杆来控制无人机移动
        // 类似于PX4的定点模式，只不过此时PX4为OFFBOARD模式（PX4的控制指令来自机载电脑）
        handle_rc_control();
        break;

    // CMD控制模式（CMD_CONTROL）：根据"/sunray/uav_control_cmd"话题的控制指令来控制无人机移动（二次开发一般使用这个模式）
    // 两种进入方式：1、通过"~/sunray/setup"话题；2、通过遥控器设置进入
    case Control_Mode::CMD_CONTROL:
        handle_cmd_control();
        break;

    // 原地降落控制模式（LAND_CONTROL）：当前位置原地降落，降落后会自动上锁
    // 多种进入方式：1、通过"~/sunray/setup"话题；2、通过遥控器设置进入；3、通过"~/sunray/uav_control_cmd"话题；4、控制程序自动判断进入
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
    if (!uav_state.odom_valid)
    {
        return 2;
    }
    return 0;
}

// 无人机状态回调
void UAVControl::px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg)
{
    // 每一次解锁时，将无人机的解锁位置设置为home点
    if (!px4_state.armed && msg->armed)
    {
        flight_params.home_pos[0] = px4_state.position[0];
        flight_params.home_pos[1] = px4_state.position[1];
        flight_params.home_pos[2] = px4_state.position[2];
        flight_params.home_yaw = px4_state.attitude[2];
        flight_params.set_home = true;
        Logger::info("Home position set to: ", flight_params.home_pos[0], flight_params.home_pos[1], flight_params.home_pos[2]);
    }

    // 无人机上锁后，重置set_home状态位
    if (flight_params.set_home && !msg->armed)
    {
        flight_params.set_home = false;
        Logger::warning("PX4 disarmed, flight_params.set_home [false]!");
    }

    // 更新px4_state
    px4_state = *msg;

    // 同步关键的px4_state至uav_state
    uav_state.connected = px4_state.connected;
    uav_state.armed = px4_state.armed;
    uav_state.mode = px4_state.mode;
    uav_state.landed_state = px4_state.landed_state;
    uav_state.battery_state = px4_state.battery_state;
    uav_state.battery_percentage = px4_state.battery_percentage;
    uav_state.location_source = px4_state.external_odom.external_source;
    uav_state.odom_valid = px4_state.external_odom.odom_valid;
    uav_state.vio_start = px4_state.external_odom.vio_start;
    uav_state.algo_status = px4_state.external_odom.algo_status;

    for (int i = 0; i < 3; i++)
    {
        uav_state.position[i] = px4_state.position[i];
        uav_state.velocity[i] = px4_state.velocity[i];
        uav_state.attitude[i] = px4_state.attitude[i];
        uav_state.attitude_rate[i] = px4_state.attitude_rate[i];
        uav_state.pos_setpoint[i] = px4_state.pos_setpoint[i];
        uav_state.vel_setpoint[i] = px4_state.vel_setpoint[i];
        uav_state.att_setpoint[i] = px4_state.att_setpoint[i];
    }
    uav_state.attitude_q = px4_state.attitude_q;

    // 如果使用相对坐标，则将当前位置转换为相对坐标 仅显示用
    if (system_params.use_offset && flight_params.set_home)
    {
        flight_params.relative_pos[0] = px4_state.position[0] - flight_params.home_pos[0];
        flight_params.relative_pos[1] = px4_state.position[1] - flight_params.home_pos[1];
        flight_params.relative_pos[2] = px4_state.position[2] - flight_params.home_pos[2];
    }
}

// 遥控器状态回调
void UAVControl::rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg)
{
    rc_state = *msg;
    system_params.get_rc_signal = true;

    // 解锁与上锁通道判断
    if (rc_state.arm_state == sunray_msgs::RcState::ARM)
    {
        setArm(true);
    }
    else if (rc_state.arm_state == sunray_msgs::RcState::DISARM)
    {
        setArm(false);
    }

    // 状态机模式通道判断 1 - INIT 2 - RC_CONTROL 3 - CMD_CONTROL
    if (rc_state.mode_state == sunray_msgs::RcState::INIT)
    {
        Logger::warning("Switch to INIT mode with rc");
        system_params.control_mode = Control_Mode::INIT;
    }
    else if (rc_state.mode_state == sunray_msgs::RcState::RC_CONTROL)
    {
        Logger::warning("Switch to RC_CONTROL mode with rc");
        set_offboard_control(Control_Mode::RC_CONTROL);
    }
    else if (rc_state.mode_state == sunray_msgs::RcState::CMD_CONTROL)
    {
        Logger::warning("Switch to CMD_CONTROL mode with rc");
        set_offboard_control(Control_Mode::CMD_CONTROL);
    }

    // 降落通道判断
    if (rc_state.land_state == 1)
    {
        Logger::warning("Switch to LAND_CONTROL mode with rc");
        set_land();
    }

    // 紧急停止通道判断
    if (rc_state.kill_state == 1)
    {
        Logger::warning("Emergency Stop with rc");
        emergencyStop();
    }
}

// 设置PX4飞行模式
void UAVControl::set_px4_flight_mode(std::string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);

    Logger::warning("set_px4_flight_mode:", mode);
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
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;    // Reboot autopilot
    reboot_srv.request.param2 = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;
    px4_reboot_client.call(reboot_srv);

    system_params.control_mode = Control_Mode::INIT; 
    Logger::error("Reboot PX4!");
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

// 控制指令回调
void UAVControl::control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg)
{
    control_cmd = *msg;
    control_cmd.header.stamp = ros::Time::now();

    // 特殊模式单独判断 （如：紧急停止 航点任务）
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Point)
    {
        // Point模式：直接发布路径规划目标点（与程序本身状态无关）
        publish_goal();
        // 发布完成后要切换回之前的状态
        control_cmd = last_control_cmd;
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
        Logger::warning("Get uav setup cmd: Arm UAV.");
        break;
    // 设置指令：无人机上锁
    case sunray_msgs::UAVSetup::DISARM:
        setArm(false);
        Logger::warning("Get uav setup cmd: Disrm UAV.");
        break;
    // 设置指令：设置PX4模式，需要配合px4_mode进行设置
    case sunray_msgs::UAVSetup::SET_PX4_MODE:
        set_px4_flight_mode(msg->px4_mode);
        Logger::warning("Get uav setup cmd: Switch PX4 mode [", msg->px4_mode,"]");
        break;
    // 设置指令：重启PX4飞控
    case sunray_msgs::UAVSetup::REBOOT_PX4:
        reboot_px4();
        Logger::warning("Get uav setup cmd: Reboot PX4.");
        break;
    // 设置指令： 无人机紧急上锁
    case sunray_msgs::UAVSetup::EMERGENCY_KILL:
        emergencyStop();
        system_params.control_mode = Control_Mode::INIT;
        Logger::warning("Get uav setup cmd: Kill UAV.");
        break;
    // 设置指令：设置无人机控制模式，需要配合control_mode进行设置
    case sunray_msgs::UAVSetup::SET_CONTROL_MODE:
        if (msg->control_mode == "INIT")
        {
            system_params.control_mode = Control_Mode::INIT;
            Logger::warning("Get uav setup cmd: Switch to INIT mode.");
        }
        else if (msg->control_mode == "RC_CONTROL")
        {
            if (system_params.safety_state == 0)
            {
                Logger::warning("Get uav setup cmd: Switch to RC_CONTROL mode.");
                set_offboard_control(Control_Mode::RC_CONTROL);
            }
            else if (system_params.safety_state != 0)
            {
                Logger::error("Get uav setup cmd: Safety state error, cannot switch to RC_CONTROL mode.");
            }
        }
        else if (msg->control_mode == "CMD_CONTROL")
        {
            if (system_params.safety_state == 0)
            {
                set_offboard_control(Control_Mode::CMD_CONTROL);
                Logger::warning("Get uav setup cmd: Switch to CMD_CONTROL mode.");
            }
            else if (system_params.safety_state != 0)
            {
                Logger::error("Get uav setup cmd: Safety state error, cannot switch to CMD_CONTROL mode.");
            }
        }
        else if (msg->control_mode == "LAND_CONTROL")
        {
            set_land();
            Logger::warning("Get uav setup cmd: Switch to LAND_CONTROL mode.");
        }
        else if (msg->control_mode == "WITHOUT_CONTROL")
        {
            system_params.control_mode = Control_Mode::WITHOUT_CONTROL;
            Logger::warning("Get uav setup cmd: Switch to WITHOUT_CONTROL mode.");
        }
        else
        {
            Logger::error("Get uav setup cmd: Unknown control_mode!");
        }
        break;
    default:
        break;
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

// 发送角度期望值至飞控（输入: 期望角度-四元数,期望推力）
void UAVControl::send_attitude_setpoint(Eigen::Vector4d &u_att)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    // Mappings: If any of these bits are set, the corresponding input should be ignored:
    // bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
    //  0b00000111;
    att_setpoint.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                             mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    Eigen::Vector3d att_des;
    att_des << u_att(0), u_att(1), u_att(2);
    Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);
    att_setpoint.orientation.x = q_des.x();
    att_setpoint.orientation.y = q_des.y();
    att_setpoint.orientation.z = q_des.z();
    att_setpoint.orientation.w = q_des.w();
    att_setpoint.thrust = u_att(3);
    px4_setpoint_attitude_pub.publish(att_setpoint);
}

// 发布PX4位置环控制指令（/mavros/setpoint_raw/local，包括期望位置、速度、加速度等接口，坐标系:ENU系） - 本节点 -> mavros -> 飞控
void UAVControl::pub_setpoint_raw_local(mavros_msgs::PositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    px4_setpoint_local_pub.publish(setpoint);
}

// 发布惯性系下的目标点
void UAVControl::setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.type_mask = type_mask;
    px4_setpoint_local_pub.publish(setpoint);
}

// 发送经纬度以及高度期望值至飞控(输入,期望lat/lon/alt,期望yaw)
void UAVControl::pub_setpoint_raw_global(mavros_msgs::GlobalPositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    px4_setpoint_global_pub.publish(setpoint);
}

// 设置默认目标点 用于清除过去指令的影响
void UAVControl::set_default_local_setpoint()
{
    local_setpoint.header.stamp = ros::Time::now();
    // 这里虽然赋值是FRAME_LOCAL_NED，但是Mavros会当成ENU进行处理
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
    if(mode != Control_Mode::RC_CONTROL && mode != Control_Mode::CMD_CONTROL)
    {
        Logger::error("Cannot enter OFFBOARD mode: wrong Control_Mode!");
        return;
    }

    // 如果无人机未解锁，不允许进入OFFBOARD模式
    if (!system_params.allow_lock && !px4_state.armed && system_params.use_rc)
    {
        Logger::error("Cannot enter OFFBOARD mode: UAV not armed!");
        return;
    }

    // 如果不使用遥控器不允许进入RC_CONTROL模式
    if (mode == Control_Mode::RC_CONTROL && !system_params.use_rc)
    {
        Logger::error("Cannot enter RC_CONTROL mode: RC not enabled!");
        return;
    }
    // 如果没有收到遥控器的回调，不允许进入RC_CONTROL模式
    if (mode == Control_Mode::RC_CONTROL && !system_params.get_rc_signal)
    {
        Logger::error("Cannot enter RC_CONTROL mode: RC signal timeout");
        return;
    }

    // 通过检查，
    system_params.control_mode = mode;

    // 设置无人机默认指令为Hover
    control_cmd.header.stamp = ros::Time::now();
    control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    // 设置当前点为默认悬停目标点
    set_hover_pos();

    // 设置默认目标点+设置PX4为OFFBOARD模式
    // 注意：PX4从其他飞行模式进入OFFBOARD模式，需要发送期望指令才可进入，此处发送0指令
    set_default_local_setpoint();
    setpoint_local_pub(TypeMask::XYZ_VEL, local_setpoint);

    // 发送Mavros指令，切换PX4至OFFBOARD模式
    if (px4_state.mode != "OFFBOARD")
    {
        // 设置PX4飞行模式
        set_px4_flight_mode("OFFBOARD");
    }
}

// 安全检查 + 发布状态
void UAVControl::check_state()
{
    // 安全检查：检查是否超出地理围栏，检查里程计状态
    system_params.safety_state = safetyCheck();
    if (system_params.safety_state == 1)
    {
        // 超出安全范围 进入降落模式
        if (px4_state.armed && system_params.control_mode == Control_Mode::CMD_CONTROL)
        {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("safetyCheck: Out of safe range, landing...");
        }
    }
    else if (system_params.safety_state == 2) // 定位数据失效
    {
        // 定位失效要要进入降落模式
        if (system_params.control_mode == Control_Mode::RC_CONTROL || system_params.control_mode == Control_Mode::CMD_CONTROL)
        {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("safetyCheck: Lost odom, landing...");
        }
    }

    // 检查RC是否持续连接
    if ((ros::Time::now() - rc_state.header.stamp).toSec() > 0.2)
    {
        system_params.get_rc_signal = false;
    }

    // 发布uav_state
    uav_state.uav_id = uav_id;
    uav_state.header.stamp = ros::Time::now();
    uav_state.control_mode = system_params.control_mode;
    uav_state.move_mode = control_cmd.cmd;
    uav_state.takeoff_height = flight_params.takeoff_height;
    uav_state.home_pos[0] = flight_params.home_pos[0];
    uav_state.home_pos[1] = flight_params.home_pos[1];
    uav_state.home_pos[2] = flight_params.home_pos[2];
    uav_state.home_yaw = flight_params.home_yaw;
    uav_state.hover_pos[0] = flight_params.hover_pos[0];
    uav_state.hover_pos[1] = flight_params.hover_pos[1];
    uav_state.hover_pos[2] = flight_params.hover_pos[2];
    uav_state.hover_yaw = flight_params.hover_yaw;
    uav_state.land_pos[0] = flight_params.land_pos[0];
    uav_state.land_pos[1] = flight_params.land_pos[1];
    uav_state.land_pos[2] = flight_params.land_pos[2];
    uav_state.land_yaw = flight_params.land_yaw;

    uav_state_pub.publish(uav_state);
}

void UAVControl::check_flip()
{
    // 无人机已上锁，跳过该检查
    if(!uav_state.armed)
    {
        return;
    }

    // 判断是否翻转
    if (uav_state.attitude[0] > FLIP_ANGLE || uav_state.attitude[0] < -FLIP_ANGLE || uav_state.attitude[1] > M_PI/3 || uav_state.attitude[1]  < -M_PI/3 )
    {
        // 停桨
        emergencyStop();
        system_params.control_mode = Control_Mode::INIT;
        Logger::error("check_flip: FLIP detected! Emergency kill");
    }
}


// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系,yaw_angle是当前偏航角[rad]
void UAVControl::body2enu(double body_frame[2], double enu_frame[2], double yaw)
{
    enu_frame[0] = cos(yaw) * body_frame[0] - sin(yaw) * body_frame[1];
    enu_frame[1] = sin(yaw) * body_frame[0] + cos(yaw) * body_frame[1];
}

void UAVControl::pos_controller()
{
    // 期望值
    Desired_State_t desired_state;

    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos)
    {
        for (int i = 0; i < 3; i++)
        {
            desired_state.pos[i] = control_cmd.desired_pos[i];
            desired_state.vel[i] = 0.0;
            desired_state.acc[i] = 0.0;
        }
        desired_state.yaw = control_cmd.desired_yaw;
        desired_state.q = geometry_utils::yaw_to_quaternion((double)control_cmd.desired_yaw);
    }
    else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj)
    {
        for (int i = 0; i < 3; i++)
        {
            desired_state.pos[i] = control_cmd.desired_pos[i];
            desired_state.vel[i] = control_cmd.desired_vel[i];
            desired_state.acc[i] = control_cmd.desired_acc[i];
        }
        desired_state.yaw = control_cmd.desired_yaw;
        desired_state.q = geometry_utils::yaw_to_quaternion((double)control_cmd.desired_yaw);
    }

    // 设定期望值
    pos_controller_pid.set_desired_state(desired_state);
    // 设定当前值
    pos_controller_pid.set_current_state(px4_state);
    // 控制器更新
    Eigen::Vector4d u_att = pos_controller_pid.ctrl_update(100.0);
    send_attitude_setpoint(u_att);
}

// CMD控制模式（CMD_CONTROL）处理函数
void UAVControl::handle_cmd_control()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    // 当需要检查指令超时，即 system_params.check_cmd_timeout = true
    // 判断：1、是否是新的指令，只有不是新的指令才会超时；2、悬停和起飞模式下不进行超时判断
    if (system_params.check_cmd_timeout && !new_cmd && control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover && control_cmd.cmd != sunray_msgs::UAVControlCMD::Takeoff)
    {
        if ((ros::Time::now() - last_control_cmd.header.stamp).toSec() > system_params.cmd_timeout)
        {
            Logger::error("CMD_CONTROL: Command timeout, change to hover mode");
            // 指令超时：切换到悬停模式
            control_cmd.header.stamp = ros::Time::now();
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            set_desired_from_hover();
            return;
        }
    }

    // 在 use_rc 为 true 的情况下 如果无人机未解锁则不执行
    if (!system_params.allow_lock && !px4_state.armed && system_params.use_rc)
    {
        if (new_cmd)
        {
            Logger::error("CMD_CONTROL: UAV not armed, can't set desired from cmd");
            last_control_cmd = control_cmd;
        }
        // 切换回INIT
        system_params.control_mode = Control_Mode::INIT;
        return;
    }

    // 特殊指令单独判断，执行对应的特殊指令处理函数
    if (advancedModeFuncMap.find(control_cmd.cmd) != advancedModeFuncMap.end())
    {
        if (new_cmd)
        {
            Logger::warning("CMD_CONTROL:", moveModeMapStr[control_cmd.cmd]);
        }
        // 调用对应的函数对local_setpoint进行赋值
        advancedModeFuncMap[control_cmd.cmd]();
        // 发布PX4指令
        setpoint_local_pub(system_params.type_mask, local_setpoint);
    }
    else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::GlobalPos)
    {
        // 经纬度海拔控制模式
        set_default_global_setpoint();
        global_setpoint.latitude = control_cmd.latitude;
        global_setpoint.longitude = control_cmd.longitude;
        global_setpoint.altitude = control_cmd.altitude;
        global_setpoint.yaw = control_cmd.desired_yaw;
        global_setpoint.type_mask = TypeMask::GLOBAL_POSITION;
        pub_setpoint_raw_global(global_setpoint);
    }
    else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos || control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj)
    {
        // 根据期望值计算期望姿态角，并发送至PX4无人机
        pos_controller();
    }
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
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyVelZPosYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyVelZPosYawrateBody)
                {
                    // Body系的需要转换到NED下
                    // 清除过去数据 更新时间戳
                    set_default_local_setpoint();
                    // 机体系位置转换
                    double body_pos[2] = {control_cmd.desired_pos[0], control_cmd.desired_pos[1]};
                    double enu_pos[2] = {0.0, 0.0};
                    UAVControl::body2enu(body_pos, enu_pos, px4_state.attitude[2]); // 偏航角 px4_state.attitude[2]
                    local_setpoint.position.x = px4_state.position[0] + enu_pos[0];
                    local_setpoint.position.y = px4_state.position[1] + enu_pos[1];
                    local_setpoint.position.z = px4_state.position[2] + control_cmd.desired_pos[2];

                    // 机体系速度转换
                    double body_vel[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                    double enu_vel[2] = {0.0, 0.0};
                    UAVControl::body2enu(body_vel, enu_vel, px4_state.attitude[2]);
                    local_setpoint.velocity.x = enu_vel[0];
                    local_setpoint.velocity.y = enu_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];

                    // 机体系偏航角转换
                    local_setpoint.yaw = control_cmd.desired_yaw + px4_state.attitude[2];
                    local_setpoint.yaw_rate = control_cmd.desired_yaw_rate;
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
                        // todo：感觉有bug
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
                    // 设置控制模式
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
    if ((ros::Time::now() - rc_state.header.stamp).toSec() > 1.0)
    {
        Logger::error("RC timeout!");
        return;
    }

    // 第一次进入RC_CONTROL模式时，先对system_params.last_rc_time赋值
    if (system_params.last_control_mode != system_params.control_mode)
    {
        system_params.last_rc_time = ros::Time::now();
    }
    
    // 计算两次控制间隔时间
    double delta_t = (ros::Time::now() - system_params.last_rc_time).toSec();
    system_params.last_rc_time = ros::Time::now();

    // 遥控器的指令为机体系，因此需要将指令转换为惯性系
    double body_xy[2], enu_xy[2], body_z, body_yaw;
    // 允许一定误差(0.2 也就是1400 到 1600)保持悬停
    // RC_CONTROL控制逻辑：
    // 1、遥控器的摇杆共有四路输入，取值范围是[-1，1]；分别为:滚转通道rc_state.channel[0]、俯仰通道rc_state.channel[1]、油门通道rc_state.channel[2]、偏航通道rc_state.channel[3]
    // 2、这四个通道对应都是机体系的控制指令，实施控制的时候将其转换为惯性系的指令
    // 3、遥控器控制存在一定死区(deadzone)，也就是当输入的绝对值小于死区时，认为没有控制输入
    // 4、以机体系X轴举例，rc_state.channel[1]大于死区时，无人机往前移动的距离 = rc_state.channel[1]*最大平移速度*时间间隔(大约为0.005，与主程序频率有关)
    body_xy[0] =  ((abs(rc_state.channel[1]) <= RC_DEADZONE) ? 0 : rc_state.channel[1]) * rc_control_params.max_vel_xy * delta_t;
    body_xy[1] = -((abs(rc_state.channel[0]) <= RC_DEADZONE) ? 0 : rc_state.channel[0]) * rc_control_params.max_vel_xy * delta_t;
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

    // 发布PX4控制指令 - XYZ_POS_YAW
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

    // 降落模式1：使用AUTO.LAND飞行模式进行降落
    // AUTO.LAND：PX4内部的飞行模式，无人机原地降落，降落的速度和逻辑由PX4内部的控制逻辑决定
    if (flight_params.land_type == 1)
    {
        Logger::warning("Land in AUTO.LAND mode!");
        if (px4_state.mode != "AUTO.LAND")
        {
            set_px4_flight_mode("AUTO.LAND");
        }
        else
        {
            return;
        }
    }

    // 降落模式2：
    // 第一次进入的时候设置降落位置：降落位置设置为当前点下方(地面高度选择为home_pos的高度），偏航角为当前偏航角
    // 首先使用XYZ_POS_VEL_YAW移动模式向下移动到指定高度（flight_params.disarm_height）
    // 然后使用XYZ_VEL_YAW移动模式向下移动一段时间（flight_params.land_end_time）后，直接上锁
    if (!flight_params.set_land_pos)
    {
        Logger::warning("Set Land Position Done!");
        system_params.last_land_time = ros::Time(0);
        set_default_local_setpoint();
        flight_params.land_pos[0] = px4_state.position[0];
        flight_params.land_pos[1] = px4_state.position[1];
        flight_params.land_pos[2] = flight_params.home_pos[2];
        flight_params.land_yaw = px4_state.attitude[2];
        flight_params.set_land_pos = true;

        local_setpoint.position.x = flight_params.land_pos[0];
        local_setpoint.position.y = flight_params.land_pos[1];
        local_setpoint.position.z = flight_params.land_pos[2];
        local_setpoint.velocity.z = -flight_params.land_speed;
        local_setpoint.yaw = flight_params.land_yaw;
        system_params.type_mask = TypeMask::XYZ_POS_VEL_YAW;
    }

    // 当无人机位置低于指定高度时，自动上锁
    if (px4_state.position[2] < flight_params.home_pos[2] + flight_params.disarm_height)
    {
        if (system_params.last_land_time == ros::Time(0))
        {
            system_params.last_land_time = ros::Time::now();
        }
        // 到达指定高度后向下移动land_end_time 防止其直接锁桨掉落
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
            flight_params.set_land_pos = false;
        }
    }

    setpoint_local_pub(system_params.type_mask, local_setpoint);
}

// 高级模式-返航模式的实现函数
void UAVControl::return_to_home()
{
    // 返航逻辑：
    // 第一次进入的时候设置返航位置：返航位置设置为home点上方，偏航角也为home点偏航角
    // 返航到home点上方后，执行降落指令

    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        // 如果未设置home点，则无法进入返航模式
        if (!flight_params.set_home)
        {
            Logger::error("return_to_home: Home position not set! Cannot return to home!");
            set_desired_from_hover();
            return;
        }

        Logger::warning("return_to_home: Set Return Position Done!");
        set_default_local_setpoint();
        local_setpoint.position.x = flight_params.home_pos[0];
        local_setpoint.position.y = flight_params.home_pos[1];
        local_setpoint.position.z = px4_state.position[2];
        local_setpoint.yaw = flight_params.home_yaw;
        system_params.type_mask = TypeMask::XYZ_POS_YAW;
    }

    // 达到home点上方后，且速度降低后开始降落
    if ((px4_state.position[0] - flight_params.home_pos[0]) < 0.15 &&
        (px4_state.position[1] - flight_params.home_pos[1]) < 0.15 &&
        abs(px4_state.velocity[0]) < 0.1 &&
        abs(px4_state.velocity[1]) < 0.1 &&
        abs(px4_state.velocity[2]) < 0.1)
    {
        set_land();
    }
}

// 高级模式-航点模式的实现函数
void UAVControl::waypoint_mission()
{
    // 弃用，新的实现方式请查看waypoint文件夹
}

// 特殊指令实现函数（sunray_msgs::UAVControlCMD::Takeoff）：设置起飞的期望值
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
        if (uav_state.landed_state != sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
        {
            Logger::error("UAV already takeoff!");
            return;
        }

        set_default_local_setpoint();
        local_setpoint.position.x = flight_params.home_pos[0];
        local_setpoint.position.y = flight_params.home_pos[1];
        local_setpoint.position.z = flight_params.home_pos[2] + flight_params.takeoff_height;
        local_setpoint.yaw = flight_params.home_yaw;
        system_params.type_mask = TypeMask::XYZ_POS_YAW;

        Logger::info("Takeoff height param:", flight_params.takeoff_height, "[ m ]");
        Logger::info("Takeoff_POS [X Y Z]:",
                            flight_params.home_pos[0],
                            flight_params.home_pos[1],
                            flight_params.home_pos[2] + flight_params.takeoff_height,
                            "[ m ]");
        Logger::info("Takeoff_YAW :", flight_params.home_yaw / M_PI * 180, "[deg]");
    }
}

// 特殊指令实现函数：进入降落模式
void UAVControl::set_land()
{
    // 当前模式不是降落模式，且要处于RC_CONTROL或CMD_CONTROL模式才会进入降落模式
    if (system_params.control_mode != Control_Mode::LAND_CONTROL && (system_params.control_mode == Control_Mode::RC_CONTROL || system_params.control_mode == Control_Mode::CMD_CONTROL))
    {
        system_params.control_mode = Control_Mode::LAND_CONTROL;
    }
}

// Hover移动模式实现函数：获取悬停的期望值
void UAVControl::set_desired_from_hover()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if ((new_cmd && last_control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover) || control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover)
    {
        control_cmd.header.stamp = ros::Time::now();
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        set_default_local_setpoint();
        // 将当前位置设置为flight_params.hover_pos
        set_hover_pos();
    }
    
    // 如果无人机在地面（未起飞），发送0速度指令而不是位置指令，避免传感器噪声导致自动离地
    if (uav_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
    {
        local_setpoint.velocity.x = 0.0;
        local_setpoint.velocity.y = 0.0;
        local_setpoint.velocity.z = 0.0;
        local_setpoint.yaw = flight_params.hover_yaw;
        system_params.type_mask = TypeMask::XYZ_VEL_YAW;
    }
    else
    {
        // 已起飞，使用位置控制
        local_setpoint.position.x = flight_params.hover_pos[0];
        local_setpoint.position.y = flight_params.hover_pos[1];
        local_setpoint.position.z = flight_params.hover_pos[2];
        local_setpoint.yaw = flight_params.hover_yaw;
        system_params.type_mask = TypeMask::XYZ_POS_YAW;
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

    Logger::info("Publish goal point to planner");
    Logger::info("GOAL_POS [X Y Z]:",
                        control_cmd.desired_pos[0],
                        control_cmd.desired_pos[1],
                        control_cmd.desired_pos[2],
                        "[ m ]");
    Logger::info("GOAL_YAW :", control_cmd.desired_yaw / M_PI * 180, "[deg]");
}

// 打印状态
void UAVControl::show_ctrl_state()
{
    Logger::print_color(int(LogColor::cyan), ">>>>>>>>>>>>>>>> 无人机控制节点 - [", uav_name, "] <<<<<<<<<<<<<<<<<");

    // 基本信息 - 连接状态、飞控模式、电池状态
    Logger::print_color(int(LogColor::cyan), "-------- 无人机状态 - [~/sunray/uav_state] ");

    Logger::print_color(int(LogColor::green), "飞控连接: [ 已连接 ]  电池状态:", px4_state.battery_state, "[V]", px4_state.battery_percentage, "[%]");

    if (px4_state.armed)
    {
        if (px4_state.landed_state == 1)
        {
            Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]", LOG_GREEN, "[", px4_state.mode, "]", LOG_GREEN, "[ 未起飞 ]");
        }
        else
        {
            Logger::print_color(int(LogColor::green), "飞控状态: [ 已解锁 ]", LOG_GREEN, "[", px4_state.mode, "]", LOG_GREEN, "[ 已起飞 ]");
        }
    }
    else
    {
        if (px4_state.landed_state == 1)
        {
            Logger::print_color(int(LogColor::red), "飞控状态: [ 未解锁 ]", LOG_GREEN, "[ ", px4_state.mode, " ]", LOG_GREEN, "[ 未起飞 ]");
        }
        else
        {
            Logger::print_color(int(LogColor::red), "飞控状态: [ 未解锁 ]", LOG_GREEN, "[ ", px4_state.mode, " ]", LOG_GREEN, "[ 已起飞 ]");
        }
    }

    // 无GPS模式的情况
    Logger::print_color(int(LogColor::green), "无人机位置[X Y Z]:",
                        px4_state.position[0],
                        px4_state.position[1],
                        px4_state.position[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "无人机速度[X Y Z]:",
                        px4_state.velocity[0],
                        px4_state.velocity[1],
                        px4_state.velocity[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "无人机姿态[X Y Z]:",
                        px4_state.attitude[0] / M_PI * 180,
                        px4_state.attitude[1] / M_PI * 180,
                        px4_state.attitude[2] / M_PI * 180,
                        "[deg]");
    Logger::print_color(int(LogColor::green), "位置期望值[X Y Z]:",
                        px4_state.pos_setpoint[0],
                        px4_state.pos_setpoint[1],
                        px4_state.pos_setpoint[2],
                        "[ m ]");
    Logger::print_color(int(LogColor::green), "速度期望值[X Y Z]:",
                        px4_state.vel_setpoint[0],
                        px4_state.vel_setpoint[1],
                        px4_state.vel_setpoint[2],
                        "[m/s]");
    Logger::print_color(int(LogColor::green), "姿态期望值[X Y Z]:",
                        px4_state.att_setpoint[0] / M_PI * 180,
                        px4_state.att_setpoint[1] / M_PI * 180,
                        px4_state.att_setpoint[2] / M_PI * 180,
                        "[deg]",
                        "推力期望值 :", px4_state.thrust_setpoint * 100, "[ % ]");

    // control_cmd
    Logger::print_color(int(LogColor::cyan), "-------- 无人机控制指令 - [~/sunray/uav_control_cmd] ");

    // control_cmd - CMD_CONTROL
    if (system_params.control_mode == Control_Mode::CMD_CONTROL)
    {
        Logger::print_color(int(LogColor::green), "控制模式: [", modeMap[system_params.control_mode], "]", "- 移动模式: [", moveModeMapStr[control_cmd.cmd], "]");

        // 高级指令
        if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Takeoff)
        {
            Logger::print_color(int(LogColor::green), "起飞高度:", flight_params.takeoff_height, "[ m ]");
            Logger::print_color(int(LogColor::green), "Takeoff_POS [X Y Z]:",
                                flight_params.home_pos[0],
                                flight_params.home_pos[1],
                                flight_params.home_pos[2] + flight_params.takeoff_height,
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "Takeoff_YAW :",
                                flight_params.home_yaw / M_PI * 180,
                                "[deg]");
        }
        else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Land)
        {
            if (flight_params.land_type == 1)
            {
                Logger::print_color(int(LogColor::green), "Land in AUTO.LAND mode");
            }
            else
            {
                Logger::print_color(int(LogColor::green), "Land disarm_height:", flight_params.disarm_height, "[ m ]");
                Logger::print_color(int(LogColor::green), "Land_POS [X Y Z]:",
                                    flight_params.land_pos[0],
                                    flight_params.land_pos[1],
                                    flight_params.land_pos[2],
                                    "[ m ]");
                Logger::print_color(int(LogColor::green), "Land_VEL:",
                                    -flight_params.land_speed,
                                    "[m/s]");
                Logger::print_color(int(LogColor::green), "Land_YAW :",
                                    flight_params.land_yaw / M_PI * 180,
                                    "[deg]");
            }
        }
        else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Hover)
        {
            Logger::print_color(int(LogColor::green), "Hover_POS [X Y Z]:",
                                flight_params.hover_pos[0],
                                flight_params.hover_pos[1],
                                flight_params.hover_pos[2],
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "Hover_YAW :",
                                flight_params.hover_yaw / M_PI * 180,
                                "[deg]");
        }
        else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Return)
        {
            Logger::print_color(int(LogColor::green), "HOME_POS [X Y Z]:",
                                flight_params.home_pos[0],
                                flight_params.home_pos[1],
                                flight_params.home_pos[2],
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "HOME_YAW :",
                                flight_params.home_yaw / M_PI * 180,
                                "[deg]");
        }
        else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Point)
        {
            Logger::print_color(int(LogColor::green), "Publish goal point to planner");
            Logger::print_color(int(LogColor::green), "GOAL_POS [X Y Z]:",
                                control_cmd.desired_pos[0],
                                control_cmd.desired_pos[1],
                                control_cmd.desired_pos[2],
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "GOAL_YAW :",
                                control_cmd.desired_yaw / M_PI * 180,
                                "[deg]");
        }
        else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos || control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj)
        {
            pos_controller_pid.printf_debug();
        }
        else
        {
            Logger::print_color(int(LogColor::green), "POS CMD [X Y Z]:",
                                control_cmd.desired_pos[0],
                                control_cmd.desired_pos[1],
                                control_cmd.desired_pos[2],
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "VEL CMD [X Y Z]:",
                                control_cmd.desired_vel[0],
                                control_cmd.desired_vel[1],
                                control_cmd.desired_vel[2],
                                "[m/s]");
            Logger::print_color(int(LogColor::green), "ACC CMD [X Y Z]:",
                                control_cmd.desired_acc[0],
                                control_cmd.desired_acc[1],
                                control_cmd.desired_acc[2],
                                "[m/s^2]");
            Logger::print_color(int(LogColor::green), "YAW CMD [yaw, yaw_rate]:",
                                control_cmd.desired_yaw / M_PI * 180,
                                "[deg]",
                                control_cmd.desired_yaw_rate / M_PI * 180,
                                "[deg/s]");
        }
    }
    else if (system_params.control_mode == Control_Mode::RC_CONTROL)
    {
        // control_cmd - RC_CONTROL
        Logger::print_color(int(LogColor::green), "控制模式: [", modeMap[system_params.control_mode], "]", "- Move with RC by PX4 original controller");
        Logger::print_color(int(LogColor::green), "Hover_POS [X Y Z]:",
                            flight_params.hover_pos[0],
                            flight_params.hover_pos[1],
                            flight_params.hover_pos[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::green), "Hover_YAW :",
                            flight_params.hover_yaw / M_PI * 180,
                            "[deg]");
    }else
    {
        Logger::print_color(int(LogColor::green), "控制模式: [", modeMap[system_params.control_mode], "]");
    }

    if (system_params.use_offset)
    {
        // 打印相对位置
        Logger::print_color(int(LogColor::blue), "POS(relative to home)");
        Logger::print_color(int(LogColor::green), "Relative POS[X Y Z]:",
                            flight_params.relative_pos[0],
                            flight_params.relative_pos[1],
                            flight_params.relative_pos[2],
                            "[ m ]");
        Logger::print_color(int(LogColor::blue), "PX4 TARGET (relative to home)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.pos_setpoint[0] - flight_params.home_pos[0],
                            px4_state.pos_setpoint[1] - flight_params.home_pos[1],
                            px4_state.pos_setpoint[2] - flight_params.home_pos[2],
                            "[ m ]");
    }

    Logger::print_color(int(LogColor::cyan), "---------------------------------------------------------");
}

void UAVControl::printf_params()
{
    Logger::print_color(int(LogColor::green), ">>>>>>>>>> uav_control_node - [", uav_name, "] params <<<<<<<<<<<");

    Logger::print_color(int(LogColor::green), "uav_id: [", uav_id, "]");
    Logger::print_color(int(LogColor::green), "flight_params.takeoff_height: [", flight_params.takeoff_height, "]");
    Logger::print_color(int(LogColor::green), "flight_params.land_type: [", flight_params.land_type, "]");
    Logger::print_color(int(LogColor::green), "flight_params.disarm_height: [", flight_params.disarm_height, "]");
    Logger::print_color(int(LogColor::green), "flight_params.land_speed: [", flight_params.land_speed, "]");
    Logger::print_color(int(LogColor::green), "flight_params.land_end_time: [", flight_params.land_end_time, "]");
    Logger::print_color(int(LogColor::green), "flight_params.home_pos[0]: [", flight_params.home_pos[0], "]");
    Logger::print_color(int(LogColor::green), "flight_params.home_pos[1]: [", flight_params.home_pos[1], "]");
    Logger::print_color(int(LogColor::green), "flight_params.home_pos[2]: [", flight_params.home_pos[2], "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.x_min: [", uav_geo_fence.x_min, "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.x_max: [", uav_geo_fence.x_max, "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.y_min: [", uav_geo_fence.y_min, "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.y_max: [", uav_geo_fence.y_max, "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.z_min: [", uav_geo_fence.z_min, "]");
    Logger::print_color(int(LogColor::green), "uav_geo_fence.z_max: [", uav_geo_fence.z_max, "]");
    Logger::print_color(int(LogColor::green), "system_params.check_cmd_timeout: [", system_params.check_cmd_timeout, "]");
    Logger::print_color(int(LogColor::green), "system_params.cmd_timeout: [", system_params.cmd_timeout, "]");
    Logger::print_color(int(LogColor::green), "system_params.use_rc: [", system_params.use_rc, "]");
    Logger::print_color(int(LogColor::green), "system_params.use_offset: [", system_params.use_offset, "]");
    Logger::print_color(int(LogColor::cyan), "---------------------------------------------------------");
}