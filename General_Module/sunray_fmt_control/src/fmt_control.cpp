/*
本程序功能：
    1、管理无人机状态机 INIT CMD_CONTROL LAND_CONTROL WITHOUT_CONTROL等
    2、订阅外部控制话题 sunray/uav_control_cmd 用于接收和执行相应动作指令 XyzPos、XyzVel、Return等
    3、订阅外部话题 sunray/setup 用于控制程序状态机模式状态
    4、发布对应控制指令到mavros-fmt
    5、发布无人机状态uav_state
*/

#include "FMTControl.h"

void FMTControl::init(ros::NodeHandle &nh) {
    // 获取参数
    nh.param<int>("uav_id", uav_id, 1);
    nh.param<std::string>("uav_name", uav_name, "uav");
    uav_ns = "/" + uav_name + std::to_string(uav_id);
    
    loadParams(nh); // 加载参数
    printf_params(); // 打印参数
    
    // 初始化订阅者
    fmt_state_sub = nh.subscribe<sunray_msgs::PX4State>(uav_ns + "/sunray/px4_state", 10, &FMTControl::fmt_state_callback, this);
    setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(uav_ns + "/sunray/setup", 10, &FMTControl::uav_setup_callback, this);
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(uav_ns + "/sunray/uav_control_cmd", 10, &FMTControl::control_cmd_callback, this);
    
    // 初始化发布者
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(uav_ns + "/sunray/uav_state", 10);
    fmt_setpoint_local_pub = nh.advertise<mavros_msgs::PositionTarget>(uav_ns + "/mavros/setpoint_raw/local", 10);
    fmt_setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(uav_ns + "/mavros/setpoint_raw/global", 10);
    fmt_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(uav_ns + "/mavros/setpoint_raw/attitude", 10);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(uav_ns + "/sunray/goal", 10);
    
    // 初始化服务客户端
    fmt_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_ns + "/mavros/cmd/arming");
    fmt_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_ns + "/mavros/set_mode");
    fmt_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_ns + "/mavros/cmd/command");
    fmt_stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>(uav_ns + "/mavros/set_stream_rate");
    fmt_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(uav_ns + "/mavros/cmd/command");
    
    // 初始化系统参数
    system_params.control_mode = Control_Mode::INIT;
    system_params.last_control_mode = Control_Mode::INIT;
    system_params.type_mask = 0;
    system_params.safety_state = -1;
    flight_params.home_pos[0] = default_home_x;
    flight_params.home_pos[1] = default_home_y;
    flight_params.home_pos[2] = default_home_z;
    allow_lock = false;
    offboard_channel_activated = false;
    
    // 初始化PID控制器
    pos_controller_pid.init(nh);

     // 绑定高级模式对应的实现函数
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Takeoff] = std::bind(&FMTControl::set_takeoff, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Land] = std::bind(&FMTControl::set_land, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Hover] = std::bind(&FMTControl::set_desired_from_hover, this);
    advancedModeFuncMap[sunray_msgs::UAVControlCMD::Return] = std::bind(&FMTControl::return_to_home, this);
    
    ROS_INFO("FMTControl initialized for %s", uav_ns.c_str());
}

void FMTControl::mainLoop() {
    // 安全检查 + 发布状态
    check_state();
    
    // 状态机处理
    switch (system_params.control_mode) {
        case Control_Mode::INIT:
            handle_init_mode();
            break;
        case Control_Mode::CMD_CONTROL:
            handle_cmd_control();
            break;
        case Control_Mode::LAND_CONTROL:
            handle_land_control();
            break;
        case Control_Mode::WITHOUT_CONTROL:
            // 无控制，不执行任何操作
            break;
        default:
            set_desired_from_hover();
            setpoint_local_pub(system_params.type_mask, local_setpoint);
            break;
    }
    system_params.last_control_mode = system_params.control_mode;
}

void FMTControl::loadParams(ros::NodeHandle &nh) {
    // 这里可以添加更多参数加载
        // 飞行参数
    nh.param<float>("flight_param/takeoff_height", flight_params.takeoff_height, 1.0);
    nh.param<float>("flight_param/takeoff_speed",flight_params.takeoff_speed,0.4);
    nh.param<int>("flight_param/land_type", flight_params.land_type, 0);
    nh.param<float>("flight_param/disarm_height", flight_params.disarm_height, 0.2);
    nh.param<float>("flight_param/land_speed", flight_params.land_speed, 0.2);
    nh.param<float>("flight_param/land_end_time", flight_params.land_end_time, 1.0);
    nh.param<float>("flight_param/land_end_speed", flight_params.land_end_speed, 0.3);
    
    // 地理围栏
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0);
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0);
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);
    
    // 系统参数
    nh.param<bool>("system/check_cmd_timeout", system_params.check_cmd_timeout, false);
    nh.param<float>("system/cmd_timeout", system_params.cmd_timeout, 2.0);
    nh.param<bool>("system/use_offset", system_params.use_offset, false);
}

// 安全检查
int FMTControl::safetyCheck() {
    // 检查是否超出地理围栏
    if (fmt_state.position[0] < uav_geo_fence.x_min ||
        fmt_state.position[0] > uav_geo_fence.x_max ||
        fmt_state.position[1] < uav_geo_fence.y_min ||
        fmt_state.position[1] > uav_geo_fence.y_max ||
        fmt_state.position[2] < uav_geo_fence.z_min ||
        fmt_state.position[2] > uav_geo_fence.z_max) {
        return 1; // 超出地理围栏
    }
    
    // 检查位置数据是否有效
    if (fmt_state.position[0] == 0 && fmt_state.position[1] == 0 && fmt_state.position[2] == 0) {
        return 2; // 位置数据无效
    }
    
    return 0; // 安全
}


// FMT状态回调
void FMTControl::fmt_state_callback(const sunray_msgs::PX4State::ConstPtr &msg) {
    // 每一次解锁时，将无人机的解锁位置设置为home点
    if (!fmt_state.armed && msg->armed) {
        flight_params.home_pos[0] = fmt_state.position[0];
        flight_params.home_pos[1] = fmt_state.position[1];
        flight_params.home_pos[2] = fmt_state.position[2];
        flight_params.home_yaw = fmt_state.attitude[2];
        flight_params.set_home = true;
        ROS_INFO("Home position set to: %.2f, %.2f, %.2f", 
                 flight_params.home_pos[0], flight_params.home_pos[1], flight_params.home_pos[2]);
    }
    
    // 无人机上锁后，重置set_home状态位
    if (flight_params.set_home && !msg->armed) {
        flight_params.set_home = false;
        ROS_INFO("FMT disarmed, flight_params.set_home [false]!");
    }
    
    // 更新fmt_state
    fmt_state = *msg;

    offboard_channel_activated = true; // 假设offboard模式总是激活
    
    // 同步关键的fmt_state至uav_state
    uav_state.connected = fmt_state.connected;
    uav_state.armed = fmt_state.armed;
    uav_state.mode = fmt_state.mode;
    uav_state.landed_state = fmt_state.landed_state;
    uav_state.battery_state = fmt_state.battery_state;
    uav_state.battery_percentage = fmt_state.battery_percentage;
    uav_state.location_source = fmt_state.external_odom.external_source;
    uav_state.odom_valid = fmt_state.external_odom.odom_valid;
    for (int i = 0; i < 3; i++) {
        uav_state.position[i] = fmt_state.position[i];
        uav_state.velocity[i] = fmt_state.velocity[i];
        uav_state.attitude[i] = fmt_state.attitude[i];
        uav_state.attitude_rate[i] = fmt_state.attitude_rate[i];
        uav_state.pos_setpoint[i] = fmt_state.pos_setpoint[i];
        uav_state.vel_setpoint[i] = fmt_state.vel_setpoint[i];
        uav_state.att_setpoint[i] = fmt_state.att_setpoint[i];
    }
    uav_state.attitude_q = fmt_state.attitude_q;
    
    // 如果使用相对坐标，则将当前位置转换为相对坐标 仅显示用
    if (system_params.use_offset && flight_params.set_home) {
        flight_params.relative_pos[0] = fmt_state.position[0] - flight_params.home_pos[0];
        flight_params.relative_pos[1] = fmt_state.position[1] - flight_params.home_pos[1];
        flight_params.relative_pos[2] = fmt_state.position[2] - flight_params.home_pos[2];
    }
}

// 设置FMT飞行模式
void FMTControl::set_fmt_flight_mode(std::string mode) {
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    fmt_set_mode_client.call(mode_cmd);

    //ROS_INFO("Already in %s",mode);
}

// 紧急停止
void FMTControl::emergencyStop() {
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
    
    if (fmt_emergency_client.call(emergency_srv)) {
        system_params.control_mode = Control_Mode::INIT;
        ROS_ERROR("Emergency Stop!");
    } else {
        ROS_ERROR("Failed to send emergency stop command");
    }
}

// 重启FMT
void FMTControl::reboot_fmt() {
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.confirmation = 0;
    reboot_srv.request.param1 = 1.0; // 1.0表示重启
    reboot_srv.request.param2 = 0.0;
    reboot_srv.request.param3 = 0.0;
    reboot_srv.request.param4 = 0.0;
    reboot_srv.request.param5 = 0.0;
    reboot_srv.request.param6 = 0.0;
    reboot_srv.request.param7 = 0.0;
    
    if (fmt_reboot_client.call(reboot_srv)) {
        ROS_WARN("Reboot command sent to FMT");
    } else {
        ROS_ERROR("Failed to send reboot command");
    }
}

// FMT解锁/上锁
void FMTControl::setArm(bool arm) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    
    if (fmt_arming_client.call(arm_cmd)) {
        if (arm) {
            ROS_INFO("Arming command sent");
        } else {
            ROS_INFO("Disarming command sent");
        }
    } else {
        ROS_ERROR("Failed to send arming command");
    }
}

// 设置自动降落模式
void FMTControl::set_auto_land() {
    set_fmt_flight_mode("AUTO.LAND");
}

// 控制指令回调
void FMTControl::control_cmd_callback(const sunray_msgs::UAVControlCMD::ConstPtr &msg) {
    control_cmd = *msg;
    control_cmd.header.stamp = ros::Time::now();
    
    // 特殊指令处理
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Point) {
        // Point模式：直接发布路径规划目标点
        publish_goal();
        // 发布完成后要切换回之前的状态
        control_cmd = last_control_cmd;
    } else {
        allow_lock = false;
    }
}

// 回调函数：接收无人机设置指令
void FMTControl::uav_setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg) {
    switch (msg->cmd) {
        case sunray_msgs::UAVSetup::ARM:
            setArm(true);
            break;
        case sunray_msgs::UAVSetup::DISARM:
            setArm(false);
            break;
        case sunray_msgs::UAVSetup::SET_PX4_MODE:
            set_fmt_flight_mode(msg->px4_mode);
            break;
        case sunray_msgs::UAVSetup::REBOOT_PX4:
            reboot_fmt();
            Logger::warning("Reboot FMT with UAVsetup cmd.");
            break;
        case sunray_msgs::UAVSetup::EMERGENCY_KILL:
            emergencyStop();
            system_params.control_mode = Control_Mode::INIT;
            break;
        case sunray_msgs::UAVSetup::SET_CONTROL_MODE:
            if (msg->control_mode == "INIT") {
                system_params.control_mode = Control_Mode::INIT;
                ROS_WARN("Switch to INIT mode with cmd");
            } else if (msg->control_mode == "CMD_CONTROL") {
                if (system_params.safety_state == 0) {
                    set_offboard_control(Control_Mode::CMD_CONTROL);
                    ROS_WARN("Switch to CMD_CONTROL mode with cmd");
                } else {
                    ROS_ERROR("Safety state error, cannot switch to CMD_CONTROL mode");
                } 
            } else if (msg->control_mode == "LAND_CONTROL") {
                set_land();
            } else if (msg->control_mode == "WITHOUT_CONTROL") {
                system_params.control_mode = Control_Mode::WITHOUT_CONTROL;
            } else {
                ROS_ERROR("Unknown control state!");
            }
            break;
        default:
            break;
    }
}

// 设置悬停位置
void FMTControl::set_hover_pos() {
    // 将当前位置设置为悬停位置
    flight_params.hover_pos[0] = fmt_state.position[0];
    flight_params.hover_pos[1] = fmt_state.position[1];
    flight_params.hover_pos[2] = fmt_state.position[2];
    flight_params.hover_yaw = fmt_state.attitude[2];
}

// 发送角度期望值至飞控
void FMTControl::send_attitude_setpoint(Eigen::Vector4d &u_att) {
    mavros_msgs::AttitudeTarget att_setpoint;
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
    fmt_setpoint_attitude_pub.publish(att_setpoint);
}

// 发布惯性系下的目标点
void FMTControl::setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint) {
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    fmt_setpoint_local_pub.publish(setpoint);
}

// 发送经纬度以及高度期望值至飞控
void FMTControl::setpoint_global_pub(uint16_t type_mask, mavros_msgs::GlobalPositionTarget setpoint) {
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    fmt_setpoint_global_pub.publish(setpoint);
}

// 设置默认目标点 local
void FMTControl::set_default_local_setpoint() {
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

// 设置默认目标点 global
void FMTControl::set_default_global_setpoint() {
    global_setpoint.header.stamp = ros::Time::now();
    global_setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
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

bool FMTControl::check_offboard_channel() {
    // 检查飞控当前模式是否为OFFBOARD
    if (fmt_state.mode == "OFFBOARD") {
        ROS_INFO("FMT is already in OFFBOARD mode");
        return true;
    }

    // 默认情况下不允许进入offboard模式
    ROS_WARN("Cannot determine offboard channel status, defaulting to deny");
    return false;
}

// 设置offboard控制模式
void FMTControl::set_offboard_control(int mode) {
    // 设置悬停位置
    set_desired_from_hover();
    
    // 如果已经在OFFBOARD模式，直接设置控制模式
    if (fmt_state.mode != "OFFBOARD") {
        set_offboard_mode();
        
    }
    system_params.control_mode = mode;

}

// 设置offboard模式
void FMTControl::set_offboard_mode() {
    // 发送零指令以确保可以进入OFFBOARD模式
    set_default_local_setpoint();
    local_setpoint.velocity.x = 0;
    local_setpoint.velocity.y = 0;
    local_setpoint.velocity.z = 0;
    
    setpoint_local_pub(TypeMask::XYZ_VEL, local_setpoint);

    // 切换到OFFBOARD模式
    set_fmt_flight_mode("OFFBOARD");

    ROS_INFO("In set_offboard_mode");
    // 设置悬停指令
    set_desired_from_hover();
}

// 安全检查 + 发布状态
void FMTControl::check_state() {
    // 安全检查
    system_params.safety_state = safetyCheck();
    if (system_params.safety_state == 1) {
        // 超出安全范围 进入降落模式
        if (fmt_state.armed && system_params.control_mode != Control_Mode::LAND_CONTROL) {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            ROS_ERROR("Out of safe range, landing...");
        }
    } else if (system_params.safety_state == 2) {
        // 定位失效要要进入降落模式
        if (system_params.control_mode == Control_Mode::CMD_CONTROL) {
            system_params.control_mode = Control_Mode::LAND_CONTROL;
            ROS_ERROR("Lost odom, landing...");
        }
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

// 坐标系旋转函数 - 机体系到enu系
// body_frame是机体系,enu_frame是惯性系,yaw_angle是当前偏航角[rad]
void FMTControl::body2enu(double body_frame[2], double enu_frame[2], double yaw) {
    enu_frame[0] = cos(yaw) * body_frame[0] - sin(yaw) * body_frame[1];
    enu_frame[1] = sin(yaw) * body_frame[0] + cos(yaw) * body_frame[1];
}

// 位置控制器
void FMTControl::pos_controller() {
    // 期望值
    Desired_State_t desired_state;
    
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos) {
        for (int i = 0; i < 3; i++) {
            desired_state.pos[i] = control_cmd.desired_pos[i];
            desired_state.vel[i] = 0.0;
            desired_state.acc[i] = 0.0;
        }
        desired_state.yaw = control_cmd.desired_yaw;
        desired_state.q = geometry_utils::yaw_to_quaternion((double)control_cmd.desired_yaw);
    } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj) {
        for (int i = 0; i < 3; i++) {
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
    pos_controller_pid.set_current_state(fmt_state);
    // 控制器更新
    Eigen::Vector4d u_att = pos_controller_pid.ctrl_update(100.0);
    send_attitude_setpoint(u_att);
}

// 从指令中获取期望位置
void FMTControl::handle_cmd_control() {
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    
    // 如果需要检查指令超时
    if (system_params.check_cmd_timeout && !new_cmd && 
        control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover && 
        control_cmd.cmd != sunray_msgs::UAVControlCMD::Takeoff) {
        if ((ros::Time::now() - last_control_cmd.header.stamp).toSec() > system_params.cmd_timeout) {
            ROS_WARN("Command timeout, change to hover mode");
            // 超时会切换到悬停模式
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            control_cmd.header.stamp = ros::Time::now();
            set_desired_from_hover();
            return;
        }
    }
       
    // 特殊指令单独判断，执行对应的特殊指令处理函数
    if (advancedModeFuncMap.find(control_cmd.cmd) != advancedModeFuncMap.end()) {
        // 调用对应的函数
        advancedModeFuncMap[control_cmd.cmd]();
        ROS_INFO_THROTTLE(1.0,"in advanced mode");
        // 发布FMT指令
        setpoint_local_pub(system_params.type_mask, local_setpoint);
    } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::GlobalPos) {
        // 经纬度海拔控制模式
        set_default_global_setpoint();
        global_setpoint.latitude = control_cmd.latitude;
        global_setpoint.longitude = control_cmd.longitude;
        global_setpoint.altitude = control_cmd.altitude;
        global_setpoint.yaw = control_cmd.desired_yaw;
        system_params.type_mask = TypeMask::GLOBAL_POSITION;
        
        setpoint_global_pub(system_params.type_mask, global_setpoint);
    } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos || 
               control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj) {
        // 根据期望值计算期望姿态角，并发送至FMT无人机
        pos_controller();
    } else {
        // 基础控制模式
        if (new_cmd) {
            // 判断指令是否存在
            auto it = moveModeMap.find(control_cmd.cmd);
            if (it != moveModeMap.end()) {
                system_params.type_mask = it->second;
                
                // 机体系需要单独做转换
                if (control_cmd.cmd == sunray_msgs::UAVControlCMD::XyzPosYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyzVelYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyVelZPosYawBody ||
                    control_cmd.cmd == sunray_msgs::UAVControlCMD::XyVelZPosYawrateBody) {
                    // Body系的需要转换到NED下
                    set_default_local_setpoint();
                    double body_pos[2] = {control_cmd.desired_pos[0], control_cmd.desired_pos[1]};
                    double enu_pos[2] = {0.0, 0.0};
                    body2enu(body_pos, enu_pos, fmt_state.attitude[2]);
                    
                    local_setpoint.position.x = fmt_state.position[0] + enu_pos[0];
                    local_setpoint.position.y = fmt_state.position[1] + enu_pos[1];
                    local_setpoint.position.z = fmt_state.position[2] + control_cmd.desired_pos[2];
                    
                    // Body 系速度向量到 NED 系的转换
                    double body_vel[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                    double enu_vel[2] = {0.0, 0.0};
                    body2enu(body_vel, enu_vel, fmt_state.attitude[2]);
                    
                    local_setpoint.velocity.x = enu_vel[0];
                    local_setpoint.velocity.y = enu_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];
                    
                    local_setpoint.yaw = control_cmd.desired_yaw + fmt_state.attitude[2];
                    local_setpoint.yaw_rate = control_cmd.desired_yaw_rate;
                    
                    // 设置控制模式
                    system_params.type_mask = moveModeMap[control_cmd.cmd];
                } else {
                    // 惯性系下的控制将直接赋值
                    set_default_local_setpoint();
                    local_setpoint.position.x = control_cmd.desired_pos[0];
                    local_setpoint.position.y = control_cmd.desired_pos[1];
                    local_setpoint.position.z = control_cmd.desired_pos[2];
                    if (system_params.use_offset) {
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
            } else {
                ROS_ERROR("Unknown command!");
                if (new_cmd) {
                    set_desired_from_hover();
                }
            }
        }
        setpoint_local_pub(system_params.type_mask, local_setpoint);
    }
    
    last_control_cmd = control_cmd;
}


// 计算降落的期望值
void FMTControl::handle_land_control() {
    // 如果无人机已经上锁，代表已经降落结束，切换控制状态机为INIT模式
    if (!fmt_state.armed) {
        system_params.control_mode = Control_Mode::INIT;
        ROS_INFO("Landing finished");
        return;
    }
    
    // 使用自动降落模式
    if (flight_params.land_type == 1) {
        if (fmt_state.mode != "AUTO.LAND") {
            set_auto_land();
        }
        return;
    }
    
    bool new_cmd = system_params.control_mode != system_params.last_control_mode ||
                   (control_cmd.cmd == sunray_msgs::UAVControlCMD::Land && 
                    (control_cmd.header.stamp != last_control_cmd.header.stamp));
    if (new_cmd) {
        system_params.last_land_time = ros::Time(0);
        set_default_local_setpoint();
        flight_params.land_pos[0] = fmt_state.position[0];
        flight_params.land_pos[1] = fmt_state.position[1];
        flight_params.land_pos[2] = flight_params.home_pos[2];
        flight_params.land_yaw = fmt_state.attitude[2];
    }
    
    local_setpoint.position.x = flight_params.land_pos[0];
    local_setpoint.position.y = flight_params.land_pos[1];
    //local_setpoint.position.z = flight_params.land_pos[2];
    local_setpoint.velocity.z = -flight_params.land_speed;
    local_setpoint.yaw = flight_params.land_yaw;
    system_params.type_mask = TypeMask::XY_POS_Z_VEL_YAW;
    
    // 当无人机位置低于指定高度时，自动上锁
    if (fmt_state.position[2] < flight_params.home_pos[2] + flight_params.disarm_height) {
        if (system_params.last_land_time == ros::Time(0)) {
            system_params.last_land_time = ros::Time::now();
        }
        // 到达制定高度后向下移动land_end_time 防止其直接锁桨掉落
        set_default_local_setpoint();
        if ((ros::Time::now() - system_params.last_land_time).toSec() < flight_params.land_end_time) {
            local_setpoint.velocity.z = -flight_params.land_end_speed;
            local_setpoint.yaw = flight_params.land_yaw;
            system_params.type_mask = TypeMask::XYZ_VEL_YAW;
        } else {
            // 停桨降落完成
            emergencyStop();
            system_params.control_mode = Control_Mode::INIT;
        }
    }
    
    setpoint_local_pub(system_params.type_mask, local_setpoint);
}

// 设置悬停期望值
void FMTControl::set_desired_from_hover()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if ((new_cmd && last_control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover) || 
        control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover)
    {
        control_cmd.header.stamp = ros::Time::now();
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        set_default_local_setpoint();
        //将当前位置设置为flight_params.hover_pos
        set_hover_pos();
    }

    local_setpoint.position.x = flight_params.hover_pos[0];
    local_setpoint.position.y = flight_params.hover_pos[1];
    local_setpoint.position.z = flight_params.hover_pos[2];
    local_setpoint.yaw = flight_params.hover_yaw;
    local_setpoint.type_mask = TypeMask::XYZ_POS_YAW;
    system_params.type_mask = TypeMask::XYZ_POS_YAW;
}

// 高级模式-返航模式的实现函数
void FMTControl::return_to_home() {
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        // 如果未设置home点，则无法进入返航模式
        if (!flight_params.set_home)
        {
            ROS_ERROR("Home position not set! Cannot return to home!");
            set_desired_from_hover();
            return;
        }
        else
        {
            set_default_local_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = fmt_state.position[2]; // 使用当前高度，不是起飞高度
            local_setpoint.yaw = flight_params.home_yaw;
            system_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
    // 达到home点上方后，且速度降低后开始降落
    if ((abs(fmt_state.position[0] - flight_params.home_pos[0]) < 0.15) &&
        (abs(fmt_state.position[1] - flight_params.home_pos[1]) < 0.15) &&
        abs(fmt_state.velocity[0]) < 0.1 &&
        abs(fmt_state.velocity[1]) < 0.1 &&
        abs(fmt_state.velocity[2]) < 0.1)
    {
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
        control_cmd.header.stamp = ros::Time::now();
    }
}

// 设置起飞的期望值
void FMTControl::set_takeoff() {
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd) {
        // 如果未设置home点，则无法起飞
        if (!flight_params.set_home) {
            ROS_ERROR("Home position not set! Cannot takeoff!");
            set_desired_from_hover();
            return;
        }
        // 如果已经起飞，则不执行
        if ((fmt_state.position[0] - flight_params.home_pos[0]) > 0.1 ||
            (fmt_state.position[1] - flight_params.home_pos[1]) > 0.1 ||
            (fmt_state.position[2] - flight_params.home_pos[2]) > 0.1) {
            ROS_WARN("UAV already takeoff!");
            is_takeoff_active = false;  
        } else {
            //初始化起飞参数，开始线性爬升
            is_takeoff_active = true;
            flight_params.takeoff_start_time = ros::Time::now();   //记录起飞开始时间

            ROS_INFO("Takeoff cmd received. Start smooth climbing.");
        }
    }

    // 执行线性爬升控制（仅在起飞激活状态下）
    if (is_takeoff_active) {
        // 计算最终目标高度
        double desired_alt = flight_params.home_pos[2] + flight_params.takeoff_height;
        // 计算已起飞时间
        double elapsed_time = (ros::Time::now() - flight_params.takeoff_start_time).toSec();
        // 启动平滑：前2秒使用渐入函数
        double startup_duration = 2.0;  // 秒
        double smooth_factor = 1.0;
        if (elapsed_time < startup_duration) {
            smooth_factor = (1.0 - cos(M_PI * elapsed_time / startup_duration)) / 2.0;
        }
        // 计算当前目标高度（线性增长，不超过最终目标）
        double current_target_alt = std::min(
            flight_params.home_pos[2] + flight_params.takeoff_speed * elapsed_time * smooth_factor,
            desired_alt
        );

        set_default_local_setpoint();
        local_setpoint.position.x = flight_params.home_pos[0];
        local_setpoint.position.y = flight_params.home_pos[1];
        local_setpoint.position.z = current_target_alt;
        //local_setpoint.position.z = flight_params.home_pos[2] + flight_params.takeoff_height;
        local_setpoint.yaw = flight_params.home_yaw;
        system_params.type_mask = TypeMask::XYZ_POS_YAW;
    }
}

// 进入降落模式
void FMTControl::set_land() {
    // 当前模式不是降落模式，且要处于CMD_CONTROL模式才会进入降落模式
    if (system_params.control_mode != Control_Mode::LAND_CONTROL && 
         system_params.control_mode == Control_Mode::CMD_CONTROL) {
        system_params.control_mode = Control_Mode::LAND_CONTROL;
        //handle_land_control();
    }
}

// 发布goal话题
void FMTControl::publish_goal() {
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

// 控制模式处理函数实现
void FMTControl::handle_init_mode() {
    // 无人机在未解锁状态且处于非定点模式时切换到定点模式
    if (!fmt_state.armed && fmt_state.mode != "POSCTL") {
        set_fmt_flight_mode("POSCTL");
        ros::Duration(1.0).sleep();
    }
}

// 工具函数实现
bool FMTControl::set_fmt_stream_rate(uint8_t stream_id, uint16_t message_rate, bool on_off) {
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = stream_id;
    srv.request.message_rate = message_rate;
    srv.request.on_off = on_off;
    
    if (fmt_stream_rate_client.call(srv)) {
        ROS_INFO("Set stream rate for ID %d to %d Hz", stream_id, message_rate);
        return true;
    } else {
        ROS_ERROR("Failed to set stream rate for ID %d", stream_id);
        return false;
    }
}

// 打印状态
void FMTControl::show_ctrl_state() {
    Logger::print_color(static_cast<int>(LogColor::white_bg_blue), ">>>>>>>>>>>>>>>> FMTControl - [", uav_ns, "] <<<<<<<<<<<<<<<<<");
    
    if (!fmt_state.connected) {
        Logger::print_color(static_cast<int>(LogColor::red), "FMT FCU:", "[ UNCONNECTED ]");
        Logger::print_color(static_cast<int>(LogColor::red), "Wait for FMT FCU connection...");
        return;
    }
    
    // 基本信息 - 连接状态、飞控模式、电池状态
    Logger::print_color(static_cast<int>(LogColor::white_bg_green), ">>>>> TOPIC: ~/sunray/uav_state");
    Logger::print_color(static_cast<int>(LogColor::green), "FMT FCU  : [ CONNECTED ]  BATTERY:", 
                        fmt_state.battery_state, "[V]", fmt_state.battery_percentage, "[%]");
    
    if (fmt_state.armed) {
        if (fmt_state.landed_state == 1) {
            Logger::print_color(static_cast<int>(LogColor::green), "FMT STATE: [ ARMED ]", LOG_GREEN, "[", fmt_state.mode, "]", LOG_GREEN, "[ ON_GROUND ]");
        } else {
            Logger::print_color(static_cast<int>(LogColor::green), "FMT STATE: [ ARMED ]", LOG_GREEN, "[", fmt_state.mode, "]", LOG_GREEN, "[ IN_AIR ]");
        }
    } else {
        if (fmt_state.landed_state == 1) {
            Logger::print_color(static_cast<int>(LogColor::red), "FMT STATE: [ DISARMED ]", LOG_GREEN, "[ ", fmt_state.mode, " ]", LOG_GREEN, "[ ON_GROUND ]");
        } else {
            Logger::print_color(static_cast<int>(LogColor::red), "FMT STATE: [ DISARMED ]", LOG_GREEN, "[ ", fmt_state.mode, " ]", LOG_GREEN, "[ IN_AIR ]");
        }
    }
    
    // 无GPS模式的情况
    Logger::print_color(static_cast<int>(LogColor::blue), "FMT Local Position & Attitude:");
    Logger::print_color(static_cast<int>(LogColor::green), "POS_UAV [X Y Z]:",
                        fmt_state.position[0],
                        fmt_state.position[1],
                        fmt_state.position[2],
                        "[ m ]");
    Logger::print_color(static_cast<int>(LogColor::green), "VEL_UAV [X Y Z]:",
                        fmt_state.velocity[0],
                        fmt_state.velocity[1],
                        fmt_state.velocity[2],
                        "[m/s]");
    Logger::print_color(static_cast<int>(LogColor::green), "ATT_UAV [X Y Z]:",
                        fmt_state.attitude[0] / M_PI * 180,
                        fmt_state.attitude[1] / M_PI * 180,
                        fmt_state.attitude[2] / M_PI * 180,
                        "[deg]");

    Logger::print_color(static_cast<int>(LogColor::green), "Control Mode: [", modeMap[system_params.control_mode], "]");
    
    // control_cmd - CMD_CONTROL
    if (system_params.control_mode == Control_Mode::CMD_CONTROL) {
        Logger::print_color(static_cast<int>(LogColor::green), "Move Mode: [", moveModeMapStr[control_cmd.cmd], "]");

        // 高级指令
        
        // 高级指令
        if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Takeoff) {
            Logger::print_color(static_cast<int>(LogColor::green), "Takeoff height param:", flight_params.takeoff_height, "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "Takeoff_POS [X Y Z]:",
                                flight_params.home_pos[0],
                                flight_params.home_pos[1],
                                flight_params.home_pos[2] + flight_params.takeoff_height,
                                "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "Takeoff_YAW :",
                                flight_params.home_yaw / M_PI * 180,
                                "[deg]");
        } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Land) {
            if (flight_params.land_type == 1)
            {
                Logger::print_color(static_cast<int>(LogColor::green), "Land in AUTO.LAND mode");
            }
            else
            {
                Logger::print_color(static_cast<int>(LogColor::green), "Land disarm_height:", flight_params.disarm_height, "[ m ]");
                Logger::print_color(static_cast<int>(LogColor::green), "Land_POS [X Y Z]:",
                                    flight_params.land_pos[0],
                                    flight_params.land_pos[1],
                                    flight_params.land_pos[2],
                                    "[ m ]");
                Logger::print_color(static_cast<int>(LogColor::green), "Land_VEL:",
                                    -flight_params.land_speed,
                                    "[m/s]");
                Logger::print_color(static_cast<int>(LogColor::green), "Land_YAW :",
                                    flight_params.land_yaw / M_PI * 180,
                                    "[deg]");
            }
        } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Hover) {
            Logger::print_color(static_cast<int>(LogColor::green), "Hover_POS [X Y Z]:",
                                flight_params.hover_pos[0],
                                flight_params.hover_pos[1],
                                flight_params.hover_pos[2],
                                "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "Hover_YAW :",
                                flight_params.hover_yaw / M_PI * 180,
                                "[deg]");
        } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Return) {
            Logger::print_color(static_cast<int>(LogColor::green), "HOME_POS [X Y Z]:",
                                flight_params.home_pos[0],
                                flight_params.home_pos[1],
                                flight_params.home_pos[2],
                                "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "HOME_YAW :",
                                flight_params.home_yaw / M_PI * 180,
                                "[deg]");
        } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::Point) {
            Logger::print_color(static_cast<int>(LogColor::green), "Publish goal point to planner");
            Logger::print_color(static_cast<int>(LogColor::green), "GOAL_POS [X Y Z]:",
                                control_cmd.desired_pos[0],
                                control_cmd.desired_pos[1],
                                control_cmd.desired_pos[2],
                                "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "GOAL_YAW :",
                                control_cmd.desired_yaw / M_PI * 180,
                                "[deg]");
        } else if (control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_XyzPos || 
                   control_cmd.cmd == sunray_msgs::UAVControlCMD::CTRL_Traj) {
            // 位置控制器调试信息
            pos_controller_pid.printf_debug();
        } else {
            Logger::print_color(static_cast<int>(LogColor::green), "POS CMD [X Y Z]:",
                                control_cmd.desired_pos[0],
                                control_cmd.desired_pos[1],
                                control_cmd.desired_pos[2],
                                "[ m ]");
            Logger::print_color(static_cast<int>(LogColor::green), "VEL CMD [X Y Z]:",
                                control_cmd.desired_vel[0],
                                control_cmd.desired_vel[1],
                                control_cmd.desired_vel[2],
                                "[m/s]");
            Logger::print_color(static_cast<int>(LogColor::green), "ACC CMD [X Y Z]:",
                                control_cmd.desired_acc[0],
                                control_cmd.desired_acc[1],
                                control_cmd.desired_acc[2],
                                "[m/s^2]");
            Logger::print_color(static_cast<int>(LogColor::green), "YAW CMD [yaw, yaw_rate]:",
                                control_cmd.desired_yaw / M_PI * 180,
                                "[deg]",
                                control_cmd.desired_yaw_rate / M_PI * 180,
                                "[deg/s]");
        }
    }
 
    if (system_params.use_offset) {
        // 打印相对位置
        Logger::print_color(static_cast<int>(LogColor::blue), "POS(relative to home)");
        Logger::print_color(static_cast<int>(LogColor::green), "Relative POS[X Y Z]:",
                            flight_params.relative_pos[0],
                            flight_params.relative_pos[1],
                            flight_params.relative_pos[2],
                            "[ m ]");
    }
}

// 打印参数
void FMTControl::printf_params() {
    Logger::print_color(static_cast<int>(LogColor::blue), LOG_BOLD, ">>>>>>>>>> FMTControl - [", uav_ns, "] params <<<<<<<<<<<");
    
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_id: [", uav_id, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "flight_params.takeoff_height: [", flight_params.takeoff_height, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "flight_params.land_type: [", flight_params.land_type, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "flight_params.disarm_height: [", flight_params.disarm_height, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "flight_params.land_speed: [", flight_params.land_speed, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "flight_params.land_end_time: [", flight_params.land_end_time, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "default_home_x: [", default_home_x, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "default_home_y: [", default_home_y, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "default_home_z: [", default_home_z, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.x_min: [", uav_geo_fence.x_min, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.x_max: [", uav_geo_fence.x_max, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.y_min: [", uav_geo_fence.y_min, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.y_max: [", uav_geo_fence.y_max, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.z_min: [", uav_geo_fence.z_min, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "uav_geo_fence.z_max: [", uav_geo_fence.z_max, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "system_params.check_cmd_timeout: [", system_params.check_cmd_timeout, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "system_params.cmd_timeout: [", system_params.cmd_timeout, "]");
    Logger::print_color(static_cast<int>(LogColor::blue), "system_params.use_offset: [", system_params.use_offset, "]");
}
