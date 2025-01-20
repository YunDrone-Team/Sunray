#include "UAVControl.h"

// 安全检查 是否超出地理围栏 外部定位是否有效
int UAVControl::safetyCheck()
{
    if (px4_state.pos[0] < uav_geo_fence.x_min ||
        px4_state.pos[0] > uav_geo_fence.x_max ||
        px4_state.pos[1] < uav_geo_fence.y_min ||
        px4_state.pos[1] > uav_geo_fence.y_max ||
        px4_state.pos[2] < uav_geo_fence.z_min ||
        px4_state.pos[2] > uav_geo_fence.z_max)
    {
        return 1;
    }

    float time_diff = (ros::Time::now() - odom_valid_time).toSec();
    if (odom_valid_time == ros::Time(0))
    {
        return 0;
    }
    else if (time_diff > odom_valid_timeout || !odom_valid)
    {
        return 2;
    }
    else if (time_diff < odom_valid_timeout && odom_valid && time_diff > odom_valid_warming_time)
    {
        return 3;
    }
    else
    {
        return 0;
    }
}

// 无人机odom回调
void UAVControl::px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    px4_state.pos[0] = msg->pose.pose.position.x;
    px4_state.pos[1] = msg->pose.pose.position.y;
    px4_state.pos[2] = msg->pose.pose.position.z;
    px4_state.vel[0] = msg->twist.twist.linear.x;
    px4_state.vel[1] = msg->twist.twist.linear.y;
    px4_state.vel[2] = msg->twist.twist.linear.z;
}

// 无人机pose回调
void UAVControl::px4_local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (location_source != 5 || location_source != 6)
    {
        px4_state.pos[0] = msg->pose.position.x;
        px4_state.pos[1] = msg->pose.position.y;
        px4_state.pos[2] = msg->pose.position.z;
    }
}

// 无人机pose回调
void UAVControl::px4_global_pos_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (location_source == 5 || location_source == 6)
    {
        px4_state.pos[0] = msg->pose.pose.position.x;
        px4_state.pos[1] = msg->pose.pose.position.y;
        px4_state.pos[2] = msg->pose.pose.position.z;
    }
}

// 无人机vel回调
void UAVControl::px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    px4_state.vel[0] = msg->twist.linear.x;
    px4_state.vel[1] = msg->twist.linear.y;
    px4_state.vel[2] = msg->twist.linear.z;
}

// 无人机状态回调
void UAVControl::px4_state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    if (!px4_state.armed && msg->armed)
    {
        flight_params.home_pos[0] = px4_state.pos[0];
        flight_params.home_pos[1] = px4_state.pos[1];
        flight_params.home_pos[2] = px4_state.pos[2];
        flight_params.home_set = true;
        Logger::info("Home position set to: ", flight_params.home_pos[0], flight_params.home_pos[1], flight_params.home_pos[2]);
    }

    px4_state.connected = msg->connected;
    px4_state.armed = msg->armed;
    px4_state.mode = msg->mode;

    if (flight_params.home_set && !px4_state.armed)
    {
        flight_params.home_set = false;
    }
}

// 无人机电量回调
void UAVControl::px4_battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    px4_state.batt_volt = msg->voltage;
    px4_state.batt_perc = msg->percentage * 100;
}

// 无人机姿态回调
void UAVControl::px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    px4_state.att_q[0] = msg->orientation.x;
    px4_state.att_q[1] = msg->orientation.y;
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

// 无人机目标位置回调
void UAVControl::px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_state.target_pos[0] = msg->position.x;
    px4_state.target_pos[1] = msg->position.y;
    px4_state.target_pos[2] = msg->position.z;
    px4_state.target_vel[0] = msg->velocity.x;
    px4_state.target_vel[1] = msg->velocity.y;
    px4_state.target_vel[2] = msg->velocity.z;
}

// 外部定位状态回调
void UAVControl::odom_state_callback(const sunray_msgs::ExternalOdom::ConstPtr &msg)
{
    odom_valid_time = ros::Time::now();
    odom_valid = msg->odom_valid;
    location_source = msg->external_source;
}

// 遥控器状态回调
void UAVControl::rc_state_callback(const sunray_msgs::RcState::ConstPtr &msg)
{
    rcState_cb = true;
    rc_state = *msg;
    if (rc_state.arm_state == 2)
    {
        setArm(true);
    }
    else if (rc_state.arm_state == 1)
    {
        setArm(false);
    }
    if (rc_state.mode_state == 1)
    {
        control_mode = Control_Mode::INIT;
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
    if (rc_state.land_state == 1)
    {
        Logger::warning("Switch to LAND_CONTROL mode with rc");
        set_land();
        // set_offboard_control(Control_Mode::LAND_CONTROL);
    }
    if (rc_state.kill_state == 1)
    {
        Logger::error("Emergency Stop with rc");
        emergencyStop();
    }
}

// 设置模式
void UAVControl::setMode(std::string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}

// 紧急停止
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

    control_mode = Control_Mode::INIT; // 紧急停止后，切换到初始化模式
    Logger::error("Emergency Stop!");
}

// 重启飞控
void UAVControl::reboot()
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

// 解锁
void UAVControl::setArm(bool arm)
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm;
    px4_arming_client.call(arm_cmd);
    if (arm)
    {

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
    // 航点模式直接自动切换到CMD_CONTROL模式
    if (control_cmd.cmd == Waypoint)
    {
        allow_lock = true;
        control_mode = Control_Mode::CMD_CONTROL;
    }
    else if (control_cmd.cmd == Point)
    {
        publish_goal();
        control_cmd = last_control_cmd;
    }
    else
    {
        allow_lock = false; // 其他模式需要手动解锁
    }
}

// 模式设置回调
void UAVControl::setup_callback(const sunray_msgs::UAVSetup::ConstPtr &msg)
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
        if (msg->control_state == "INIT" && control_mode != Control_Mode::INIT)
        {
            control_mode = Control_Mode::INIT;
            Logger::warning("Switch to INIT mode with cmd");
        }
        else if (msg->control_state == "CMD_CONTROL")
        {
            if (safety_state == 0 && (control_mode != Control_Mode::CMD_CONTROL || px4_state.mode != "OFFBOARD"))
            {
                Logger::warning("Switch to CMD_CONTROL mode with cmd");
                set_offboard_control(Control_Mode::CMD_CONTROL);
            }
            else
            {
                if (safety_state != 0)
                {
                    Logger::error("Safety state error, cannot switch to CMD_CONTROL mode");
                }
            }
        }
        else if (msg->control_state == "RC_CONTROL" && (control_mode != Control_Mode::RC_CONTROL || px4_state.mode != "OFFBOARD"))
        {
            if (safety_state == 0)
            {
                Logger::warning("Switch to RC_CONTROL mode with cmd");
                set_offboard_control(Control_Mode::RC_CONTROL);
            }
            else
            {
                if (safety_state != 0)
                {
                    Logger::error("Safety state error, cannot switch to RC_CONTROL mode!");
                }
            }
        }
        else if (msg->control_state == "LAND_CONTROL" && control_mode != Control_Mode::LAND_CONTROL)
        {
            set_land();
            // set_offboard_control(Control_Mode::LAND_CONTROL);
        }
        else if (msg->control_state == "WITHOUT_CONTROL" && control_mode != Control_Mode::WITHOUT_CONTROL)
        {
            control_mode = Control_Mode::WITHOUT_CONTROL;
        }
        else
        {
            Logger::error("Unknown control state!");
        }
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::EMERGENCY_KILL)
    {
        emergencyStop();
        control_mode = Control_Mode::INIT;
    }
}

void UAVControl::waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg)
{
    // 如果当前模式处于航点模式则不允许赋值
    if (control_cmd.cmd == Waypoint)
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
    // 环绕点最后一位
    wp_params.wp_points[10][0] = msg->wp_circle_point[0];
    wp_params.wp_points[10][1] = msg->wp_circle_point[1];

    wp_params.wp_init = true;
    Logger::warning("Waypoint setup success!");
}

// 打印状态
void UAVControl::print_state(const ros::TimerEvent &event)
{
    Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_prefix, "<<<<<<<<<<<<<<<");
    if (px4_state.connected)
    {
        Logger::print_color(int(LogColor::green), "CONNECTED:", "TRUE");
        if (px4_state.armed)
            Logger::print_color(int(LogColor::green), "MODE:", LOG_BLUE, px4_state.mode, "ARMED");
        else
            Logger::print_color(int(LogColor::green), "MODE:", LOG_BLUE, px4_state.mode, LOG_RED, "DISARMED");
        Logger::print_color(int(LogColor::green), "BATTERY:", px4_state.batt_volt, "[V]", px4_state.batt_perc, "[%]");
        Logger::print_color(int(LogColor::blue), "PX4 POS(receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.pos[0],
                            px4_state.pos[1],
                            px4_state.pos[2],
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            px4_state.vel[0],
                            px4_state.vel[1],
                            px4_state.vel[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            px4_state.att[0] / M_PI * 180,
                            px4_state.att[1] / M_PI * 180,
                            px4_state.att[2] / M_PI * 180,
                            "[deg]");
    }
    else
        Logger::print_color(int(LogColor::red), "CONNECTED:", "FALSE");
    Logger::color_no_del(int(LogColor::green), "Control Mode [", LOG_BLUE, modeMap[control_mode], LOG_GREEN, "]");
    if (control_mode == Control_Mode::CMD_CONTROL || control_mode == Control_Mode::RC_CONTROL)
    {
        Logger::color_no_del(int(LogColor::green), "Move Mode [", LOG_BLUE, moveModeMapStr[control_cmd.cmd], LOG_GREEN, "]");
        Logger::print_color(int(LogColor::blue), "PX4 TARGET (receive)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z]:",
                            px4_state.target_pos[0],
                            px4_state.target_pos[1],
                            px4_state.target_pos[2],
                            "[m]");
        Logger::print_color(int(LogColor::green), "VEL[X Y Z]:",
                            px4_state.target_vel[0],
                            px4_state.target_vel[1],
                            px4_state.target_vel[2],
                            "[m/s]");
        Logger::print_color(int(LogColor::green), "ATT[X Y Z]:",
                            px4_state.target_att[0],
                            px4_state.target_att[1],
                            px4_state.target_att[2],
                            "[m/s]");
    }
}

// 设置悬停位置
void UAVControl::set_hover_pos()
{
    flight_params.hover_pos = px4_state.pos;
    flight_params.hover_yaw = px4_state.att[2];
}

// 发布NED目标点
void UAVControl::setpoint_local_pub(uint16_t type_mask, mavros_msgs::PositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    px4_setpoint_local_pub.publish(setpoint);
}
// 发布Global目标点
void UAVControl::setpoint_global_pub(uint16_t type_mask, mavros_msgs::GlobalPositionTarget setpoint)
{
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    setpoint.type_mask = type_mask;
    px4_setpoint_global_pub.publish(setpoint);
}

// 设置默认目标点
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

// 设置默认目标点
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
    if (mode == Control_Mode::RC_CONTROL && !use_rc)
    {
        Logger::error("RC not enabled, cannot enter RC_CONTROL mode!");
        return;
    }
    if (mode == Control_Mode::RC_CONTROL && !rcState_cb)
    {
        Logger::error("RC callback error, cannot enter RC_CONTROL mode!");
        return;
    }
    if (!px4_state.armed && use_rc)
    {
        Logger::error("UAV not armed, cannot enter OFFBOARD mode!");
        return;
    }
    set_hover_pos();
    control_cmd.cmd = Hover;
    control_cmd.header.stamp = ros::Time::now();
    if (px4_state.mode != "OFFBOARD")
    {
        set_offboard_mode();
    }
    if (control_mode != mode)
    {
        control_mode = mode;
    }
}

// 设置offboard模式
void UAVControl::set_offboard_mode()
{
    // 使用遥控器时 未解锁则不允许进入OFFBOARD模式
    if (!px4_state.armed && use_rc)
    {
        Logger::error("UAV not armed, cannot enter OFFBOARD mode!");
        return;
    }
    set_default_local_setpoint();
    local_setpoint.velocity.x = 0.0;
    local_setpoint.velocity.y = 0.0;
    local_setpoint.velocity.z = 0.0;
    setpoint_local_pub(TypeMask::XYZ_VEL, local_setpoint);
    control_cmd.cmd = Hover;
    set_hover_pos();
    control_cmd.header.stamp = ros::Time::now();
    setMode("OFFBOARD");
}

// 任务定时器回调函数
void UAVControl::task_timer_callback(const ros::TimerEvent &event)
{
    // 安全检查
    safety_state = safetyCheck();
    if (safety_state == 1)
    {
        // 超出安全范围 进入降落模式
        if (px4_state.armed && control_mode != Control_Mode::LAND_CONTROL)
        {
            control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("Out of safe range, landing...");
        }
    }
    else if (safety_state == 2) // 定位数据失效
    {
        // 定位失效要要进入降落模式
        if (control_mode == Control_Mode::RC_CONTROL || control_mode == Control_Mode::CMD_CONTROL)
        {
            control_mode = Control_Mode::LAND_CONTROL;
            Logger::error("Lost odom, landing...");
        }
    }
    else if (safety_state == 3) // 定位数据没有失效但是延迟较高
    {
        // 预留暂不做任何事情
        Logger::warning("Odom delay too high!");
    }
    // 发布状态
    uav_state.uav_id = uav_id;
    uav_state.header.stamp = ros::Time::now();
    uav_state.connected = px4_state.connected;
    uav_state.armed = px4_state.armed;
    uav_state.mode = px4_state.mode;
    uav_state.location_source = location_source;
    uav_state.odom_valid = odom_valid;
    for (int i = 0; i < 3; i++)
    {
        uav_state.position[i] = px4_state.pos[i];
        uav_state.velocity[i] = px4_state.vel[i];
        uav_state.attitude[i] = px4_state.att[i];
        uav_state.pos_setpoint[i] = px4_state.target_pos[i];
        uav_state.vel_setpoint[i] = px4_state.target_vel[i];
        uav_state.att_setpoint[i] = px4_state.target_att[i];
    }
    uav_state.attitude_q.x = px4_state.att_q[0];
    uav_state.attitude_q.y = px4_state.att_q[1];
    uav_state.attitude_q.z = px4_state.att_q[2];
    uav_state.attitude_q.w = px4_state.att_q[3];
    uav_state.battery_state = px4_state.batt_volt;
    uav_state.battery_percetage = px4_state.batt_perc;
    uav_state.control_mode = control_mode;
    uav_state.move_mode = control_cmd.cmd;
    uav_state_pub.publish(uav_state);
}

// 坐标系转换
void UAVControl::body2ned(double body_xy[2], double ned_xy[2], double yaw)
{
    ned_xy[0] = cos(yaw) * body_xy[0] - sin(yaw) * body_xy[1];
    ned_xy[1] = sin(yaw) * body_xy[0] + cos(yaw) * body_xy[1];
}

// 从指令中获取期望位置
void UAVControl::set_desired_from_cmd()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (check_cmd_timeout && !new_cmd && control_cmd.cmd != Hover && control_cmd.cmd != Takeoff)
    {
        if ((ros::Time::now() - last_control_cmd.header.stamp).toSec() > cmd_timeout)
        {
            Logger::error("Command timeout, change to hover mode");
            control_cmd.cmd = Hover;
            control_cmd.header.stamp = ros::Time::now();
            set_desired_from_hover();
            return;
        }
    }
    // 如果无人机未解锁则不执行
    if (!allow_lock && !px4_state.armed && use_rc)
    {
        if (new_cmd)
        {
            Logger::error("UAV not armed, can't set desired frome cmd");
            last_control_cmd = control_cmd;
        }

        return;
    }
    // 高级模式单独判断
    if (advancedModeFuncMap.find(control_cmd.cmd) != advancedModeFuncMap.end())
    {
        // 调用对应的函数
        // std::cout<<"advancedMode"<<std::endl;
        advancedModeFuncMap[control_cmd.cmd]();
        setpoint_local_pub(flight_params.type_mask, local_setpoint);
    }
    else if (control_cmd.cmd == GlobalPos)
    {

        // 经纬度海拔控制模式
        // std::cout<<"globalPos"<<std::endl;
        set_default_global_setpoint();
        global_setpoint.latitude = control_cmd.latitude;
        global_setpoint.longitude = control_cmd.longitude;
        global_setpoint.altitude = control_cmd.altitude;
        global_setpoint.yaw = control_cmd.desired_yaw;
        flight_params.type_mask = TypeMask::GLOBAL_POSITION;

        setpoint_global_pub(flight_params.type_mask, global_setpoint);
    }
    else if (control_cmd.cmd == Att)
    {
        // 姿态控制模式
    }
    else
    {
        // std::cout<<"baseMode"<<std::endl;
        // 基础控制模式
        if (new_cmd)
        {
            auto it = moveModeMap.find(control_cmd.cmd);
            if (it != moveModeMap.end())
            {
                flight_params.type_mask = it->second;

                if (control_cmd.cmd == XyzPosYawBody ||
                    control_cmd.cmd == XyzVelYawBody ||
                    control_cmd.cmd == XyVelZPosYawBody)
                {
                    // Body系的需要转换到NED下
                    set_default_local_setpoint();
                    double body_pos[2] = {control_cmd.desired_pos[0], control_cmd.desired_pos[1]};
                    double ned_pos[2] = {0.0, 0.0};
                    UAVControl::body2ned(body_pos, ned_pos, px4_state.att[2]); // 偏航角 px4_state.att[2]

                    local_setpoint.position.x = px4_state.pos[0] + ned_pos[0];
                    local_setpoint.position.y = px4_state.pos[1] + ned_pos[1];
                    local_setpoint.position.z = px4_state.pos[2] + control_cmd.desired_pos[2];

                    // Body 系速度向量到 NED 系的转换
                    double body_vel[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                    double ned_vel[2] = {0.0, 0.0};
                    UAVControl::body2ned(body_vel, ned_vel, px4_state.att[2]);

                    local_setpoint.velocity.x = ned_vel[0];
                    local_setpoint.velocity.y = ned_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];

                    local_setpoint.yaw = control_cmd.desired_yaw + px4_state.att[2];

                    // 设置控制模式
                    flight_params.type_mask = moveModeMap[control_cmd.cmd];
                }
                else
                {
                    local_setpoint.position.x = control_cmd.desired_pos[0];
                    local_setpoint.position.y = control_cmd.desired_pos[1];
                    local_setpoint.position.z = control_cmd.desired_pos[2];
                    local_setpoint.velocity.x = control_cmd.desired_vel[0];
                    local_setpoint.velocity.y = control_cmd.desired_vel[1];
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];
                    local_setpoint.acceleration_or_force.x = control_cmd.desired_acc[0];
                    local_setpoint.acceleration_or_force.y = control_cmd.desired_acc[1];
                    local_setpoint.acceleration_or_force.z = control_cmd.desired_acc[2];
                    local_setpoint.yaw = control_cmd.desired_yaw;
                    local_setpoint.yaw_rate = control_cmd.desired_yaw_rate;
                    flight_params.type_mask = moveModeMap[control_cmd.cmd];
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
        setpoint_local_pub(flight_params.type_mask, local_setpoint);
    }

    last_control_cmd = control_cmd;
}

// 从遥控器状态中获取并计算期望值
void UAVControl::set_desired_from_rc()
{
    if ((ros::Time::now() - rc_state.header.stamp).toSec() > 1.5)
    {
        Logger::error("RC timeout!");
        return;
    }
    if (last_control_mode != control_mode)
    {
        flight_params.last_rc_time = ros::Time::now();
    }
    double delta_t = (ros::Time::now() - flight_params.last_rc_time).toSec();
    flight_params.last_rc_time = ros::Time::now();
    double body_xy[2], enu_xy[2], body_z, body_yaw;
    // 允许一定误差(0.2 也就是1400 到 1600)保持悬停
    body_xy[0] = ((rc_state.channel[1] >= -0.2 && rc_state.channel[1] <= 0.2) ? 0 : rc_state.channel[1]) * flight_params.max_vel_xy * delta_t;
    body_xy[1] = -((rc_state.channel[0] >= -0.2 && rc_state.channel[0] <= 0.2) ? 0 : rc_state.channel[0]) * flight_params.max_vel_xy * delta_t;
    body_z = ((rc_state.channel[2] >= -0.2 && rc_state.channel[2] <= 0.2) ? 0 : rc_state.channel[2]) * flight_params.max_vel_z * delta_t;
    body_yaw = -rc_state.channel[3] * flight_params.max_vel_yaw * delta_t;
    body2ned(body_xy, enu_xy, px4_state.att[2]);

    // 悬停位置 = 前一个悬停位置 + 遥控器数值[-1,1] * 0.01(如果主程序中设定是100Hz的话)
    flight_params.hover_pos[0] += enu_xy[0];
    flight_params.hover_pos[1] += enu_xy[1];
    flight_params.hover_pos[2] += body_z;
    flight_params.hover_yaw += body_yaw;

    // 如果低于起飞点，则悬停点的高度为起飞点高度 这对非回中的遥控控制很重要
    if (flight_params.hover_pos[2] < flight_params.home_pos[2] + 0.2)
        flight_params.hover_pos[2] = flight_params.home_pos[2] + 0.2;

    local_setpoint.position.x = flight_params.hover_pos[0];
    local_setpoint.position.y = flight_params.hover_pos[1];
    local_setpoint.position.z = flight_params.hover_pos[2];
    local_setpoint.yaw = flight_params.hover_yaw;
    // 因为这是一个积分系统，所以即使停杆了，无人机也还会继续移动一段距离
    flight_params.type_mask = TypeMask::XYZ_POS_YAW;
    setpoint_local_pub(flight_params.type_mask, local_setpoint);
}

// 计算降落的期望值
void UAVControl::set_desired_from_land()
{
    bool new_cmd = control_mode != last_control_mode ||
                   (control_cmd.cmd == Land && (control_cmd.header.stamp != last_control_cmd.header.stamp));
    if (new_cmd)
    {
        flight_params.last_land_time = ros::Time(0);
        set_default_local_setpoint();
        flight_params.land_pos[0] = px4_state.pos[0];
        flight_params.land_pos[1] = px4_state.pos[1];
        flight_params.land_pos[2] = flight_params.home_pos[2];
        flight_params.land_yaw = px4_state.att[2];
    }

    local_setpoint.position.x = flight_params.land_pos[0];
    local_setpoint.position.y = flight_params.land_pos[1];
    local_setpoint.position.z = flight_params.land_pos[2];
    local_setpoint.velocity.z = -land_speed;
    local_setpoint.yaw = flight_params.land_yaw;
    flight_params.type_mask = TypeMask::XYZ_POS_VEL_YAW;

    // 当无人机位置低于指定高度时，自动上锁
    if (px4_state.pos[2] < disarm_height)
    {
        if (flight_params.last_land_time == ros::Time(0))
        {
            flight_params.last_land_time = ros::Time::now();
        }
        // 到达制定高度后向下移动land_end_time 防止其直接锁桨掉落
        set_default_local_setpoint();
        if ((ros::Time::now() - flight_params.last_land_time).toSec() < land_end_time)
        {
            local_setpoint.velocity.z = -land_end_speed;
            local_setpoint.yaw = flight_params.land_yaw;
            flight_params.type_mask = TypeMask::XYZ_VEL_YAW;
        }
        else
        {
            // 停桨降落完成
            emergencyStop();
            control_mode = Control_Mode::INIT;
        }
    }

    // 如果无人机已经上锁，代表已经降落结束
    if (!px4_state.armed)
    {
        control_mode = Control_Mode::INIT;
        Logger::warning("Landing finished!");
    }

    setpoint_local_pub(flight_params.type_mask, local_setpoint);
}

// 获取悬停的期望值
void UAVControl::set_desired_from_hover()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if ((new_cmd && last_control_cmd.cmd != Hover) || control_cmd.cmd != Hover)
    {
        control_cmd.cmd = Hover;
        control_cmd.header.stamp = ros::Time::now();
        set_default_local_setpoint();
        set_hover_pos();
    }
    local_setpoint.position.x = flight_params.hover_pos[0];
    local_setpoint.position.y = flight_params.hover_pos[1];
    local_setpoint.position.z = flight_params.hover_pos[2];
    local_setpoint.yaw = flight_params.hover_yaw;
    flight_params.type_mask = TypeMask::XYZ_POS_YAW;
}

// 高级模式-返航模式的实现函数
void UAVControl::return_to_home()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        // 如果未设置home点，则无法进入返航模式
        if (!flight_params.home_set)
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
            local_setpoint.position.z = px4_state.pos[2];
            local_setpoint.yaw = flight_params.home_yaw;
            flight_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
    // 达到home点上方后，且速度降低后开始降落
    if ((px4_state.pos[0] - flight_params.home_pos[0]) < 0.15 &&
        (px4_state.pos[1] - flight_params.home_pos[1]) < 0.15 &&
        abs(px4_state.vel[0]) < 0.1 &&
        abs(px4_state.vel[1]) < 0.1 &&
        abs(px4_state.vel[2]) < 0.1)

    {
        control_cmd.cmd = Land;
        control_cmd.header.stamp = ros::Time::now();
    }
}

// 计算航点需要的yaw值
float UAVControl::get_yaw_from_waypoint(int type, float point_x, float point_y)
{
    if (type == 2)
    {
        float yaw = atan2(point_y - px4_state.pos[1],
                          point_x - px4_state.pos[0]);
        // 如果航点距离较近，则不改变yaw值
        if ((abs(px4_state.pos[0] - flight_params.home_pos[0]) < 0.2) &&
            (abs(px4_state.pos[1] - flight_params.home_pos[1]) < 0.2) && wp_params.wp_state != 4)
        {
            yaw = px4_state.att[2];
        }
        return yaw;
    }
    else if (type == 3)
    {
        // 指向环绕点
        return atan2(wp_params.wp_points[10][1] - px4_state.pos[1],
                     wp_params.wp_points[10][0] - px4_state.pos[0]);
    }
    else
    {
        return wp_params.wp_points[wp_params.wp_index][3];
    }
}

// 计算航点需要的速度
float UAVControl::get_vel_from_waypoint(float point_x, float point_y)
{
    wp_params.wp_x_vel = (point_x - px4_state.pos[0]) * wp_params.wp_vel_p;
    wp_params.wp_y_vel = (point_y - px4_state.pos[1]) * wp_params.wp_vel_p;
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
        wp_params.wp_point_takeoff[0] = px4_state.pos[0];
        wp_params.wp_point_takeoff[1] = px4_state.pos[1];
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
                control_cmd.cmd = Waypoint;
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
        if ((abs(px4_state.pos[0] - wp_params.wp_point_takeoff[0]) < 0.15) &&
            (abs(px4_state.pos[1] - wp_params.wp_point_takeoff[1]) < 0.15) &&
            (abs(px4_state.pos[2] - wp_params.z_height) < 0.15))
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
            flight_params.type_mask = TypeMask::XYZ_POS;
        }
        break;
    case 3:
        // 判断是否达到航点
        if ((abs(px4_state.pos[0] - wp_params.wp_points[wp_params.wp_index][0]) < 0.15) &&
            (abs(px4_state.pos[1] - wp_params.wp_points[wp_params.wp_index][1]) < 0.15) &&
            (abs(px4_state.pos[2] - wp_params.wp_points[wp_params.wp_index][2]) < 0.15))
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
                // flight_params.type_mask = TypeMask::XYZ_POS_YAW;
                flight_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
            }
        }
        break;
    case 4:
        // 如果航点结束需要返航
        if (wp_params.wp_end_type == 3)
        {
            // 到达返航点后降落
            if ((abs(px4_state.pos[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.pos[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.pos[2] - wp_params.z_height) < 0.15))
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
                local_setpoint.yaw = get_yaw_from_waypoint(2,
                                                           flight_params.home_pos[0],
                                                           flight_params.home_pos[1]);
                flight_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
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
            if ((abs(px4_state.att[2] - flight_params.home_yaw) < 0.0872) &&
                (abs(px4_state.pos[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.pos[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.pos[2] - wp_params.z_height) < 0.15))
            {
                Logger::warning("Mission completed! waiting for landing");
                control_cmd.cmd = Land;
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
                flight_params.type_mask = TypeMask::XYZ_POS_YAW;
            }
        }
        else if (wp_params.wp_end_type == 2)
        {
            // 原地降落
            Logger::warning("Mission completed! waiting for landing");
            control_cmd.cmd = Land;
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
        if (!flight_params.home_set)
        {
            Logger::error("Home position not set! Cannot takeoff!");
            set_desired_from_hover();
            return;
        }
        // 如果已经起飞，则不执行
        if ((px4_state.pos[0] - flight_params.home_pos[0]) > 0.1 ||
            (px4_state.pos[1] - flight_params.home_pos[1]) > 0.1 ||
            (px4_state.pos[2] - flight_params.home_pos[2]) > 0.1)
        {
            Logger::warning("UAV already takeoff!");
        }
        else
        {
            set_default_local_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = flight_params.home_pos[2] + takeoff_height;
            local_setpoint.yaw = flight_params.home_yaw;
            flight_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
}

// 进入降落模式
void UAVControl::set_land()
{
    if (control_mode != Control_Mode::LAND_CONTROL && (control_mode == Control_Mode::RC_CONTROL || control_mode == Control_Mode::CMD_CONTROL))
    {
        control_mode = Control_Mode::LAND_CONTROL;
        set_desired_from_land();
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