#include "mavros_control.h"

int mavros_control::safetyCheck()
{
    float time_diff = (ros::Time::now() - odom_valid_time).toSec();
    if (uav_state.position[0] < uav_geo_fence.x_min || uav_state.position[0] > uav_geo_fence.x_max || uav_state.position[1] < uav_geo_fence.y_min || uav_state.position[1] > uav_geo_fence.y_max || uav_state.position[2] < uav_geo_fence.z_min || uav_state.position[2] > uav_geo_fence.z_max)
    {
        return 1;
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

void mavros_control::px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    px4_state.pos[0] = msg->pose.pose.position.x;
    px4_state.pos[1] = msg->pose.pose.position.y;
    px4_state.pos[2] = msg->pose.pose.position.z;
    px4_state.vel[0] = msg->twist.twist.linear.x;
    px4_state.vel[1] = msg->twist.twist.linear.y;
    px4_state.vel[2] = msg->twist.twist.linear.z;
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

void mavros_control::px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_state.target_pos[0] = msg->position.x;
    px4_state.target_pos[1] = msg->position.y;
    px4_state.target_pos[2] = msg->position.z;
    px4_state.target_vel[0] = msg->velocity.x;
    px4_state.target_vel[1] = msg->velocity.y;
    px4_state.target_vel[2] = msg->velocity.z;
}

void mavros_control::odom_state_callback(const std_msgs::Bool::ConstPtr &msg)
{
    odom_valid_time = ros::Time::now();
    odom_valid = msg->data;
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
            if (safety_state == 0)
            {
                set_offboard_mode();
                control_mode = Control_Mode::CMD_CONTROL;
            }
            else
            {
                std::cout << "Safety state is not 0, cannot switch to CMD_CONTROL mode!" << std::endl;
            }
        }
        else if (msg->control_state == "RC_CONTROL")
        {
            if (safety_state == 0)
            {
                control_mode = Control_Mode::RC_CONTROL;
            }
            else
            {
                std::cout << "Safety state is not 0, cannot switch to RC_CONTROL mode!" << std::endl;
            }
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

void mavros_control::print_state(const ros::TimerEvent &event)
{
    Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_prefix, "<<<<<<<<<<<<<<<");
    Logger::color_no_del(int(LogColor::green), "Control Mode [", LOG_BLUE, modeMap[control_mode], LOG_GREEN, "]");

    if (px4_state.connected)
    {
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

void mavros_control::set_offboard_mode()
{
    set_default_setpoint();
    local_setpoint.velocity.x = 0.0;
    local_setpoint.velocity.y = 0.0;
    local_setpoint.velocity.z = 0.0;
    setpoint_pub(TypeMask::XYZ_VEL, local_setpoint);
    control_cmd.cmd = Hover;
    control_cmd.header.stamp = ros::Time::now();
    setMode("OFFBOARD");
}

void mavros_control::task_timer_callback(const ros::TimerEvent &event)
{
    // 安全检查
    safety_state = safetyCheck();
    if (safety_state == 1)
    {
        // 超出安全范围 进入降落模式
        control_mode = Control_Mode::LAND_CONTROL;
    }
    else if (safety_state == 2) // 定位数据失效
    {
        // 定位失效要要进入降落模式
        control_mode = Control_Mode::LAND_CONTROL;
    }
    else if (safety_state == 3) // 定位数据没有失效但是延迟较高
    {
        // 预留暂不做任何事情
    }
}

void mavros_control::body2ned(double body_xy[2], double ned_xy[2], double yaw)
{
    ned_xy[0] = cos(yaw) * body_xy[0] - sin(yaw) * body_xy[1];
    ned_xy[1] = sin(yaw) * body_xy[0] + cos(yaw) * body_xy[1];
}

void mavros_control::set_desired_from_cmd()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp != last_control_cmd.header.stamp;
    if (new_cmd)
    {
        if (control_cmd.cmd == Takeoff)
        {
            set_default_setpoint();
            local_setpoint.position.x = flight_params.home_pos[0];
            local_setpoint.position.y = flight_params.home_pos[1];
            local_setpoint.position.z = flight_params.home_pos[2] + takeoff_height;
            local_setpoint.yaw = flight_params.home_yaw;
            flight_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
        else if (control_cmd.cmd == Land)
        {
            control_mode = Control_Mode::LAND_CONTROL;
        }
        else if (control_cmd.cmd == Hover)
        {
            set_default_setpoint();
            set_hover_pos();
            local_setpoint.position.x = flight_params.hover_pos[0];
            local_setpoint.position.y = flight_params.hover_pos[1];
            local_setpoint.position.z = flight_params.hover_pos[2];
            local_setpoint.yaw = flight_params.hover_yaw;
            flight_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
        else if( control_cmd.cmd == XyzPosVelYaw)
        {
            set_default_setpoint();
            local_setpoint.position.x = control_cmd.desired_pos[0];
            local_setpoint.position.y = control_cmd.desired_pos[1];
            local_setpoint.position.z = control_cmd.desired_pos[2];
            local_setpoint.velocity.x = control_cmd.desired_vel[0];
            local_setpoint.velocity.y = control_cmd.desired_vel[1];
            local_setpoint.velocity.z = control_cmd.desired_vel[2];
            local_setpoint.yaw = control_cmd.desired_yaw;
            flight_params.type_mask = TypeMask::XYZ_POS_VEL_YAW;
            std::cout << "set XyzPosVelYaw" << std::endl;
        }
        else
        {
            auto it = moveModeMap.find(control_cmd.cmd);
            if (it != moveModeMap.end())
            {
                flight_params.type_mask = it->second;

                if (control_cmd.cmd == XyzPosYawBody ||
                    control_cmd.cmd == XyzVelYawBody ||
                    control_cmd.cmd == XyVelZPosYawBody)
                {
                    set_default_setpoint();
                    // Body系的需要转换到NED下
                    local_setpoint.position.x =
                        control_cmd.desired_pos[0] * cos(px4_state.att[2]) - control_cmd.desired_pos[1] * sin(px4_state.att[2]);
                    local_setpoint.position.y =
                        control_cmd.desired_pos[1] * sin(px4_state.att[2]) + control_cmd.desired_pos[1] * cos(px4_state.att[2]);
                    local_setpoint.position.z = control_cmd.desired_pos[2] + px4_state.pos[2];

                    local_setpoint.velocity.x =
                        control_cmd.desired_vel[0] * cos(px4_state.att[2]) - control_cmd.desired_vel[1] * sin(px4_state.att[2]);
                    local_setpoint.velocity.y =
                        control_cmd.desired_vel[1] * sin(px4_state.att[2]) + control_cmd.desired_vel[1] * cos(px4_state.att[2]);
                    local_setpoint.velocity.z = control_cmd.desired_vel[2];

                    local_setpoint.yaw = control_cmd.desired_yaw + px4_state.att[2];
                    flight_params.type_mask = moveModeMap[control_cmd.cmd];
                }
                else
                {
                    set_default_setpoint();
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
                std::cout << "Unknown command!" << std::endl;
                if (new_cmd)
                {
                    set_default_setpoint();
                    set_hover_pos();
                    local_setpoint.position.x = flight_params.hover_pos[0];
                    local_setpoint.position.y = flight_params.hover_pos[1];
                    local_setpoint.position.z = flight_params.hover_pos[2];
                    local_setpoint.yaw = flight_params.hover_yaw;
                    flight_params.type_mask = TypeMask::XYZ_POS_YAW;
                };
            }
        }
    }

    setpoint_pub(flight_params.type_mask, local_setpoint);
    last_control_cmd = control_cmd;
}

void mavros_control::set_desired_from_rc()
{
    if (last_control_mode != control_mode)
    {
        flight_params.last_rc_time = ros::Time::now();
    }
    double delta_t = (ros::Time::now() - flight_params.last_rc_time).toSec();
    flight_params.last_rc_time = ros::Time::now();
    double body_xy[2], enu_xy[2];
    body_xy[0] = rc_input.ch[1] * flight_params.max_vel_xy * delta_t;
    body_xy[1] = -rc_input.ch[0] * flight_params.max_vel_xy * delta_t;

    body2ned(body_xy, enu_xy, px4_state.att[2]);

    // 悬停位置 = 前一个悬停位置 + 遥控器数值[-1,1] * 0.01(如果主程序中设定是100Hz的话)
    local_setpoint.position.x = enu_xy[0] + px4_state.pos[0];
    local_setpoint.position.y = enu_xy[1] + px4_state.pos[1];
    local_setpoint.position.z = rc_input.ch[2] * flight_params.max_vel_z * delta_t + px4_state.pos[2];
    local_setpoint.yaw = -rc_input.ch[3] * flight_params.max_vel_yaw * delta_t + px4_state.att[2];
    // 因为这是一个积分系统，所以即使停杆了，无人机也还会继续移动一段距离
    flight_params.type_mask = TypeMask::XYZ_POS_YAW;
}

void mavros_control::set_desired_from_land()
{
    bool new_cmd = control_mode != last_control_mode;
    if (new_cmd)
    {
        flight_params.last_land_time = ros::Time(0);
        set_default_setpoint();
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
        set_default_setpoint();
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
    }

    setpoint_pub(flight_params.type_mask, local_setpoint);
}

void mavros_control::set_desired_from_hover()
{
}