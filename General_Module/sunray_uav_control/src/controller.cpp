#include "mavros_control.h"

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

void mavros_control::set_desired_from_cmd()
{
    // 判断是否是新的指令
    bool new_cmd = control_cmd.header.stamp == last_control_cmd.header.stamp;
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
    else
    {
        auto it = moveModeMap.find(control_cmd.cmd);
        if (it != moveModeMap.end())
        {
            flight_params.type_mask = it->second;
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
}

void mavros_control::set_desired_from_land()
{
}

void mavros_control::set_desired_from_hover()
{
}