#include "uav_control.h"

void UAVControl::init(ros::NodeHandle& nh)
{
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 0);
    // 【参数】无人机名称
    nh.param<string>("uav_name", uav_name, "uav");
    uav_name = uav_name + std::to_string(uav_id);
    node_name = ros::this_node::getName();
    // 【参数】是否仿真模式
    nh.param<bool>("sim_mode", sim_mode, true);
    // 【参数】默认起飞高度
    nh.param<float>("uav_control/Takeoff_height", Takeoff_height, 1.0);
    // 【参数】降落时自动上锁高度
    nh.param<float>("uav_control/Disarm_height", Disarm_height, 0.2);
    // 【参数】降落速度
    nh.param<float>("uav_control/Land_speed", Land_speed, 0.2);
    // 【参数】地理围栏
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0);
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0);
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);
    // 【函数】打印参数
    printf_param();

    string topic_prefix = "/" + uav_name;
    // 【订阅】无人机状态 -- vision_pose -> 本节点
    uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1, &UAVControl::uav_state_cb, this);
    // 【订阅】外部控制指令 -- 外部节点 -> 本节点
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1, &UAVControl::control_cmd_cb, this);
    // 【订阅】无人机设置指令 -- 外部节点 -> 本节点
    uav_setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1, &UAVControl::uav_setup_cb, this);
    //【订阅】PX4中无人机的位置/速度/加速度设定值 -- mavros -> 本节点
    px4_position_target_sub =
        nh.subscribe<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/target_local",
                                                  1,
                                                  &UAVControl::px4_pos_target_cb, this);

    //【订阅】飞控遥控器数据 -- 飞控 -> 本节点 
    string rc_topic_name;
    if (sim_mode)
    {
        rc_topic_name = topic_prefix + "/sunray/fake_rc_in";
    }
    else
    {
        rc_topic_name = topic_prefix + "/mavros/rc/in";
    }
    px4_rc_sub = nh.subscribe<mavros_msgs::RCIn>(rc_topic_name, 1, &UAVControl::px4_rc_cb, this);

    // 【发布】本地位置控制指令，包括期望位置、速度、加速度等接口 坐标系:ENU系 -- 本节点->飞控
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/local", 1);
    // 【发布】姿态控制指令，包括期望姿态等接口 -- 本节点->飞控
    setpoint_raw_attitude_pub = nh.advertise<sunray_msgs::AttitudeSetpoint>(topic_prefix + "/mavros/setpoint_raw/attitude", 1);
    // 【发布】全局位置控制指令，包括期望经纬度等接口 坐标系:WGS84坐标系 -- 本节点->飞控
    setpoint_raw_global_pub = nh.advertise<sunray_msgs::GlobalPositionSetpoint>(topic_prefix + "/mavros/setpoint_raw/global", 1);
    // 【发布】无人机状态+cmd 
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state_cmd", 1);

    // 【服务】解锁/上锁 -- 本节点->飞控
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(topic_prefix + "/mavros/cmd/arming");
    // 【服务】修改系统模式 -- 本节点->飞控
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(topic_prefix + "/mavros/set_mode");
    // 【服务】紧急上锁服务(KILL) -- 本节点->飞控
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");
    // 【服务】重启PX4飞控 -- 本节点->飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");
    
    // 状态初始化
    control_mode = Control_Mode::INIT;
    control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;

    desired_state.pos << 0.0, 0.0, 0.0;
    desired_state.vel << 0.0, 0.0, 0.0;
    desired_state.acc << 0.0, 0.0, 0.0;
    desired_state.att << 0.0, 0.0, 0.0;
    desired_state.yaw = 0.0;
    desired_state.yaw_rate = 0.0;
    desired_state.thrust = 0.0;
    desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    
    uav_pos.setZero();
    uav_vel.setZero();
    u_att.setZero();
    rc_input.init();

    quick_land = false;
    set_landing_des = false;
    check_off = false;

    cout << GREEN << node_name << " init! " << TAIL << endl;
}

int UAVControl::safety_check()
{
    // 一般不会出现，除非发送了重启飞控指令，或者飞控连接线物理断裂
    if (!uav_state.connected)
    {
        cout << RED << uav_name << ":----> Failsafe: Waiting for PX4 connection!" << TAIL << endl;
        return -1;
    }

    if (uav_state.position[0] < uav_geo_fence.x_min || uav_state.position[0] > uav_geo_fence.x_max ||
        uav_state.position[1] < uav_geo_fence.y_min || uav_state.position[1] > uav_geo_fence.y_max ||
        uav_state.position[2] < uav_geo_fence.z_min || uav_state.position[2] > uav_geo_fence.z_max)
    {
        cout << RED << uav_name << ":----> Failsafe: Out of the geo fence, swtich to land control mode!" << TAIL << endl;
        return 1;
    }
    else if (!uav_state.odom_valid)
    {
        cout << RED << uav_name << ":----> Failsafe: Odom invalid, swtich to land control mode!" << TAIL << endl;
        return 2;
    }
    else
    {
        return 0;
    }
}

void UAVControl::mainloop()
{
    // 安全检查
    if (control_mode == Control_Mode::RC_CONTROL || control_mode == Control_Mode::CMD_CONTROL)
    {
        // 安全检查 - 包括地理围栏、定位有效性检查
        int safety_flag = safety_check();

        if (safety_flag == -1)
        {
            // 与PX4断开连接，直接返回
            return;
        }
        else if (safety_flag == 1)
        {
            // 超出geofence，原地降落
            control_mode = Control_Mode::LAND_CONTROL;
        }
        else if (safety_flag == 2)
        {
            // 检测到odom失效，快速降落
            quick_land = true;
            control_mode = Control_Mode::LAND_CONTROL;
        }

        // 检查是否满足维持在RC_POS_CONTROL的条件，不满足则自动退出
        if (uav_state.mode != "OFFBOARD" && check_off)
        {
            check_off = false;
            // 进入RC_POS_CONTROL，需启动OFFBOARD模式
            mavros_msgs::PositionTarget pos_setpoint;
            pos_setpoint.header.stamp = ros::Time::now();
            pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                    mavros_msgs::PositionTarget::IGNORE_PY |
                                    mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            pos_setpoint.velocity.x = 0.0;
            pos_setpoint.velocity.y = 0.0;
            pos_setpoint.velocity.z = 0.0;
            setpoint_raw_local_pub.publish(pos_setpoint);
            // 只会在第一次进入offboard 并置于hover模式 确保不会错误有预设点 且遥控拥有最高权限
            set_hover_pose_with_odom();
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            set_px4_mode_func("OFFBOARD");
        }
    }

    switch (control_mode)
    {
    case Control_Mode::INIT:
        // 检查无人机是否位于定点模式，否则切换至定点模式
        if (uav_state.mode != "POSCTL")
        {
            set_px4_mode_func("POSCTL");
        }
        break;

    case Control_Mode::RC_CONTROL:
        get_desired_state_from_rc();
        desired_state.pos = Hover_position;
        desired_state.vel << 0.0, 0.0, 0.0;
        desired_state.acc << 0.0, 0.0, 0.0;
        desired_state.yaw = Hover_yaw;
        // RC_CONTROL控制模式下，使用位置指令控制接口
        send_local_pos_setpoint(desired_state.pos, desired_state.yaw, false);
        break;

    case Control_Mode::CMD_CONTROL:
        // 设置期望值
        get_desired_state_from_cmd();
        break;

    // 当前位置原地降落，降落后会自动上锁，且切换为mannual模式
    case Control_Mode::LAND_CONTROL:
        
        // 第一次进入，设置降落的期望位置和速度
        if (!set_landing_des)
        {
            // 快速降落 - 一般用于无人机即将失控时，快速降落保证安全
            if (quick_land)
            {
                Land_speed = 1.0;
            }
            pos_des[0] = uav_pos[0];
            pos_des[1] = uav_pos[1];
            pos_des[2] = Takeoff_position[2]; // 高度设定为初始起飞时的高度
            vel_des << 0.0, 0.0, -Land_speed;
            acc_des << 0.0, 0.0, 0.0;
            yaw_des = uav_yaw;
            set_landing_des = true;
        }
        // 当无人机位置低于指定高度时，自动上锁
        // 需要考虑万一高度数据不准确时，从高处自由落体
        if (uav_pos[2] < Disarm_height)
        {
            // 进入急停
            enable_emergency_func();
        }

        // 降落结束的标志：无人机上锁
        if (!uav_state.armed)
        {
            control_mode = Control_Mode::INIT;
            //控制命令初始化,不初始化将影响setup接口切换command_control模式
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            set_landing_des = false;
        }
        // LAND_CONTROL控制模式下，使用位置+速度的控制接口
        send_pos_vel_xyz_setpoint(pos_des, vel_des, yaw_des);
    }
}

void UAVControl::set_hover_pose_with_odom()
{
    // 设定悬停点
    Hover_position[0] = uav_pos[0];
    Hover_position[1] = uav_pos[1];
    Hover_position[2] = uav_pos[2];
    Hover_yaw = uav_yaw;

    last_set_hover_pose_time = ros::Time::now();
}

void UAVControl::get_desired_state_from_rc()
{
    ros::Time now = ros::Time::now();
    double delta_t = (now - last_set_hover_pose_time).toSec();
    last_set_hover_pose_time = now;

    double max_vel_xy = 1.5;
    double max_vel_z = 1.3;
    double max_vel_yaw = 1.5;

    float body_xy[2], enu_xy[2];
    body_xy[0] = rc_input.ch[1] * max_vel_xy * delta_t;
    body_xy[1] = -rc_input.ch[0] * max_vel_xy * delta_t;

    rotation_yaw(Hover_yaw, body_xy, enu_xy);

    // 悬停位置 = 前一个悬停位置 + 遥控器数值[-1,1] * 0.01(如果主程序中设定是100Hz的话)
    Hover_position(0) += enu_xy[0];
    Hover_position(1) += enu_xy[1];
    Hover_position(2) += rc_input.ch[2] * max_vel_z * delta_t;
    Hover_yaw += -rc_input.ch[3] * max_vel_yaw * delta_t;
    // 因为这是一个积分系统，所以即使停杆了，无人机也还会继续移动一段距离

    // 高度限制
    if (Hover_position(2) < 0.2)
        Hover_position(2) = 0.2;
}

// 【坐标系旋转函数】- 机体系到enu系
// body_frame是机体系,enu_frame是惯性系,yaw_angle是当前偏航角[rad]
void UAVControl::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    enu_frame[0] = body_frame[0] * cos(yaw_angle) - body_frame[1] * sin(yaw_angle);
    enu_frame[1] = body_frame[0] * sin(yaw_angle) + body_frame[1] * cos(yaw_angle);
}

// 从UAVControlCMD中提取指令，并设置期望值
void UAVControl::get_desired_state_from_cmd()
{
    //【Move】 移动，移动子模式的区别详见UAVCommand.msg中的说明
    switch (control_cmd.cmd)
    {
        case sunray_msgs::UAVControlCMD::Takeoff:
        {
            desired_state.pos << Takeoff_position + Eigen::Vector3d(0, 0, Takeoff_height);
            desired_state.vel << 0.0, 0.0, 0.0;
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.yaw = 0.0;
            desired_state.yaw_rate = 0.0;
            desired_state.thrust = 0.0;
            desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            send_local_pos_setpoint(desired_state.pos, desired_state.yaw);
            break;
        }  
        case sunray_msgs::UAVControlCMD::Hover:
        {
            if (last_control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover)
            {
                set_hover_pose_with_odom();
            }
            desired_state.pos << Hover_position;
            desired_state.vel << 0.0, 0.0, 0.0;
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.yaw = Hover_yaw;
            desired_state.yaw_rate = 0.0;
            desired_state.thrust = 0.0;
            // 需要转化成q
            //desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            send_local_pos_setpoint(desired_state.pos, desired_state.yaw);
            break;
        } 
        case sunray_msgs::UAVControlCMD::Land:
        {
            control_mode = Control_Mode::LAND_CONTROL;
            set_landing_des = false;
            break;
        }  
        case sunray_msgs::UAVControlCMD::XYZ_POS:
        {
            desired_state.pos[0] = control_cmd.desired_pos[0];
            desired_state.pos[1] = control_cmd.desired_pos[1];
            desired_state.pos[2] = control_cmd.desired_pos[2];
            desired_state.vel << 0.0, 0.0, 0.0;
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.yaw_rate = 0.0;
            desired_state.thrust = 0.0;
            // 需要转化成q
            //desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw = control_cmd.desired_yaw;
                desired_state.yaw_rate = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw, control_cmd.enable_yawRate);
            }
            
            break;
        }  
        case sunray_msgs::UAVControlCMD::XYZ_VEL:
        {
            desired_state.pos << 0.0, 0.0, 0.0;
            desired_state.vel[0] = control_cmd.desired_vel[0];
            desired_state.vel[1] = control_cmd.desired_vel[1];
            desired_state.vel[2] = control_cmd.desired_vel[2];
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.thrust = 0.0;
            // 需要转化成q
            //desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_local_vel_setpoint(desired_state.vel, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw = control_cmd.desired_yaw;
                desired_state.yaw_rate = 0.0;
                send_local_vel_setpoint(desired_state.vel, desired_state.yaw, control_cmd.enable_yawRate);
            }        
            break;
        }
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS:
        {
            // todo
            desired_state.pos[0] = 0.0;
            desired_state.pos[1] = 0.0;
            desired_state.pos[2] = control_cmd.desired_pos[2];
            desired_state.vel[0] = control_cmd.desired_vel[0];
            desired_state.vel[1] = control_cmd.desired_vel[1];
            desired_state.vel[2] = 0.0;
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.thrust = 0.0;
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_vel_xy_pos_z_setpoint(desired_state.pos, desired_state.vel, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw = control_cmd.desired_yaw;
                desired_state.yaw_rate = 0.0;
                send_vel_xy_pos_z_setpoint(desired_state.pos, desired_state.vel, desired_state.yaw, control_cmd.enable_yawRate);
            }  
            break;
        }
        case sunray_msgs::UAVControlCMD::XYZ_POS_BODY:
        {
            // 【XYZ_POS_BODY】XYZ位置转换为惯性系
            // 机体系的定点控制，必须使得cmd_id 递增，否则无人机会持续移动
            if (control_cmd.cmd_id > last_control_cmd.cmd_id)
            {
                float d_pos_body[2] = {control_cmd.desired_pos[0], control_cmd.desired_pos[1]};
                float d_pos_enu[2];
                rotation_yaw(uav_yaw, d_pos_body, d_pos_enu);

                desired_state.pos[0] = uav_pos[0] + d_pos_enu[0];
                desired_state.pos[1] = uav_pos[1] + d_pos_enu[1];
                desired_state.pos[2] = uav_pos[2] + control_cmd.desired_pos[2];
                desired_state.vel << 0.0, 0.0, 0.0;
                desired_state.acc << 0.0, 0.0, 0.0;
                desired_state.att << 0.0, 0.0, 0.0;
                desired_state.yaw = control_cmd.desired_yaw + uav_yaw;
            }
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw_rate = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw, control_cmd.enable_yawRate);
            }
            break;
        }
        case sunray_msgs::UAVControlCMD::XYZ_VEL_BODY:
        {
            // todo
            if (control_cmd.cmd_id > last_control_cmd.cmd_id)
            {
                float d_vel_body[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                float d_vel_enu[2];
                rotation_yaw(uav_yaw, d_vel_body, d_vel_enu);
                desired_state.vel[0] = d_vel_enu[0];
                desired_state.vel[1] = d_vel_enu[1];
                desired_state.vel[2] = d_vel_enu[2];
                desired_state.pos << 0.0, 0.0, 0.0;
                desired_state.acc << 0.0, 0.0, 0.0;
                desired_state.att << 0.0, 0.0, 0.0;
                desired_state.yaw = control_cmd.desired_yaw + uav_yaw;
                desired_state.yaw_rate = control_cmd.desired_yaw_rate;
            }
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw_rate = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw, control_cmd.enable_yawRate);
            }
            break;
        }
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY:
        {
            if (control_cmd.cmd_id > last_control_cmd.cmd_id)
            {
                float d_vel_body[2] = {control_cmd.desired_vel[0], control_cmd.desired_vel[1]};
                float d_vel_enu[2];
                rotation_yaw(uav_yaw, d_vel_body, d_vel_enu);
                desired_state.vel[0] = d_vel_enu[0];
                desired_state.vel[1] = d_vel_enu[1];
                desired_state.vel[2] = d_vel_enu[2];
                desired_state.pos[0] = 0.0;
                desired_state.pos[1] = 0.0;
                desired_state.pos[2] = uav_pos[2] + control_cmd.desired_pos[2];
                desired_state.acc << 0.0, 0.0, 0.0;
                desired_state.att << 0.0, 0.0, 0.0;
                desired_state.yaw = control_cmd.desired_yaw + uav_yaw;
            }
            if(control_cmd.enable_yawRate){
                desired_state.yaw_rate = control_cmd.desired_yaw;
                desired_state.yaw = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw_rate, control_cmd.enable_yawRate);
            }
            else{
                desired_state.yaw_rate = 0.0;
                send_local_pos_setpoint(desired_state.pos, desired_state.yaw, control_cmd.enable_yawRate);
            }
            break;
        }
        case sunray_msgs::UAVControlCMD::TRAJECTORY:
        {
            for (int i = 0; i < 3; i++)
            {
                desired_state.pos[i] = control_cmd.desired_pos[i];
                desired_state.vel[i] = control_cmd.desired_vel[i];
            }
            desired_state.yaw = control_cmd.desired_yaw;
            send_pos_vel_xyz_setpoint(desired_state.pos, desired_state.vel, desired_state.yaw, control_cmd.enable_yawRate);
            break;
        }
        case sunray_msgs::UAVControlCMD::XYZ_ATT:
        {
            desired_state.pos << 0.0, 0.0, 0.0;
            desired_state.vel << 0.0, 0.0, 0.0;
            desired_state.acc << 0.0, 0.0, 0.0;
            desired_state.att << 0.0, 0.0, 0.0;
            desired_state.att[0] = control_cmd.desired_att[0];
            desired_state.att[1] = control_cmd.desired_att[1];
            desired_state.att[2] = control_cmd.desired_att[2];
            desired_state.yaw = 0.0;
            desired_state.yaw_rate = 0.0;
            desired_state.thrust = control_cmd.desired_thrust;
            desired_state.q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            send_attitude_setpoint(desired_state.att, desired_state.thrust);  
            break;
        }
        case sunray_msgs::UAVControlCMD::LAT_LON_ALT:
        {
            desired_state.global_pos[0] = control_cmd.latitude;
            desired_state.global_pos[1] = control_cmd.longitude;
            desired_state.global_pos[2] = control_cmd.altitude;
            desired_state.yaw = control_cmd.desired_yaw;
            send_global_pos_setpoint(desired_state.global_pos, desired_state.yaw);
            break;
        }
        default:
        {
            cout << RED << node_name << "Wrong command!" << TAIL << endl;
            break;
        }
    }
    // 记录上一时刻命令
    last_control_cmd = control_cmd;
}

void UAVControl::printf_debug_info()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] State    <<<<<<<<<<<<<<<<<<" << TAIL << endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    printf_uav_state();
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>> UAV [" << uav_id << "] Control  <<<<<<<<<<<<<<<<<<" << TAIL << endl;

    switch (control_mode)
    {
    case Control_Mode::INIT:
        cout << GREEN << "Control_Mode: [ INIT ] " << TAIL << endl;
        break;

    case Control_Mode::RC_CONTROL:
        cout << GREEN << "Control_Mode: [ RC_CONTROL ] " << TAIL << endl;
        cout << GREEN << "Hover_Pos [X Y Z] : " << Hover_position[0] << " [ m ] " << Hover_position[1] << " [ m ] " << Hover_position[2] << " [ m ] " << TAIL << endl;
        break;

    case Control_Mode::CMD_CONTROL:
        cout << GREEN << "Control_Mode: [ CMD_CONTROL ] " << TAIL << endl;
        break;
    case Control_Mode::LAND_CONTROL:
        if (quick_land)
        {
            cout << GREEN << "Control_Mode: [ LAND_CONTROL ] - quick land mode " << TAIL << endl;
        }
        else
        {
            cout << GREEN << "Control_Mode: [ LAND_CONTROL ] " << TAIL << endl;
        }
        break;
    }

    // 打印 CMD_CONTROL控制模式下 指令信息
    if (control_mode == Control_Mode::CMD_CONTROL)
    {
        switch (control_cmd.cmd)
        {
        case sunray_msgs::UAVControlCMD::Takeoff:
            cout << GREEN << "Command: [ Takeoff ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::Hover:
            cout << GREEN << "Command: [ Hover ] " << TAIL << endl;
            cout << GREEN << "Hover_Pos  [X Y Z] : " << Hover_position[0] << " [ m ] " << Hover_position[1] << " [ m ] " << Hover_position[2] << " [ m ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::Land:
            cout << GREEN << "Command: [ Land ] " << TAIL << endl;
            break;

        case sunray_msgs::UAVControlCMD::XYZ_POS:
            cout << GREEN << "Command: [ Move in XYZ_POS ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [X Y Z] : " << control_cmd.desired_pos[0] << " [ m ] " << control_cmd.desired_pos[1] << " [ m ] " << control_cmd.desired_pos[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS:
            cout << GREEN << "Command: [ Move in XY_VEL_Z_POS ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [    Z] : " << control_cmd.desired_pos[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y  ] : " << control_cmd.desired_vel[0] << " [m/s] " << control_cmd.desired_vel[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_VEL:
            cout << GREEN << "Command: [ Move in XYZ_VEL ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y Z] : " << control_cmd.desired_vel[0] << " [m/s] " << control_cmd.desired_vel[1] << " [m/s] " << control_cmd.desired_vel[2] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_POS_BODY:
            cout << GREEN << "Command: [ Move in XYZ_POS_BODY ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [X Y Z] : " << control_cmd.desired_pos[0] << " [ m ] " << control_cmd.desired_pos[1] << " [ m ] " << control_cmd.desired_pos[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_VEL_BODY:
            cout << GREEN << "Command: [ Move in XYZ_VEL_BODY ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y Z] : " << control_cmd.desired_vel[0] << " [m/s] " << control_cmd.desired_vel[1] << " [m/s] " << control_cmd.desired_vel[2] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XY_VEL_Z_POS_BODY:
            cout << GREEN << "Command: [ Move in XY_VEL_Z_POS_BODY ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [    Z] : " << control_cmd.desired_pos[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y  ] : " << control_cmd.desired_vel[0] << " [m/s] " << control_cmd.desired_vel[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::TRAJECTORY:
            cout << GREEN << "Command: [ Move in TRAJECTORY ] " << TAIL << endl;
            cout << GREEN << "Pos_ref [X Y Z] : " << control_cmd.desired_pos[0] << " [ m ] " << control_cmd.desired_pos[1] << " [ m ] " << control_cmd.desired_pos[2] << " [ m ] " << TAIL << endl;
            cout << GREEN << "Vel_ref [X Y Z] : " << control_cmd.desired_vel[0] << " [m/s] " << control_cmd.desired_vel[1] << " [m/s] " << control_cmd.desired_vel[2] << " [m/s] " << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::XYZ_ATT:
            cout << YELLOW << node_name << " Send control cmd from EXTERNAL_CONTROLLER, be careful! " << TAIL << endl;
            cout << GREEN << "Command: [ Move in XYZ_ATT ] " << TAIL << endl;
            cout << GREEN << "Att_ref [X Y Z] : " << control_cmd.desired_att[0] * 180 / M_PI << " [deg] " << control_cmd.desired_att[1] * 180 / M_PI << " [deg] " << control_cmd.desired_att[2] * 180 / M_PI << " [deg] " << TAIL << endl;
            cout << GREEN << "Thrust_ref[0-1] : " << control_cmd.desired_thrust << TAIL << endl;
            break;
        case sunray_msgs::UAVControlCMD::LAT_LON_ALT:
            cout << GREEN << "Command: [ Move in LAT_LON_ALT ] " << TAIL << endl;
            cout << GREEN << "LAT : " << control_cmd.latitude << " LON :  " << control_cmd.longitude << " ALT : " << control_cmd.altitude << TAIL << endl;
            cout << GREEN << "Yaw_ref : " << control_cmd.desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;
            break;
        default:
            cout << GREEN << "Command: [ Unknown Mode ]. " << TAIL << endl;
            break;
        }
    }

    // 打印PX4回传信息用于验证
    cout << GREEN << "Pos_target [X Y Z] : " << px4_pos_target[0] << " [ m ] " << px4_pos_target[1] << " [ m ] " << px4_pos_target[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel_target [X Y Z] : " << px4_vel_target[0] << " [m/s] " << px4_vel_target[1] << " [m/s] " << px4_vel_target[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "Yaw_target         : " << px4_yaw_target * 180 / M_PI << " [deg] " << TAIL << endl;
    cout << GREEN << "Thr_target [ 0-1 ] : " << px4_thrust_target << TAIL << endl;
    if (control_cmd.cmd == sunray_msgs::UAVControlCMD::XYZ_ATT)
    {
        cout << GREEN << "Att_target [X Y Z] : " << px4_att_target[0] * 180 / M_PI << " [deg] " << px4_att_target[1] * 180 / M_PI << " [deg] " << px4_att_target[2] * 180 / M_PI << " [deg] " << TAIL << endl;
        cout << GREEN << "Thr_target [ 0-1 ] : " << px4_thrust_target << TAIL << endl;
    }
}

void UAVControl::printf_uav_state()
{
    // 打印 无人机状态
    if (uav_state.connected == true)
    {
        cout << GREEN << "PX4 Status:  [ Connected ] ";
    }
    else
    {
        cout << RED << "PX4 Status:[ Unconnected ] ";
    }
    //是否上锁
    if (uav_state.armed == true)
    {
        cout << GREEN << "[  Armed   ] ";
    }
    else
    {
        cout << RED << "[ DisArmed ] ";
    }

    cout << "[ " << uav_state.mode << " ] " << TAIL << endl;

    // 打印外部输入的原始数据
    switch (uav_state.location_source)
    {
    case sunray_msgs::ExternalOdom::MOCAP:
        cout << GREEN << "External Odom: [ MOCAP ] ";
        break;
    case sunray_msgs::ExternalOdom::VIOBOT:
        cout << GREEN << "External Odom: [ VIOBOT ] ";
        break;
    case sunray_msgs::ExternalOdom::GAZEBO:
        cout << GREEN << "External Odom: [ GAZEBO ] ";
        break;
    case sunray_msgs::ExternalOdom::VINS:
        cout << GREEN << "External Odom: [ VINS ] ";
        break;
    }

    if (uav_state.odom_valid)
    {
        cout << GREEN << " Odom Status : [ Valid ] " << TAIL << endl;
    }
    else
    {
        cout << RED   << " Odom Status : [ Invalid ] " << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y Z] : " << uav_state.position[0] << " [ m ] " << uav_state.position[1] << " [ m ] " << uav_state.position[2] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y Z] : " << uav_state.velocity[0] << " [m/s] " << uav_state.velocity[1] << " [m/s] " << uav_state.velocity[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << uav_state.attitude[0] * 180 / M_PI << " [deg] " << uav_state.attitude[1] * 180 / M_PI << " [deg] " << uav_state.attitude[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    cout << GREEN << "Battery Voltage : " << uav_state.battery_state << " [V] "
         << " Battery Percent : " << uav_state.battery_percetage << " [%] "<< TAIL << endl;
}


void UAVControl::control_cmd_cb(const sunray_msgs::UAVControlCMD::ConstPtr &msg)
{
    control_cmd = *msg;
    // if(msg->cmd == sunray_msgs::UAVControlCMD::Takeoff){
    //     cout<< "cmd:"<< *msg << endl;
    //     cout<< "control_cmd:"<< control_cmd << endl;
    //     uint8_t data = msg->cmd;
    //     cout<< "cmd:"<< msg->cmd <<"."<< endl;
    // }
    // else{
    //     cout<< "Init:"<< sunray_msgs::UAVControlCMD::Init <<"."<< endl;
    // }
}

void UAVControl::uav_state_cb(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;

    uav_pos = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
    uav_vel = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);

    uav_quat.w() = msg->attitude_q.w;
    uav_quat.x() = msg->attitude_q.x;
    uav_quat.y() = msg->attitude_q.y;
    uav_quat.z() = msg->attitude_q.z;

    uav_yaw = geometry_utils::get_yaw_from_quaternion(uav_quat);

    // 在无人机解锁时，将无人机解锁位置设定为起飞点
    if (uav_state.armed && !uav_state_last.armed)
    {
        Takeoff_position[0] = uav_pos[0];
        Takeoff_position[1] = uav_pos[1];
        Takeoff_position[2] = uav_pos[2];
        Takeoff_yaw = uav_yaw;
        cout << BLUE << "Set Takeoff_position [X Y Z] : " << Takeoff_position[0] << " [ m ] " << Takeoff_position[1] << " [ m ] " << Takeoff_position[2] << " [ m ] " << TAIL << endl;
        cout << BLUE << "Set Takeoff_yaw : " << Takeoff_yaw/3.1415926*180 << " [ deg ] " << TAIL << endl;
    }

    uav_state_last = uav_state;

    uav_state.control_mode = control_mode;
    uav_state_pub.publish(uav_state);
}

void UAVControl::px4_rc_cb(const mavros_msgs::RCIn::ConstPtr &msg)
{
    // 调用外部函数对遥控器数据进行处理，具体见rc_data.h，此时rc_input中的状态已更新
    rc_input.handle_rc_data(msg);

    // 重启PX4飞控，条件: 无人机已上锁
    if (rc_input.toggle_reboot && !uav_state.armed)
    {
        rc_input.toggle_reboot = false;
        reboot_PX4();
        return;
    }

    // 紧急KILL指令
    if (rc_input.toggle_kill)
    {
        rc_input.toggle_kill = false;
        enable_emergency_func();
        return;
    }

    // 解锁，条件: 无人机已上锁
    if (rc_input.toggle_arm)
    {
        rc_input.toggle_arm = false;
        arm_disarm_func(true);
        return;
    }

    // 上锁，条件: 无人机已解锁
    // PX4代码规定:无人机必须处于地面时才可以上锁
    if (rc_input.toggle_disarm)
    {
        rc_input.toggle_arm = false;
        arm_disarm_func(false);
        return;
    }

    // 自动降落，条件: 必须在RC_POS_CONTROL或者COMMAND_CONTROL模式才可以触发
    bool if_in_hover_or_command_mode =
        control_mode == Control_Mode::RC_CONTROL || control_mode == Control_Mode::CMD_CONTROL;
    if (rc_input.toggle_land && if_in_hover_or_command_mode)
    {
        rc_input.toggle_land = false;
        control_mode = Control_Mode::LAND_CONTROL;
        set_landing_des = false;
        return;
    }

    // 如果无人机没有解锁，则不需要判断模式切换指令，直接返回
    if (!uav_state.armed)
    {
        return;
    }

    // 如果无人机处于LAND_CONTROL下，单独判断无人机模式切换指令
    if (control_mode == Control_Mode::LAND_CONTROL)
    {
        // 收到进入INIT指令，且不在INIT模式时
        if (rc_input.enter_init)
        {
            rc_input.enter_init = false;
            control_mode = Control_Mode::INIT;
            cout << BLUE << node_name << " Switch to INIT" << TAIL << endl;
        }

        if (rc_input.enter_rc_pos_control)
        {
            rc_input.enter_rc_pos_control = false;
            // odom失效，拒绝进入
            if (!uav_state.odom_valid)
            {
                this->text_info.MessageType = sunray_msgs::TextInfo::ERROR;
                this->text_info.Message = "Reject RC_CONTROL. Odom invalid!";
                cout << RED << node_name << " Reject RC_CONTROL. Odom invalid! " << TAIL << endl;
                return;
            }
            // 切换至RC_POS_CONTROL
            control_mode = Control_Mode::RC_CONTROL;
            // 初始化默认的UAVCommand
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            // 进入RC_POS_CONTROL，需设置初始悬停点
            set_hover_pose_with_odom();
            cout << BLUE << node_name << " Switch to RC_CONTROL" << TAIL << endl;
            return;
        }

        return;
    }

    // 收到进入INIT指令，且不在INIT模式时
    if (rc_input.enter_init && control_mode != Control_Mode::INIT)
    {
        rc_input.enter_init = false;
        control_mode = Control_Mode::INIT;
        cout << GREEN << node_name << " Switch to INIT" << TAIL << endl;
    }

    // 收到进入RC_POS_CONTROL指令，且不在RC_POS_CONTROL模式时
    if (rc_input.enter_rc_pos_control && control_mode != Control_Mode::RC_CONTROL)
    {
        rc_input.enter_rc_pos_control = false;
        // odom失效，拒绝进入
        if (!uav_state.odom_valid)
        {
            this->text_info.MessageType = sunray_msgs::TextInfo::ERROR;
            this->text_info.Message = "Reject RC_CONTROL. Odom invalid!";
            cout << RED << node_name << " Reject RC_CONTROL. Odom invalid! " << TAIL << endl;
            return;
        }
        // 切换至RC_POS_CONTROL
        control_mode = Control_Mode::RC_CONTROL;
        check_off = true;
        // 初始化默认的UAVCommand
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        // 进入RC_POS_CONTROL，需设置初始悬停点
        set_hover_pose_with_odom();
        cout << GREEN << node_name << " Switch to RC_CONTROL" << TAIL << endl;
        return;
    }

    // 必须从RC_POS_CONTROL模式切入（确保odom和offboard模式正常）

    if (rc_input.enter_command_control && control_mode == Control_Mode::RC_CONTROL && uav_state.mode == "OFFBOARD")
    {
        // 标志位重置
        rc_input.enter_command_control = false;
        // 切换至COMMAND_CONTROL
        control_mode = Control_Mode::CMD_CONTROL;
        check_off = true;
        // 初始化默认的UAVCommand
        control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
        cout << GREEN << node_name << " Switch to CMD_CONTROL" << TAIL << endl;
        return;
    }
}

void UAVControl::uav_setup_cb(const sunray_msgs::UAVSetup::ConstPtr &msg)
{
    if (msg->cmd == sunray_msgs::UAVSetup::ARMING)
    {
        arm_disarm_func(msg->arming);
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::SET_PX4_MODE)
    {
        set_px4_mode_func(msg->px4_mode);
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::REBOOT_PX4)
    {
        reboot_PX4();
    }
    else if (msg->cmd == sunray_msgs::UAVSetup::SET_CONTROL_MODE)
    {
        // todo more test
        if (msg->control_state == "CMD_CONTROL" && uav_state.armed)
        {
            cout << GREEN << node_name << " Switch to CMD_CONTROL by uav_setup cmd" << TAIL << endl;
            control_mode = Control_Mode::CMD_CONTROL;
            check_off = true;
        }
    }
}

void UAVControl::px4_pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    px4_pos_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    px4_vel_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    px4_acc_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);
    px4_yaw_target = msg->yaw;
}

// 发送位置期望值至飞控（输入: 期望xyz,期望yaw）
void UAVControl::send_local_pos_setpoint(const Eigen::Vector3d &pos_sp, double yaw_sp, bool enable_rate)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                             mavros_msgs::PositionTarget::IGNORE_VY |
                             mavros_msgs::PositionTarget::IGNORE_VZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ;
    if(enable_rate){
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW;
        pos_setpoint.yaw_rate = yaw_sp;
    }
    else{
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_setpoint.yaw = yaw_sp;
    };
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    setpoint_raw_local_pub.publish(pos_setpoint);
}

// 发送速度期望值至飞控（输入: 期望vxvyvz,期望yaw）
void UAVControl::send_local_vel_setpoint(const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                             mavros_msgs::PositionTarget::IGNORE_PY |
                             mavros_msgs::PositionTarget::IGNORE_PZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ;
    if(enable_rate){
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW;
        pos_setpoint.yaw_rate = yaw_sp;
    }
    else{
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_setpoint.yaw = yaw_sp;
    };
    pos_setpoint.position.x = 0.0;
    pos_setpoint.position.y = 0.0;
    pos_setpoint.position.z = 0.0;
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.acceleration_or_force.x = 0.0;
    pos_setpoint.acceleration_or_force.y = 0.0;
    pos_setpoint.acceleration_or_force.z = 0.0;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAVControl::send_vel_xy_pos_z_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate)
{
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.header.stamp = ros::Time::now();
    // 此处由于飞控暂不支持位置－速度追踪的复合模式，因此type_mask设定如下
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                             mavros_msgs::PositionTarget::IGNORE_PY |
                             mavros_msgs::PositionTarget::IGNORE_VZ |
                             mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ;
    if(enable_rate){
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW;
        pos_setpoint.yaw_rate = yaw_sp;
    }
    else{
        pos_setpoint.type_mask = pos_setpoint.type_mask|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        pos_setpoint.yaw = yaw_sp;
    };
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = 0.0;
    pos_setpoint.position.z = pos_sp[2];
    setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAVControl::send_pos_vel_xyz_setpoint(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, float yaw_sp, bool enable_rate)
{
    mavros_msgs::PositionTarget pos_setpoint;
    // 速度作为前馈项， 参见FlightTaskOffboard.cpp
    // 控制方法请见 PositionControl.cpp
    // 0b100111000000;  100 111 000 000  vx vy　vz x y z+ yaw
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                             mavros_msgs::PositionTarget::IGNORE_AFY |
                             mavros_msgs::PositionTarget::IGNORE_AFZ |
                             mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.yaw = desired_state.yaw;
    setpoint_raw_local_pub.publish(pos_setpoint);
}


// 发送角度期望值至飞控（输入: 期望角度-四元数,期望推力）
void UAVControl::send_attitude_setpoint(const Eigen::Vector3d &att_sp, double thrust_sp)
{
    sunray_msgs::AttitudeSetpoint att_setpoint;
    att_setpoint.header.stamp = ros::Time::now();
    att_setpoint.type_mask = sunray_msgs::AttitudeSetpoint::IGNORE_ROLL_RATE |
                             sunray_msgs::AttitudeSetpoint::IGNORE_PITCH_RATE |
                             sunray_msgs::AttitudeSetpoint::IGNORE_YAW_RATE;
    Eigen::Quaterniond q_sp = quaternion_from_rpy(att_sp);
    att_setpoint.orientation.x = q_sp.x();
    att_setpoint.orientation.y = q_sp.y();
    att_setpoint.orientation.z = q_sp.z();
    att_setpoint.orientation.w = q_sp.w();
    att_setpoint.body_rate.x = 0.0;
    att_setpoint.body_rate.y = 0.0;
    att_setpoint.body_rate.z = 0.0;
    att_setpoint.thrust = thrust_sp;
    setpoint_raw_attitude_pub.publish(att_setpoint);
}

// 发送经纬度以及高度期望值至飞控(输入,期望lat/lon/alt,期望yaw)
void UAVControl::send_global_pos_setpoint(const Eigen::Vector3d &global_pos_sp, float yaw_sp)
{
   sunray_msgs::GlobalPositionSetpoint global_setpoint;
    global_setpoint.coordinate_frame = sunray_msgs::GlobalPositionSetpoint::FRAME_GLOBAL_INT;
    global_setpoint.type_mask = sunray_msgs::GlobalPositionSetpoint::IGNORE_VX |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_VY |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_VZ |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_AFX |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_AFY |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_AFZ |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_AFY |
                                sunray_msgs::GlobalPositionSetpoint::IGNORE_YAW_RATE;
    global_setpoint.latitude = global_pos_sp[0];
    global_setpoint.longitude = global_pos_sp[1];
    global_setpoint.altitude = global_pos_sp[2];
    global_setpoint.yaw = yaw_sp;
    setpoint_raw_global_pub.publish(global_setpoint);
}


/***
 * 上锁解锁函数，调用mavros上锁和解决服务
 * 参数: bool on_or_off，true为解锁指令，false为上锁指令
 * 判断当前无人机状态，为解锁状态且on_or_off为
 * -----------------------------------------------------------------------|
 * |on_or_off/armed    |     true(无人机已解锁)      |   false（无人机未解锁） |
 * -----------------------------------------------------------------------|
 * |true（解锁指令）     |     无人机已经解锁           | 无人机正在解锁，解锁成功|
 * ----------------------------------------------------------------------|
 * |false（上锁指令）    |     无人机正在上锁，上锁成功  |   无人机已经上锁        |
 * -----------------------------------------------------------------------|
 */
void UAVControl::arm_disarm_func(bool on_or_off)
{
    mavros_msgs::CommandBool arm_cmd;

    if (uav_state.armed)
    {
        if (!on_or_off)
        {
            arm_cmd.request.value = on_or_off;
            px4_arming_client.call(arm_cmd);
            if (arm_cmd.response.success)
            {
                cout << GREEN << node_name << " vehicle disarming, success!" << TAIL << endl;
            }
            else
            {
                this->text_info.MessageType = sunray_msgs::TextInfo::ERROR;
                this->text_info.Message = "vehicle disarming, fail!";
                cout << RED << node_name << " vehicle disarming, fail!" << TAIL << endl;
            }
        }
        else
        {
            this->text_info.MessageType = sunray_msgs::TextInfo::WARN;
            this->text_info.Message = "vehicle already armed";
            cout << YELLOW << node_name << " vehicle already armed" << TAIL << endl;
        }
    }
    else if (on_or_off)
    {
        arm_cmd.request.value = on_or_off;
        px4_arming_client.call(arm_cmd);
        if (arm_cmd.response.success)
        {
            cout << GREEN << node_name << "vehicle arming, success!" << TAIL << endl;
        }
        else
        {
            this->text_info.MessageType = sunray_msgs::TextInfo::ERROR;
            this->text_info.Message = "vehicle arming, fail!";
            cout << RED << node_name << "vehicle arming, fail!" << TAIL << endl;
        }
    }
    else
    {
        this->text_info.MessageType = sunray_msgs::TextInfo::WARN;
        this->text_info.Message = "vehicle already disarmed";
        cout << YELLOW << node_name << "vehicle already disarmed" << TAIL << endl;
    }
}

void UAVControl::set_px4_mode_func(string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}

void UAVControl::enable_emergency_func()
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
    cout << RED << node_name << " send kill cmd: force disarmed!" << TAIL << endl;
    this->text_info.MessageType = sunray_msgs::TextInfo::ERROR;
    this->text_info.Message = "send kill cmd: force disarmed!";
}

void UAVControl::reboot_PX4()
{
    // https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
    mavros_msgs::CommandLong reboot_srv;
    reboot_srv.request.broadcast = false;
    reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    reboot_srv.request.param1 = 1;    // Reboot autopilot
    reboot_srv.request.param2 = 0;    // Do nothing for onboard computer
    reboot_srv.request.confirmation = true;
    px4_reboot_client.call(reboot_srv);
    cout << GREEN << node_name << " Reboot PX4!" << TAIL << endl;
    this->text_info.MessageType = sunray_msgs::TextInfo::WARN;
    this->text_info.Message = "Reboot PX4!";
}

// 打印参数
void UAVControl::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>> UAV control Param <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << endl;
    cout << GREEN << "sim_mode                  : " << sim_mode << " " << TAIL << endl;
    cout << GREEN << "Takeoff_height            : " << Takeoff_height << " [m] " << TAIL << endl;
    cout << GREEN << "Disarm_height             : " << Disarm_height << " [m] " << TAIL << endl;
    cout << GREEN << "Land_speed                : " << Land_speed << " [m/s] " << TAIL << endl;
    cout << GREEN << "geo_fence_x : " << uav_geo_fence.x_min << " [m]  to  " << uav_geo_fence.x_min << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence_y : " << uav_geo_fence.y_min << " [m]  to  " << uav_geo_fence.y_max << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence_z : " << uav_geo_fence.z_min << " [m]  to  " << uav_geo_fence.z_max << " [m]" << TAIL << endl;
}