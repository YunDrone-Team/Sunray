#include "UAVControl.h"

void UAVControl::Init(ros::NodeHandle &nh) {

    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号

    // 无人机名字 = 无人机名字前缀 + 无人机ID
    std::string uav_name = "/uav_name" + std::to_string(uav_id);

    uav_state_.uav_id = uav_id; //???

    // 【参数】飞行相关参数
    nh.param<float>("flight_params/Takeoff_height", flight_params.takeoff_height, 1.0);   // 【参数】默认起飞高度

    nh.param<int>("flight_params/land_type", flight_params.land_type, 0);                 // 【参数】降落类型 【0:使用自定义降落 1:使用px4 auto.land】
    nh.param<float>("flight_params/Land_speed", flight_params.land_speed, 0.2);           // 【参数】降落速度
    nh.param<float>("flight_params/land_end_time", flight_params.land_end_time, 1.0);     // 【参数】降落最后一阶段时间
    nh.param<float>("flight_params/land_end_speed", flight_params.land_end_speed, 0.3);   // 【参数】降落最后一阶段速度
    nh.param<float>("flight_params/land_kp", flight_params.land_kp, 1.0);                 // 【参数】降落XY位置控制增益
    nh.param<float>("flight_params/land_max_vel_xy", flight_params.land_max_vel_xy, 0.5); // 【参数】降落XY最大速度限制

    nh.param<double>("flight_params/home_x", flight_params.home_pos[0], 0.0);             // 【参数】默认home点 
    nh.param<double>("flight_params/home_y", flight_params.home_pos[1], 0.0);             // 【参数】默认home点 
    nh.param<double>("flight_params/home_z", flight_params.home_pos[2], 0.0);             // 【参数】默认home点 

    // 【参数】无人机地理围栏 - 超出围栏后无人机自动降落
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0); // 【参数】地理围栏最小x坐标
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);  // 【参数】地理围栏最大x坐标
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0); // 【参数】地理围栏最小y坐标
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);  // 【参数】地理围栏最大y坐标
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);  // 【参数】地理围栏最小z坐标
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);   // 【参数】地理围栏最大z坐标

    // 【参数】无人机系统参数
    nh.param<bool>("system_params/check_cmd_timeout", system_params.check_cmd_timeout, false); // 【参数】是否检查无人机控制指令超时
    nh.param<float>("system_params/cmd_timeout", system_params.cmd_timeout, 2.0);              // 【参数】无人机控制指令超时阈值


    // 【订阅】无人机控制指令 - 外部节点 -> 本节点
    control_cmd_sub_ = nh.subscribe<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 10, &UAVControl::ControlCmdCallback, this);
    // 【订阅】无人机PX4模式 - 飞控 -> mavros -> 本节点
    px4_state_sub_ = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, &UAVControl::PX4StateCallback, this);
    // 【订阅】无人机PX4状态（是否降落） - 飞控 -> mavros -> 本节点
    px4_extended_state_sub_ = nh.subscribe<mavros_msgs::ExtendedState>(uav_name + "/mavros/extended_state", 10, &UAVControl::PX4ExtendedStateCallback, this);
    // 【订阅】无人机电池状态 - 飞控 -> mavros -> 本节点
    px4_battery_sub_ = nh.subscribe<sensor_msgs::BatteryState>(uav_name + "/mavros/battery", 10, &UAVControl::PX4BatteryCallback, this);
    // 【订阅】PX4中的无人机位置（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, &UAVControl::PX4PoseCallback, this);
    // 【订阅】PX4中的无人机速度（坐标系:ENU系） - 飞控 -> mavros -> 本节点
    px4_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, &UAVControl::PX4VelCallback, this);
    // 【订阅】PX4中无人机的位置/速度/加速度设定值 - 飞控 -> mavros -> 本节点 （用于检验控制指令是否被PX4执行）
    px4_pos_target_sub_ = nh.subscribe<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/target_local", 1, &UAVControl::PX4PosTargetCallback, this);
    // 【订阅】PX4中无人机的姿态设定值 - 飞控 -> mavros -> 本节点 （用于检验控制指令是否被PX4执行）
    px4_att_target_sub_ = nh.subscribe<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/target_attitude", 1, &UAVControl::PX4AttTargetCallback, this);

    // 【发布】无人机状态 -> 本节点 -> 其他控制&任务节点/地面站
    uav_state_pub_ = nh.advertise<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 1);
    // 【发布】PX4位置环控制指令（包括期望位置、速度、加速度等接口，坐标系:ENU系） - 本节点 -> mavros -> 飞控
    px4_setpoint_local_pub_ = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 1);
    // 【发布】PX4全局位置控制指令（包括期望经纬度等接口 坐标系:WGS84坐标系）- 本节点 -> mavros -> 飞控
    px4_setpoint_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(uav_name + "/mavros/setpoint_raw/global", 1);
    // 【发布】PX4姿态环控制指令（包括期望姿态等接口）- 本节点 -> mavros -> 飞控
    px4_setpoint_attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>(uav_name + "/mavros/setpoint_raw/attitude", 1);
    // 【服务】PX4解锁/上锁指令 -- 本节点 -> mavros -> 飞控
    px4_arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
    // 【服务】PX4修改飞行模式指令 -- 本节点 -> mavros -> 飞控
    px4_set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    // 【服务】PX4紧急上锁指令 -- 本节点 -> mavros -> 飞控
    px4_emergency_client_ = nh.serviceClient<mavros_msgs::CommandLong>(uav_name + "/mavros/cmd/command");

}

// 回调函数：PX4状态
void UAVControl::PX4StateCallback(const mavros_msgs::State::ConstPtr& msg) {

    //px4_state_time = ros::Time::now();
    uav_state_.connected = msg->connected;
    uav_state_.armed = msg->armed;
    uav_state_.mode = msg->mode;
}

// 回调函数：PX4中的无人机位置
void UAVControl::PX4PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    uav_state_.position[0] = msg->pose.position.x;
    uav_state_.position[1] = msg->pose.position.y;
    uav_state_.position[2] = msg->pose.position.z;

    uav_state_.attitude.x = msg->pose.orientation.x;
    uav_state_.attitude.y = msg->pose.orientation.y;
    uav_state_.attitude.z = msg->pose.orientation.z;
    uav_state_.attitude.w = msg->pose.orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    uav_state_.rpy[0] = roll;
    uav_state_.rpy[1] = pitch;
    uav_state_.rpy[2] = yaw;
}

// 回调函数：PX4中的无人机速度
void UAVControl::PX4VelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

    uav_state_.velocity[0] = msg->twist.linear.x;
    uav_state_.velocity[1] = msg->twist.linear.y;
    uav_state_.velocity[2] = msg->twist.linear.z;
}

// 回调函数：PX4降落状态
void UAVControl::PX4ExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {

    uav_state_.landed_state = msg->landed_state;
}

// 回调函数：PX4电池
void UAVControl::PX4BatteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {

    uav_state_.battery_state = msg->voltage;
    uav_state_.battery_percentage = msg->percentage * 100;
}

// 回调函数：接收PX4的姿态设定值
void UAVControl::PX4AttTargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {

    uav_state_.att_setpoint.x = msg->orientation.x;
    uav_state_.att_setpoint.y = msg->orientation.y;
    uav_state_.att_setpoint.z = msg->orientation.z;
    uav_state_.att_setpoint.w = msg->orientation.w;

    // 转为rpy
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    uav_state_.rpy_setpoint[0] = roll;
    uav_state_.rpy_setpoint[1] = pitch;
    uav_state_.rpy_setpoint[2] = yaw;

    // px4_rates_target = Eigen::Vector3d(msg->body_rate.x, msg->body_rate.y, msg->body_rate.z);
    uav_state_.thrust_setpoint = msg->thrust;
}

// 回调函数：接收PX4的位置设定值
void UAVControl::PX4PosTargetCallback(const mavros_msgs::PositionTarget::ConstPtr& msg) {

    uav_state_.pos_setpoint[0] = msg->position.x;
    uav_state_.pos_setpoint[1] = msg->position.y;
    uav_state_.pos_setpoint[2] = msg->position.z;
    uav_state_.vel_setpoint[0] = msg->velocity.x;
    uav_state_.vel_setpoint[1] = msg->velocity.y;
    uav_state_.vel_setpoint[2] = msg->velocity.z;
}

// 控制指令回调
void UAVControl::ControlCmdCallback(const sunray_msgs::UAVControlCMD::ConstPtr &msg) {


    uav_state_.control_mode = msg->control_mode;
    uav_state_.move_mode = msg->move_mode;

    uav_state_.desired_pos[0] = msg->desired_pos[0];
    uav_state_.desired_pos[1] = msg->desired_pos[1];
    uav_state_.desired_pos[2] = msg->desired_pos[2];


    uav_state_.desired_vel[0] = msg->desired_vel[0];
    uav_state_.desired_vel[1] = msg->desired_vel[1];
    uav_state_.desired_vel[2] = msg->desired_vel[2];




    uav_state_.desired_yaw = msg->desired_yaw;

}

    

// 安全检查 是否超出地理围栏 外部定位是否有效
int UAVControl::SafetyCheck() {


    //uav_state.odom_valid = px4_state.external_odom.odom_valid;

    // 如果外部定位失效，则返回2
    if (!uav_state.odom_valid)
    {
        return 2;
    }

    // GPS/RTK模式下，暂时不检查地理围栏
    if (uav_state.location_source == sunray_msgs::ExternalOdom::GPS ||
        uav_state.location_source == sunray_msgs::ExternalOdom::RTK)
    {
        // GPS模式下，可以根据需要添加基于经纬度的围栏检查
        return 0;
    }

    // 如果超出地理围栏，则返回1（仅对本地定位模式有效）
    if (px4_state.position[0] < uav_geo_fence.x_min ||
        px4_state.position[0] > uav_geo_fence.x_max ||
        px4_state.position[1] < uav_geo_fence.y_min ||
        px4_state.position[1] > uav_geo_fence.y_max ||
        px4_state.position[2] < uav_geo_fence.z_min ||
        px4_state.position[2] > uav_geo_fence.z_max)
    {
        return 1;
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

    }

    return 0;
}


// 注意：无人机在POSCTL模式下，如果飞控没有接入遥控器是不允许解锁的  应该是要调参数
// 检查当前设置的状态，并进入对应的处理函数中
void UAVControl::MainLoop() {

    // 安全检查 
    SafetyCheck();

    switch (uav_state_.control_mode) {

        case sunray_msgs::UAVControlCMD::Ready: 
            break;

        case sunray_msgs::UAVControlCMD::Takeoff:

            TakeoffCallback();
            break;

        case sunray_msgs::UAVControlCMD::Hover:

            HoverCallback();
            break;

        case sunray_msgs::UAVControlCMD::Move:

            MoveCallback();
            break;

        case sunray_msgs::UAVControlCMD::Land:

            LandCallback();
            break;

        case sunray_msgs::UAVControlCMD::Return:

            ReturnCallback();
            break;

        default:
            break;
    }

    PublishUAVState();
}

// TODO: 发布uav_state
void UAVControl::PublishUAVState() { 
    
    uav_state_.header.stamp = ros::Time::now();
    uav_state_pub_.publish(uav_state_);
}

void UAVControl::TakeoffCallback() {

    // 切换到 offboard 模式
// 注意：PX4从其他飞行模式进入OFFBOARD模式，需要发送期望指令才可进入，此处发送0指令
      // set_default_local_setpoint();
    // setpoint_local_pub(TypeMask::XYZ_VEL, local_setpoint);
    SetFlightMode("OFFBOARD");

    // TODO:需要判断是否属于新指令?


    // 1.如果还没有解锁，就解锁 
    if (!uav_state_->armed) {

        SetArm(true);
    }

    // 2.每一次解锁，将当前位置记录为home点
    if (uav_state_->armed && !flight_params.set_home) {

        flight_params.home_pos[0] = msg->position[0];
        flight_params.home_pos[1] = msg->position[1];
        flight_params.home_pos[2] = msg->position[2];
        flight_params.home_yaw = msg->attitude[2];
        flight_params.set_home = true;
    }

    // 如果已经起飞，则不执行
    if (uav_state.landed_state != sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
    {
        Logger::error("UAV already takeoff!");
        return;
    }

    // 3.调用MoveControl()起飞到指定高度
    if (uav_state_->armed && flight_params.set_home) {

        local_setpoint.position.x = flight_params.home_pos[0];
        local_setpoint.position.y = flight_params.home_pos[1];
        local_setpoint.position.z = flight_params.home_pos[2] + flight_params.takeoff_height;
        local_setpoint.yaw = flight_params.home_yaw;
        MoveControl(TypeMask::XYZ_POS_YAW, local_setpoint);
    }

    // 4. 如果到指定高度，就记录悬停点并转换到悬停状态
    SetHoverPos();

}

// PX4解锁/上锁
void UAVControl::SetArm(bool arm_flag)
{
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm_flag;
        px4_arming_client.call(arm_cmd);
        if (arm_flag)
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
        arm_cmd.request.value = arm_flag;
}

void UAVControl::EmergencyStop() {

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

    // TODO???
    system_params.control_mode = sunray_msgs::UAVControlCMD::Ready; // 紧急停止后，切换到初始化模式
}

void UAVControl::LandCallback() {

    // 计算降落的期望值
    void UAVControl::handle_land_control()
    {

        
        // 如果无人机已经上锁，代表已经降落结束，切换控制状态机为INIT模式
        if (!px4_state.armed)
        {
            system_params.control_mode = Control_Mode::INIT;
            flight_params.set_land_pos = false;
            Logger::warning("Landing finished!");
            return;
        }

        // 降落模式1：使用AUTO.LAND飞行模式进行降落
        // AUTO.LAND：PX4内部的飞行模式，无人机原地降落，降落的速度和逻辑由PX4内部的控制逻辑决定
        if (flight_params.land_type == 1)
        {
            if (px4_state.mode != "AUTO.LAND")
            {
                Logger::warning("Land in AUTO.LAND mode!");
                set_px4_flight_mode("AUTO.LAND");
            }
            return;
        }

        // 降落模式2：
        // 第一次进入时设置降落的XY位置和偏航角
        // 使用XY位置控制 + Z速度控制，持续下降直到检测到着陆状态
        if (!flight_params.set_land_pos)
        {
            Logger::warning("Set Land Position Done!");
            system_params.last_land_time = ros::Time(0);
            system_params.low_velocity_start_time = ros::Time(0);
            flight_params.land_pos[0] = px4_state.position[0];
            flight_params.land_pos[1] = px4_state.position[1];
            flight_params.land_yaw = px4_state.attitude[2];
            flight_params.set_land_pos = true;
        }

        // 着陆检测：PX4 landed_state 或 速度接近零持续1秒
        bool velocity_low = fabs(px4_state.velocity[0]) < 0.1 &&
                            fabs(px4_state.velocity[1]) < 0.1 &&
                            fabs(px4_state.velocity[2]) < 0.1;

        if (velocity_low)
        {
            if (system_params.low_velocity_start_time == ros::Time(0))
            {
                system_params.low_velocity_start_time = ros::Time::now();
            }
        }
        else
        {
            system_params.low_velocity_start_time = ros::Time(0);
        }

        bool landed_by_velocity = system_params.low_velocity_start_time != ros::Time(0) &&
                                (ros::Time::now() - system_params.low_velocity_start_time).toSec() > 1.0;

        // 检测到着陆状态，进入最终降落阶段
        if (px4_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND || landed_by_velocity)
        {
            if (system_params.last_land_time == ros::Time(0))
            {
                system_params.last_land_time = ros::Time::now();
                Logger::warning("Landed detected, stopping XY control");
            }
            // 着陆后继续下压land_end_time秒，防止误判后直接锁桨掉落
            set_default_local_setpoint();
            if ((ros::Time::now() - system_params.last_land_time).toSec() < flight_params.land_end_time)
            {
                local_setpoint.velocity.x = 0.0;
                local_setpoint.velocity.y = 0.0;
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
                return;
            }
        }
        else
        {
            system_params.last_land_time = ros::Time(0);
            set_default_local_setpoint();
            float vel_x = flight_params.land_kp * (flight_params.land_pos[0] - px4_state.position[0]);
            float vel_y = flight_params.land_kp * (flight_params.land_pos[1] - px4_state.position[1]);
            // 限幅
            vel_x = std::max(-flight_params.land_max_vel_xy, std::min(flight_params.land_max_vel_xy, vel_x));
            vel_y = std::max(-flight_params.land_max_vel_xy, std::min(flight_params.land_max_vel_xy, vel_y));

            local_setpoint.velocity.x = vel_x;
            local_setpoint.velocity.y = vel_y;
            local_setpoint.velocity.z = -flight_params.land_speed;
            local_setpoint.yaw = flight_params.land_yaw;
            system_params.type_mask = TypeMask::XYZ_VEL_YAW;
        }

        setpoint_local_pub(system_params.type_mask, local_setpoint);
    }

    // 特殊指令实现函数：进入降落模式
    void UAVControl::set_land()
    {
        // 当前模式不是降落模式，且要处于RC_CONTROL或CMD_CONTROL模式才会进入降落模式
        if (system_params.control_mode != Control_Mode::LAND_CONTROL && (system_params.control_mode == Control_Mode::RC_CONTROL || system_params.control_mode == Control_Mode::CMD_CONTROL))
        {
            // 重置降落状态，确保使用当前位置作为降落点
            flight_params.set_land_pos = false;
            system_params.control_mode = Control_Mode::LAND_CONTROL;
        }
    }


      // 无人机上锁后，重置set_home状态位
    if (flight_params.set_home && !msg->armed) {

        flight_params.set_home = false;
        Logger::warning("PX4 disarmed, flight_params.set_home [false]!");
    }

    // TODO:切换到定点模式和 ready 状态s
      
    SetFlightMode("POSCTL");

        //     system_params.control_mode = sunray_msgs::UAVControlCMD::Ready;
    // 无人机在未解锁状态且处于非定点模式时切换到定点模式


}

void UAVControl::ReturnCallback() {

    // 如果未设置home点，则无法进入返航模式
    if (!flight_params.set_home) {

        set_desired_from_hover();
        return;
    }

    // 设置
    local_setpoint.position.x = flight_params.home_pos[0];
    local_setpoint.position.y = flight_params.home_pos[1];
    local_setpoint.position.z = uav_state_.position[2];
    local_setpoint.yaw = flight_params.home_yaw;
    system_params.type_mask = ;

    MoveControl(TypeMask::XYZ_POS_YAW, local_setpoint);

    // 返航到home点上方后，执行降落指令

    // 达到home点上方后，且速度降低、偏航角到位后开始降落
    // 计算偏航角误差，处理角度环绕问题
    double yaw_error = px4_state.attitude[2] - flight_params.home_yaw;
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    if (fabs(px4_state.position[0] - flight_params.home_pos[0]) < 0.15 &&
        fabs(px4_state.position[1] - flight_params.home_pos[1]) < 0.15 &&
        fabs(px4_state.velocity[0]) < 0.1 &&
        fabs(px4_state.velocity[1]) < 0.1 &&
        fabs(px4_state.velocity[2]) < 0.1 &&
        fabs(yaw_error) < 0.2) {

            LandControl();
        return;
    }
}

void UAVControl::HoverCallback() {


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
        if (uav_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND) {

            local_setpoint.velocity.x = 0.0;
            local_setpoint.velocity.y = 0.0;
            local_setpoint.velocity.z = 0.0;
            local_setpoint.yaw = flight_params.hover_yaw;
            system_params.type_mask = TypeMask::XYZ_VEL_YAW;

        } else {
            // 已起飞，使用位置控制
            local_setpoint.position.x = flight_params.hover_pos[0];
            local_setpoint.position.y = flight_params.hover_pos[1];
            local_setpoint.position.z = flight_params.hover_pos[2];
            local_setpoint.yaw = flight_params.hover_yaw;
            system_params.type_mask = TypeMask::XYZ_POS_YAW;
        }
    }
}

void UAVControl::MoveCallback(uint16_t type_mask, mavros_msgs::PositionTarget local_setpoint) {

    // 惯性系下的控制将直接赋值
      
    // 清除过去数据
    // set_default_local_setpoint()

    local_setpoint.header.stamp = ros::Time::now();

    local_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_setpoint.type_mask = type_mask;
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
        
    px4_setpoint_local_pub.publish(local_setpoint);
}

// 设置悬停位置
void UAVControl::SetHoverPos() {

    flight_params.hover_pos[0] = uav_state_.position[0];
    flight_params.hover_pos[1] = uav_state_.position[1];
    flight_params.hover_pos[2] = uav_state_.position[2];
    flight_params.hover_yaw = uav_state_.rpy[2];
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
    global_setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT; // 相对高度: FRAME_GLOBAL_REL_ALT, 绝对高度: FRAME_GLOBAL_INT
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

// 设置PX4飞行模式
void UAVControl::SetFlightMode(std::string mode) {

    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    px4_set_mode_client.call(mode_cmd);
}


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

    // 当需要检查指令超时，即 system_params.check_cmd_timeout = true
    // 判断：1、是否是新的指令，只有不是新的指令才会超时；2、悬停和起飞模式下不进行超时判断
    // if (system_params.check_cmd_timeout && !new_cmd && control_cmd.cmd != sunray_msgs::UAVControlCMD::Hover && control_cmd.cmd != sunray_msgs::UAVControlCMD::Takeoff)
    // {
    // }

    //TODO:切换到定点模式???
    // if (!px4_state.armed && uav_state.mode != "POSCTL" && system_params.safety_state == 0)
    // {
    //   
    //     ros::Duration(1.0).sleep();
    // }








