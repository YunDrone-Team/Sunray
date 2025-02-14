#include "UAVControl.h"

void UAVControl::init(ros::NodeHandle &nh)
{
    float default_home_x, default_home_y, default_home_z;

    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称
    uav_ns = ros::this_node::getName();
    // 【参数】飞行相关参数
    nh.param<float>("flight_param/Takeoff_height", flight_params.takeoff_height, 1.0); // 【参数】默认起飞高度
    nh.param<float>("flight_param/Disarm_height", flight_params.disarm_height, 0.2);   // 【参数】降落时自动上锁高度
    nh.param<float>("flight_param/Land_speed", flight_params.land_speed, 0.2);         // 【参数】降落速度
    nh.param<float>("flight_param/land_end_time", flight_params.land_end_time, 1.0);   // 【参数】降落最后一阶段时间
    nh.param<float>("flight_param/land_end_speed", flight_params.land_end_speed, 0.3); // 【参数】降落最后一阶段速度
    nh.param<int>("flight_param/land_type", flight_params.land_type, 0);             // 【参数】降落类型 【0:到达指定高度后锁桨 1:使用px4 auto.land】
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
    nh.param<bool>("system_params/check_cmd_timeout", system_params.check_cmd_timeout, true);             // 【参数】是否检查命令超时
    nh.param<float>("system_params/cmd_timeout", system_params.cmd_timeout, 2.0);                         // 【参数】命令超时时间
    nh.param<float>("system_params/odom_valid_timeout", system_params.odom_valid_timeout, 0.5);           // 【参数】定位超时降落时间
    nh.param<float>("system_params/odom_valid_warming_time", system_params.odom_valid_warming_time, 0.3); // 【参数】定位超时警告时间
    nh.param<bool>("system_params/use_rc_control", system_params.use_rc, true);                           // 【参数】是否使用遥控器控制

    uav_prefix = uav_name + std::to_string(uav_id);
    topic_prefix = "/" + uav_prefix;

    px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state",
                                                     10, &UAVControl::px4_state_callback, this);
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery",
                                                              10, &UAVControl::px4_battery_callback, this);
    // px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/mavros/local_position/odom",
    //                                                 10, &UAVControl::px4_odom_callback, this);
    px4_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/local_position/pose",
                                                                 10, &UAVControl::px4_local_pos_callback, this);
    // px4_global_pos_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/mavros/global_position/local",
    //                                                       10, &UAVControl::px4_global_pos_callback, this);
    px4_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_prefix + "/mavros/local_position/velocity_local",
                                                            10, &UAVControl::px4_vel_callback, this);
    px4_att_sub = nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 1,
                                                 &UAVControl::px4_att_callback, this);
    px4_pos_target_sub =
        nh.subscribe<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/target_local",
                                                  1,
                                                  &UAVControl::px4_pos_target_callback, this);
    // px4_att_target_sub =
    //     nh.subscribe<mavros_msgs::AttitudeTarget>(topic_prefix + "/mavros/setpoint_raw/target_attitude",
    //                                               1,
    //                                               &UAVControl::px4_att_target_callback, this);
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd",
                                                               10, &UAVControl::control_cmd_callback, this);
    setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup",
                                                    1, &UAVControl::setup_callback, this);
    odom_state_sub = nh.subscribe<sunray_msgs::ExternalOdom>(topic_prefix + "/sunray/odom_state", 10,
                                                             &UAVControl::odom_state_callback, this);
    rc_state_sub = nh.subscribe<sunray_msgs::RcState>(topic_prefix + "/sunray/rc_state", 1,
                                                      &UAVControl::rc_state_callback, this);
    uav_waypoint_sub = nh.subscribe<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 1,
                                                              &UAVControl::waypoint_callback, this);

    px4_setpoint_local_pub = nh.advertise<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/local", 1);
    px4_setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(topic_prefix + "/mavros/setpoint_raw/global", 1);
    px4_setpoint_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>(topic_prefix + "/mavros/setpoint_raw/attitude", 1);
    uav_state_pub = nh.advertise<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 1);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_" + std::to_string(uav_id), 1);

    // 【服务】解锁/上锁 -- 本节点->飞控
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(topic_prefix + "/mavros/cmd/arming");
    // 【服务】修改系统模式 -- 本节点->飞控
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(topic_prefix + "/mavros/set_mode");
    // 【服务】紧急上锁服务(KILL) -- 本节点->飞控
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");
    // 【服务】重启PX4飞控 -- 本节点->飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");

    print_timer = nh.createTimer(ros::Duration(1), &UAVControl::print_state, this);
    task_timer = nh.createTimer(ros::Duration(0.05), &UAVControl::task_timer_callback, this);

    // 初始化各个部分参数
    system_params.control_mode = Control_Mode::INIT;
    system_params.last_control_mode = Control_Mode::INIT;
    system_params.type_mask = 0;
    system_params.safety_state = -1;
    system_params.odom_valid_time = ros::Time(0);
    rcState_cb = false;
    allow_lock = false;
    flight_params.home_pos[0] = default_home_x;
    flight_params.home_pos[1] = default_home_y;
    flight_params.home_pos[2] = default_home_z;

    // 绑定高级模式对应的实现函数
    advancedModeFuncMap[Takeoff] = std::bind(&UAVControl::set_takeoff, this);
    advancedModeFuncMap[Land] = std::bind(&UAVControl::set_land, this);
    advancedModeFuncMap[Hover] = std::bind(&UAVControl::set_desired_from_hover, this);
    advancedModeFuncMap[Waypoint] = std::bind(&UAVControl::waypoint_mission, this);
    advancedModeFuncMap[Return] = std::bind(&UAVControl::return_to_home, this);
}

// 检查当前模式 并进入对应的处理函数中
void UAVControl::mainLoop()
{
    switch (system_params.control_mode)
    {
        // 检查无人机是否位于定点模式，否则切换至定点模式safety_state
    case Control_Mode::INIT:
        if (uav_state.mode != "POSCTL")
        {
            setMode("POSCTL");
        }
        break;

    case Control_Mode::RC_CONTROL:
        set_desired_from_rc();
        break;

    case Control_Mode::CMD_CONTROL:
        set_desired_from_cmd();
        break;

    case Control_Mode::LAND_CONTROL:
        set_desired_from_land();
        break;

    case Control_Mode::WITHOUT_CONTROL:
        break;
    default:
        set_desired_from_hover();
        break;
    }
    system_params.last_control_mode = system_params.control_mode;
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "UAVControl");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    UAVControl mavctrl;
    mavctrl.init(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        mavctrl.mainLoop();
    }
}