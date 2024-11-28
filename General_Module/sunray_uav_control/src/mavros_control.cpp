#include "mavros_control.h"

void mavros_control::init(ros::NodeHandle &nh)
{

    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称
    uav_ns = ros::this_node::getName();
    nh.param<float>("uav_control/Takeoff_height", takeoff_height, 1.0); // 【参数】默认起飞高度
    nh.param<float>("uav_control/Disarm_height", disarm_height, 0.2);   // 【参数】降落时自动上锁高度
    nh.param<float>("uav_control/Land_speed", land_speed, 0.2);         // 【参数】降落速度
    // 【参数】地理围栏
    nh.param<float>("geo_fence/x_min", uav_geo_fence.x_min, -10.0);
    nh.param<float>("geo_fence/x_max", uav_geo_fence.x_max, 10.0);
    nh.param<float>("geo_fence/y_min", uav_geo_fence.y_min, -10.0);
    nh.param<float>("geo_fence/y_max", uav_geo_fence.y_max, 10.0);
    nh.param<float>("geo_fence/z_min", uav_geo_fence.z_min, -1.0);
    nh.param<float>("geo_fence/z_max", uav_geo_fence.z_max, 3.0);
    // 【参数】其他参数
    nh.param<float>("flight_param/land_end_time", land_end_time, 1.0);    // 【参数】降落最后一阶段时间
    nh.param<float>("land_end_time/land_end_speed", land_end_speed, 0.3); // 【参数】降落最后一阶段速度

    uav_prefix = uav_name + std::to_string(uav_id);
    topic_prefix = "/" + uav_prefix;

    px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state",
                                                     10, &mavros_control::px4_state_callback, this);
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery",
                                                              10, &mavros_control::px4_battery_callback, this);
    px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/mavros/local_position/odom",
                                                    10, &mavros_control::px4_odom_callback, this);
    px4_target_sub = nh.subscribe<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/target_local",
                                                               1, &mavros_control::px4_pos_target_callback, this);
    control_cmd_sub = nh.subscribe<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd",
                                                               10, &mavros_control::control_cmd_callback, this);
    setup_sub = nh.subscribe<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup",
                                                    1, &mavros_control::setup_callback, this);
    odom_state_sub = nh.subscribe<std_msgs::Bool>(topic_prefix + "/sunray/odom_state", 10,
                                                  &mavros_control::odom_state_callback, this);

    px4_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/local", 1);

    // 【服务】解锁/上锁 -- 本节点->飞控
    px4_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(topic_prefix + "/mavros/cmd/arming");
    // 【服务】修改系统模式 -- 本节点->飞控
    px4_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(topic_prefix + "/mavros/set_mode");
    // 【服务】紧急上锁服务(KILL) -- 本节点->飞控
    px4_emergency_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");
    // 【服务】重启PX4飞控 -- 本节点->飞控
    px4_reboot_client = nh.serviceClient<mavros_msgs::CommandLong>(topic_prefix + "/mavros/cmd/command");

    print_timer = nh.createTimer(ros::Duration(1), &mavros_control::print_state, this);

    control_mode = Control_Mode::INIT;
    last_control_mode = Control_Mode::INIT;

    safety_state = -1;
}
void mavros_control::mainLoop()
{
    switch (control_mode)
    {
        // 检查无人机是否位于定点模式，否则切换至定点模式
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
    last_control_mode = control_mode;
}

int main(int argc, char **argv)
{
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("/home/yundrone/Sunray/General_Module/sunray_uav_control/test/log.txt");

    ros::init(argc, argv, "mavros_control");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    mavros_control mavctrl;
    mavctrl.init(nh);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        mavctrl.mainLoop();
    }
}