#include "mavros_control.h"

void mavros_control::init(ros::NodeHandle &nh)
{

    nh.param<int>("uav_id", uav_id, 0);                 // 【参数】无人机编号
    nh.param<int>("rc_mode", rc_mode, 0);               // 【参数】无人机编号
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

    topic_prefix = "/" + uav_name + std::to_string(uav_id);

    px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state",
                                                     10, &mavros_control::uav_state_cb, this);
    px4_battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_prefix + "/mavros/battery",
                                                              10, &mavros_control::px4_battery_callback, this);
    px4_odom_sub = nh.subscribe<nav_msgs::Odometry>(topic_prefix + "/mavros/local_position/odom",
                                                    10, &mavros_control::px4_odom_callback, this);
    px4_target_sub = nh.subscribe<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/target_local",
                                                               1, &mavros_control::px4_pos_target_cb, this);
    

}


