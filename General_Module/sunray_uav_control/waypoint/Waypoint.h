#ifndef Waypoint_H
#define Waypoint_H

#include "ros_msg_utils.h"

#define ERR 0.1  

class Waypoint
{
public:
    Waypoint(){};

    // 参数
    int uav_id;                         // 无人机ID
    string uav_name;                    // 无人机名字
    string node_name;
    float max_vel;                      // 航点移动最大速度
    float kp_vel;

    bool mission_start{false};

    // 航点容器 航点从1开始计数
    std::map<int, sunray_msgs::Point> waypoint_vector;

    sunray_msgs::WayPoint uav_wp;            // 无人机航点任务
    sunray_msgs::WayPointState uav_wp_state; // 无人机航点状态
    sunray_msgs::UAVState uav_state;                      // 当前时刻无人机状态（本节点发布）
    sunray_msgs::UAVControlCMD control_cmd;               // 当前时刻无人机控制指令（来自任务节点）
    sunray_msgs::UAVSetup uav_setup;

    void init(ros::NodeHandle &nh);
    void mainLoop();
    void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg);
    void waypoint_callback(const sunray_msgs::WayPoint::ConstPtr &msg);
    int waypoint_mission();
    bool arrived_waypoint();
    float get_vel_from_waypoint(float point_x, float point_y);
    float get_yaw_from_waypoint(int type, float point_x, float point_y);
    void show_state();
    void uav_init();
    
private:
    // ROS话题订阅句柄
    ros::Subscriber uav_waypoint_sub;
    ros::Subscriber uav_state_sub;
    ros::Subscriber vel_sub;

    // ROS话题发布句柄
    ros::Publisher control_cmd_pub;    
    ros::Publisher uav_setup_pub;  

    // 定时器句柄
    ros::Timer timer_send_external_pos; 
};

// 初始化函数
void Waypoint::init(ros::NodeHandle &nh)
{
    node_name = ros::this_node::getName();
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀
    nh.param<float>("max_vel", max_vel, 1.0);           // 【参数】航点移动最大速度
    nh.param<float>("kp_vel", kp_vel, 0.5);             // 【参数】速度比例

    // 初始化参数
    string topic_prefix = "/" + uav_name + to_string(uav_id);               
    // 【订阅】无人机状态
    uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, &Waypoint::uav_state_callback, this);
    // 【订阅】无人机航点数据
    uav_waypoint_sub = nh.subscribe<sunray_msgs::WayPoint>(topic_prefix + "/sunray/uav_waypoint", 10, &Waypoint::waypoint_callback, this);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 变量初始化
    control_cmd.header.stamp = ros::Time::now();
    control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    control_cmd.desired_pos[0] = 0.0;
    control_cmd.desired_pos[1] = 0.0;
    control_cmd.desired_pos[2] = 0.0;
    control_cmd.desired_vel[0] = 0.0;
    control_cmd.desired_vel[1] = 0.0;
    control_cmd.desired_vel[2] = 0.0;
    control_cmd.desired_acc[0] = 0.0;
    control_cmd.desired_acc[1] = 0.0;
    control_cmd.desired_acc[2] = 0.0;
    control_cmd.desired_att[0] = 0.0;
    control_cmd.desired_att[1] = 0.0;
    control_cmd.desired_att[2] = 0.0;
    control_cmd.desired_yaw = 0.0;
    control_cmd.desired_yaw_rate = 0.0;

    uav_wp_state.wp_index = 0;
    uav_wp_state.wp_num = 0;
    uav_wp_state.wp_state = sunray_msgs::WayPointState::NOT_READY;
}



// 高级模式-航点模式的实现函数
void Waypoint::mainLoop()
{
    // 没有接到任务开始的指令
    if(!mission_start)
    {
        return;
    }

    switch(uav_wp_state.wp_state)
    {
        // NOT_READY: 无人机未准备好
        case sunray_msgs::WayPointState::NOT_READY:
            
            uav_wp_state.wp_state = sunray_msgs::WayPointState::UAV_READY;
            break;
        // READY：无人机具备开始航点的客观条件(飞控已连接、已解锁、控制模式切换至CMD_CONTROL)
        case sunray_msgs::WayPointState::UAV_READY:
            // todo PANDUAN
            uav_init();
            uav_wp_state.wp_state = sunray_msgs::WayPointState::WAYPOINT;
            Logger::warning(node_name," ---> next point: ", waypoint_vector[uav_wp_state.wp_index].x, waypoint_vector[uav_wp_state.wp_index].y, waypoint_vector[uav_wp_state.wp_index].z);
            break;
        // WAYPOINT：执行航点任务中
        case sunray_msgs::WayPointState::WAYPOINT:
            static int mission_ = 0;
            mission_ = waypoint_mission();

            // 如果到达最后一个航点，判断是否需要返航, 不返航且需要降落则降落
            if(mission_ == 1)
            {
                uav_wp_state.wp_state = sunray_msgs::WayPointState::LAND;
            }
            break;
        // LAND：执行航点的降落任务中
        case sunray_msgs::WayPointState::LAND:

            control_cmd.header.stamp = ros::Time::now();
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            control_cmd_pub.publish(control_cmd);

            uav_wp_state.wp_state = sunray_msgs::WayPointState::NOT_READY;
            mission_start = false;

            break;
        default:
            break;
    }
}

bool Waypoint::arrived_waypoint()
{
    bool x,y,z;
    x = (abs(uav_state.position[0] - waypoint_vector[uav_wp_state.wp_index].x) < ERR);
    y = (abs(uav_state.position[1] - waypoint_vector[uav_wp_state.wp_index].y) < ERR);
    z = (abs(uav_state.position[2] - waypoint_vector[uav_wp_state.wp_index].z) < ERR);
    if (x && y && z)
    {
        return true;
    }else
    {
        return false;
    }
}

// 计算航点需要的速度
float Waypoint::get_vel_from_waypoint(float point_x, float point_y)
{
    // 根据目标点和当前位置作差计算前往目标点的期望速度
    uav_wp_state.velocity[0] = (point_x - uav_state.position[0]) * kp_vel;
    uav_wp_state.velocity[1] = (point_y - uav_state.position[1]) * kp_vel;

    float vel_norm = sqrt(uav_wp_state.velocity[0] * uav_wp_state.velocity[0] + uav_wp_state.velocity[1] * uav_wp_state.velocity[1]);
    // 如果合速度大于最大速度，则重新计算为最大速度
    if (vel_norm > uav_wp.wp_move_vel)
    {
        uav_wp_state.velocity[0] = uav_wp_state.velocity[0] * uav_wp.wp_move_vel / vel_norm;
        uav_wp_state.velocity[1] = uav_wp_state.velocity[1] * uav_wp.wp_move_vel / vel_norm;
    }
}

// 计算航点需要的yaw值
float Waypoint::get_yaw_from_waypoint(int type, float point_x, float point_y)
{
    // 固定航向
    if(type == 1)
    {
        return waypoint_vector[uav_wp_state.wp_index].yaw;
    }
    // 朝向下一个点
    else if (type == 2)
    {
        float yaw = atan2(point_y - uav_state.position[1],
                          point_x - uav_state.position[0]);
        // 如果航点距离较近，则不改变yaw值
        if ((abs(uav_state.position[0] - point_x) < 0.4) && (abs(uav_state.position[1] - point_y) < 0.4))
        {
            yaw = uav_state.attitude[2];
        }
        return yaw;
    }
}

int Waypoint::waypoint_mission()
{
    // 判断是否达到当前的航点
    bool arrived = arrived_waypoint();

    if(arrived)
    {
        uav_wp_state.wp_index++;
        
        if (uav_wp_state.wp_index > uav_wp_state.wp_num)
        {
            // 航点执行结束
            return 1;
        }

        // 下一个航点
        Logger::warning(node_name," ---> next point: ", waypoint_vector[uav_wp_state.wp_index].x, waypoint_vector[uav_wp_state.wp_index].y, waypoint_vector[uav_wp_state.wp_index].z);

    }else
    {
        get_vel_from_waypoint(waypoint_vector[uav_wp_state.wp_index].x, waypoint_vector[uav_wp_state.wp_index].y);
        
        // 发布无人机控制指令，移动模式为XyVelZPosYaw，XY控制速度，Z控制高度，偏航角固定
        control_cmd.header.stamp = ros::Time::now();
        control_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
        control_cmd.desired_vel[0] = uav_wp_state.velocity[0];
        control_cmd.desired_vel[1] = uav_wp_state.velocity[1];
        control_cmd.desired_pos[2] = waypoint_vector[uav_wp_state.wp_index].z;
        control_cmd.desired_yaw = get_yaw_from_waypoint(uav_wp.wp_yaw_type,
                                                    waypoint_vector[uav_wp_state.wp_index].x,
                                                    waypoint_vector[uav_wp_state.wp_index].y);
        control_cmd_pub.publish(control_cmd);
    }

    return 0;
}

void Waypoint::waypoint_callback(const sunray_msgs::WayPoint::ConstPtr &msg)
{
    Logger::warning("Get Waypoint!");

    if(msg->wp_points.size() != msg->wp_num)
    {
        Logger::warning("Wrong Waypoint Num!");
        return;
    }

    uav_wp = *msg;
    mission_start = uav_wp.start;
    uav_wp_state.wp_index = 1;
    uav_wp_state.wp_num = uav_wp.wp_points.size();

    Logger::print_color(int(LogColor::green), "Get ", (int)uav_wp_state.wp_num, " waypoints.");

    // 读取航点
    for(int i = 1; i <= uav_wp_state.wp_num; i++)
    {
        waypoint_vector[i] = uav_wp.wp_points[i-1];

        Logger::print_color(int(LogColor::green), "waypoint_", i, ": [",
                            waypoint_vector[i].x, ",",
                            waypoint_vector[i].y, ",",
                            waypoint_vector[i].z, "] ",
                            "[ m ],",
                            waypoint_vector[i].yaw/ M_PI * 180, "[deg].");
    }

    // 判断是否要加入返航点,如果需要的话将当前点作为返航点
    if(uav_wp.wp_end_type == 3)
    {
        uav_wp_state.wp_num = uav_wp_state.wp_num + 1;
        Logger::print_color(int(LogColor::green), "Add Return waypoints.");
        sunray_msgs::Point return_point;
        return_point.x = uav_state.position[0];
        return_point.y = uav_state.position[1];
        return_point.z = uav_wp.wp_points[uav_wp.wp_num-1].z;
        return_point.yaw = uav_state.attitude[2];
        waypoint_vector.insert(std::make_pair(uav_wp_state.wp_num,return_point));
    }

    Logger::warning("Read Waypoint!");
}

// 无人机状态回调
void Waypoint::uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void Waypoint::uav_init()
{
    // 切换到指令控制模式(同时，PX4模式将切换至OFFBOARD模式)
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
    {

        uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "CMD_CONTROL";
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": SET_CONTROL_MODE - [CMD_CONTROL]. ");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV control_mode set to [CMD_CONTROL] successfully!");

    // 解锁无人机
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 3 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 2 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 1 sec...");
    ros::Duration(1.0).sleep();
    while (ros::ok() && !uav_state.armed)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

    // 如果无人机没有起飞则起飞无人机
    if(uav_state.landed_state == sunray_msgs::PX4State::LANDED_STATE_ON_GROUND)
    {
        while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
        {
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            control_cmd_pub.publish(control_cmd);
            Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
            ros::Duration(4.0).sleep();
            ros::spinOnce();
        }    
    }

    Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");
}


void Waypoint::show_state()
{
    Logger::print_color(int(LogColor::white_bg_blue), ">>>>>>>>>>>>>>>> waypoint_mission - [", uav_name, "] <<<<<<<<<<<<<<<<<");

    switch (uav_wp_state.wp_state)
    {
        case sunray_msgs::WayPointState::NOT_READY:
            Logger::print_color(int(LogColor::green), "Mission_State: [ NOT_READY ]");
            break;
        case sunray_msgs::WayPointState::UAV_READY:
            Logger::print_color(int(LogColor::green), "Mission_State: [ UAV_READY ]");
            break;
        case sunray_msgs::WayPointState::WAYPOINT:
            Logger::print_color(int(LogColor::green), "Mission_State: [ WAYPOINT ]");
            Logger::print_color(int(LogColor::green), "Waypoint State [",
                                (int)uav_wp_state.wp_index,
                                " / ",
                                (int)uav_wp_state.wp_num,
                                "]");
            Logger::print_color(int(LogColor::green), "waypoint [X Y Z]:",
                                uav_wp_state.waypoint[0],
                                uav_wp_state.waypoint[1],
                                uav_wp_state.waypoint[2],
                                "[ m ]");
            Logger::print_color(int(LogColor::green), "velocity [X Y  ]:",
                                uav_wp_state.velocity[0],
                                uav_wp_state.velocity[1],
                                "[m/s]");
            Logger::print_color(int(LogColor::green), "waypoint yaw:",
                                uav_wp_state.yaw / M_PI * 180,
                                "[deg]");
            break;
        case sunray_msgs::WayPointState::LAND:
            Logger::print_color(int(LogColor::green), "Mission_State: [ LAND ]");
            break;
        default:
            Logger::print_color(int(LogColor::red), "Mission_State: [ UNKNOWN ]");
            break;
    }
}

#endif