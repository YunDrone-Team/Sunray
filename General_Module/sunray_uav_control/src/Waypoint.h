#ifndef Waypoint_H
#define Waypoint_H

#include "ros_msg_utils.h"

class Waypoint
{
public:
    Waypoint(){};

    // 参数
    int uav_id;                         // 无人机ID
    std::string uav_name;               // 无人机名字
    
    Sunray_msgs::UAVWayPoint uav_wp;            // 无人机航点任务
    Sunray_msgs::UAVWayPointState uav_wp_state; // 无人机航点状态

    struct Waypoint_Params
    {
        bool wp_init = false;                         // 是否初始化
        bool wp_takeoff = false;                      // 是否起飞
        int wp_num = 0;                               // 航点数量 【最大数量10】
        int wp_type = 0;                              // 航点类型 【0：NED 1：经纬】
        int wp_end_type = 3;                          // 航点结束类型 【1: 悬停 2: 降落 3: 返航】
        int wp_yaw_type = 2;                          // 航点航向类型 【1: 固定值 2: 朝向下一个航点 3: 指向环绕点】
        int wp_index = 0;                             // 当前航点索引
        int wp_state = 0;                             // 航点状态 【1：解锁 2：起飞中 3：航点执行中 4:返航中 5:降落中 6: 结束】
        float wp_move_vel = 0.5;                      // 最大水平速度
        float z_height = 1.0;                         // 起飞和返航高度
        float wp_x_vel = 0.0;                         // 水平速度
        float wp_y_vel = 0.0;                         // 水平速度
        float wp_vel_p = 1;                           // 速度比例
        double wp_point_takeoff[3] = {0.0, 0.0, 0.0}; // 起飞点
        double wp_point_return[3] = {0.0, 0.0, 0.0};  // 返航点
        std::map<int, double[3]> wp_points;           // 航点
        ros::Time start_wp_time;                      // 上一个动作时间戳
    };

    Waypoint_Params wp_params;

    void init(ros::NodeHandle &nh);
    void mainLoop();
    void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg);
    float get_vel_from_waypoint(float point_x, float point_y);
    void show_state();
    void waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg);

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
void Waypoint::init(ros::NodeHandle &nh, int uav_id = 0, string uav_name = "uav")
{
    node_name = ros::this_node::getName();
    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名字前缀

    // 初始化参数
    string topic_prefix = "/" + uav_name + to_string(uav_id);               
    // 【订阅】无人机状态
    uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, &Waypoint::uav_state_callback, this);
      // 【订阅】无人机航点数据（配合sunray_msgs::UAVControlCMD::Waypoint子模式共同使用） -- 外部节点 -> 本节点
    uav_waypoint_sub = nh.subscribe<sunray_msgs::UAVWayPoint>(topic_prefix + "/sunray/uav_waypoint", 10, &Waypoint::waypoint_callback, this);

    // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
    // 【发布】无人机设置指令（本节点 -> sunray_control_node）
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

    // 变量初始化
    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
    uav_cmd.desired_pos[0] = 0.0;
    uav_cmd.desired_pos[1] = 0.0;
    uav_cmd.desired_pos[2] = 0.0;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_vel[2] = 0.0;
    uav_cmd.desired_acc[0] = 0.0;
    uav_cmd.desired_acc[1] = 0.0;
    uav_cmd.desired_acc[2] = 0.0;
    uav_cmd.desired_att[0] = 0.0;
    uav_cmd.desired_att[1] = 0.0;
    uav_cmd.desired_att[2] = 0.0;
    uav_cmd.desired_yaw = 0.0;
    uav_cmd.desired_yaw_rate = 0.0;


    uav_wp_state.header.stamp = ros::Time::now();
    uav_wp_state.wp_index = 0;
    uav_wp_state.wp_state = Sunray_msgs::UAVWayPointState::NOT_READY;






    
}

// 高级模式-航点模式的实现函数
void Waypoint::mainLoop()
{
    switch(uav_wp_state)
    {
        case Sunray_msgs::UAVWayPointState::NOT_READY:
            break;
        case Sunray_msgs::UAVWayPointState::READY:
            break;
        case Sunray_msgs::UAVWayPointState::TAKEOFF:
            break;
        case Sunray_msgs::UAVWayPointState::WAYPOINT:
            waypoint_mission();
            break;
        case Sunray_msgs::UAVWayPointState::RETURNING:
            break;
        case Sunray_msgs::UAVWayPointState::LANDING:
            break;
        case Sunray_msgs::UAVWayPointState::FINISH:
            break;
        default:
            break;
    }


    }

    // 进入航点模式先重置航点索引
    if (last_control_cmd.cmd != control_cmd.cmd)
    {
        Logger::warning("Waypoint mission started!");
        wp_params.wp_index = 0;
        wp_params.wp_state = 0;
        wp_params.start_wp_time = ros::Time::now();
        wp_params.wp_point_takeoff[0] = px4_state.position[0];
        wp_params.wp_point_takeoff[1] = px4_state.position[1];
    }

    // wp_state 航点状态 【1：解锁 2：起飞中 3：航点执行中 4:返航中 5:降落中 6: 结束】

    // 如果过程包含起飞过程，则先解锁起飞
    if (wp_params.wp_takeoff && (wp_params.wp_state == 0 || wp_params.wp_state == 1))
    {
        if(px4_state.mode == "OFFBOARD" && px4_state.armed)
        {
            wp_params.wp_state = 2;
        }
        // 判断是否是offboard模式
        if(px4_state.mode != "OFFBOARD" && wp_params.wp_state == 0)
        {
            set_offboard_control(Control_Mode::CMD_CONTROL);
            // set_offboard_control会重置状态 所以需要手动重新赋值为Waypoint
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Waypoint;
            wp_params.wp_state = 1;
        }
        // 判断是否已经解锁 未解锁则先解锁
        if (!px4_state.armed && wp_params.wp_state == 1)
        {
            setArm(true);
        }
        else
        {
            if ((ros::Time::now() - wp_params.start_wp_time).toSec() > 5)
            {
                // 解锁超时
                Logger::error("Takeoff timeout! Cannot start waypoint mission!");
                set_desired_from_hover();
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
        if ((abs(px4_state.position[0] - wp_params.wp_point_takeoff[0]) < 0.15) &&
            (abs(px4_state.position[1] - wp_params.wp_point_takeoff[1]) < 0.15) &&
            (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
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
            system_params.type_mask = TypeMask::XYZ_POS;
        }
        break;
    case 3:
        // 判断是否达到航点
        if ((abs(px4_state.position[0] - wp_params.wp_points[wp_params.wp_index][0]) < 0.15) &&
            (abs(px4_state.position[1] - wp_params.wp_points[wp_params.wp_index][1]) < 0.15) &&
            (abs(px4_state.position[2] - wp_params.wp_points[wp_params.wp_index][2]) < 0.15))
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
                // system_params.type_mask = TypeMask::XYZ_POS_YAW;
                system_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
            }
        }
        break;
    case 4:
        // 如果航点结束需要返航
        if (wp_params.wp_end_type == 3)
        {
            // 到达返航点后降落
            if ((abs(px4_state.position[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.position[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
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
                local_setpoint.yaw = get_yaw_from_waypoint(wp_params.wp_yaw_type,
                                                           flight_params.home_pos[0],
                                                           flight_params.home_pos[1]);
                system_params.type_mask = TypeMask::XY_VEL_Z_POS_YAW;
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
            if ((abs(px4_state.attitude[2] - flight_params.home_yaw) < 0.0872) &&
                (abs(px4_state.position[0] - flight_params.home_pos[0]) < 0.15) &&
                (abs(px4_state.position[1] - flight_params.home_pos[1]) < 0.15) &&
                (abs(px4_state.position[2] - wp_params.z_height) < 0.15))
            {
                Logger::warning("Mission completed! waiting for landing");
                control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
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
                system_params.type_mask = TypeMask::XYZ_POS_YAW;
            }
        }
        else if (wp_params.wp_end_type == 2)
        {
            // 原地降落
            Logger::warning("Mission completed! waiting for landing");
            control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
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

void Waypoint::waypoint_callback(const sunray_msgs::UAVWayPoint::ConstPtr &msg)
{
    uav_wp = *msg;

    // 读取航点，并赋值到结构体
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
    // 环绕点的值在最后一位获取
    wp_params.wp_points[10][0] = msg->wp_circle_point[0];
    wp_params.wp_points[10][1] = msg->wp_circle_point[1];

    wp_params.wp_init = true;
    Logger::warning("Waypoint setup success!");
}

// 无人机状态回调
void Waypoint::uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

// 计算航点需要的速度
float Waypoint::get_vel_from_waypoint(float point_x, float point_y)
{
    wp_params.wp_x_vel = (point_x - px4_state.position[0]) * wp_params.wp_vel_p;
    wp_params.wp_y_vel = (point_y - px4_state.position[1]) * wp_params.wp_vel_p;
    // 如果合速度大于最大速度，则重新计算为最大速度
    if ((wp_params.wp_x_vel * wp_params.wp_x_vel + wp_params.wp_y_vel * wp_params.wp_y_vel) > wp_params.wp_move_vel * wp_params.wp_move_vel)
    {
        wp_params.wp_x_vel = wp_params.wp_x_vel * wp_params.wp_move_vel / sqrt(wp_params.wp_x_vel * wp_params.wp_x_vel + wp_params.wp_y_vel * wp_params.wp_y_vel);
        wp_params.wp_y_vel = wp_params.wp_y_vel * wp_params.wp_move_vel / sqrt(wp_params.wp_x_vel * wp_params.wp_x_vel + wp_params.wp_y_vel * wp_params.wp_y_vel);
    }
}


#endif