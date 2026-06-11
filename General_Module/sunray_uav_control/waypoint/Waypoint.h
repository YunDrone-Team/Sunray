#ifndef Waypoint_H
#define Waypoint_H

#include "ros_msg_utils.h"
#include <algorithm>
#include <vector>

#define ERR 0.1  // 判断到达航点的误差

struct TrajectorySample
{
    Eigen::Vector3d pos{0.0, 0.0, 0.0};
    Eigen::Vector3d vel{0.0, 0.0, 0.0};
    double yaw{0.0};
    bool finished{false};
};

struct TrajectorySegment
{
    Eigen::Vector3d p0{0.0, 0.0, 0.0};
    Eigen::Vector3d p1{0.0, 0.0, 0.0};
    Eigen::Vector3d v0{0.0, 0.0, 0.0};
    Eigen::Vector3d v1{0.0, 0.0, 0.0};
    double yaw0{0.0};
    double yaw1{0.0};
    double start_time{0.0};
    double duration{1.0};
};

class Waypoint
{
public:
    Waypoint(){};

    // 参数
    int uav_id;                         // 无人机ID
    string uav_name;                    // 无人机名字
    string node_name;                   // 节点名字
    float max_vel;                      // 航点移动最大速度
    float kp_vel;                       // 速度比例
    float min_segment_time;             // 平滑轨迹单段最短时间
    float arrival_radius;               // 平滑轨迹结束判定半径
    string trajectory_mode;             // point: 旧航点模式, smooth: 平滑轨迹模式
    bool mission_start{false};
    bool trajectory_ready{false};
    ros::Time trajectory_start_time;
    std::vector<TrajectorySegment> trajectory_segments;
    double trajectory_total_time{0.0};

    // 航点容器 航点从1开始计数
    std::map<int, sunray_msgs::Point> waypoint_vector;

    sunray_msgs::WayPoint uav_wp;                       // 无人机航点任务
    sunray_msgs::WayPointState uav_wp_state;            // 无人机航点状态
    sunray_msgs::UAVState uav_state;                    // 无人机状态
    sunray_msgs::UAVControlCMD control_cmd;             // 无人机控制指令
    sunray_msgs::UAVSetup uav_setup;                    // 无人机设置指令

    void init(ros::NodeHandle &nh);
    void mainLoop();
    void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg);
    void waypoint_callback(const sunray_msgs::WayPoint::ConstPtr &msg);
    int waypoint_mission();
    int smooth_trajectory_mission();
    bool arrived_waypoint();
    void get_vel_from_waypoint(float point_x, float point_y);
    float get_yaw_from_waypoint(int type, float point_x, float point_y);
    bool build_smooth_trajectory();
    TrajectorySample sample_smooth_trajectory(double mission_time);
    double normalize_yaw(double yaw);
    double interpolate_yaw(double yaw0, double yaw1, double s);
    double get_smooth_cruise_vel();
    Eigen::Vector3d limit_velocity(const Eigen::Vector3d &vel, double limit);
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
    ros::Publisher uav_waypoint_state_pub;

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
    nh.param<std::string>("trajectory_mode", trajectory_mode, "smooth"); // 【参数】point: 逐点航点, smooth: 平滑轨迹
    nh.param<float>("min_segment_time", min_segment_time, 1.0);           // 【参数】平滑轨迹单段最短时间
    nh.param<float>("arrival_radius", arrival_radius, 0.25);              // 【参数】平滑轨迹结束判定半径

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
    // 【发布】无人机航点状态
    uav_waypoint_state_pub = nh.advertise<sunray_msgs::WayPointState>(topic_prefix + "/sunray/uav_waypoint_state", 1);


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

    if (trajectory_mode != "point" && trajectory_mode != "smooth")
    {
        Logger::warning(node_name, ": Unknown trajectory_mode [", trajectory_mode, "], fallback to smooth.");
        trajectory_mode = "smooth";
    }
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
            trajectory_ready = false;
            if (trajectory_mode == "smooth")
            {
                if (!build_smooth_trajectory())
                {
                    Logger::error(node_name, ": Failed to build smooth waypoint trajectory.");
                    uav_wp_state.wp_state = sunray_msgs::WayPointState::FINISH;
                    break;
                }
                trajectory_start_time = ros::Time::now();
                trajectory_ready = true;
            }
            uav_wp_state.wp_state = sunray_msgs::WayPointState::WAYPOINT;
            Logger::print_color(int(LogColor::green), node_name, ": Start waypoint mission.");
            Logger::warning(node_name," ---> First point: ", waypoint_vector[uav_wp_state.wp_index].x, waypoint_vector[uav_wp_state.wp_index].y, waypoint_vector[uav_wp_state.wp_index].z);
            break;
        // WAYPOINT：执行航点任务中
        case sunray_msgs::WayPointState::WAYPOINT:
            static int mission_ = 0;
            if (trajectory_mode == "smooth")
            {
                mission_ = smooth_trajectory_mission();
            }
            else
            {
                mission_ = waypoint_mission();
            }
            // 如果到达最后一个航点，降落
            if(mission_ == 1)
            {
                uav_wp_state.wp_state = sunray_msgs::WayPointState::FINISH;
                Logger::print_color(int(LogColor::green), node_name, ": Waypoint mission complete.");
            }
            break;
        // LAND：执行航点的降落任务中
        case sunray_msgs::WayPointState::FINISH:
            control_cmd.header.stamp = ros::Time::now();
            if(uav_wp.wp_end_type == 1)
            {
                // 发布悬停指令
                control_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
                
            }else
            {
                // 发布降落指令
                control_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            }
            control_cmd_pub.publish(control_cmd);
            uav_wp_state.wp_state = sunray_msgs::WayPointState::NOT_READY;
            mission_start = false;
            break;
        default:
            break;
    }

    uav_waypoint_state_pub.publish(uav_wp_state);
}


int Waypoint::waypoint_mission()
{
    // 判断是否达到当前的航点
    bool arrived = arrived_waypoint();
    
    // 计算当前与目标点的距离用于调试
    float dx = uav_state.position[0] - waypoint_vector[uav_wp_state.wp_index].x;
    float dy = uav_state.position[1] - waypoint_vector[uav_wp_state.wp_index].y;  
    float dz = uav_state.position[2] - waypoint_vector[uav_wp_state.wp_index].z;
    float distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    Logger::info(node_name, ": Current pos: [", uav_state.position[0], ", ", uav_state.position[1], ", ", uav_state.position[2], "]");
    Logger::info(node_name, ": Target pos: [", waypoint_vector[uav_wp_state.wp_index].x, ", ", waypoint_vector[uav_wp_state.wp_index].y, ", ", waypoint_vector[uav_wp_state.wp_index].z, "]");
    Logger::info(node_name, ": Distance to target: ", distance, " m, Arrived: ", arrived ? "YES" : "NO");

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
        uav_wp_state.waypoint[0] = waypoint_vector[uav_wp_state.wp_index].x;
        uav_wp_state.waypoint[1] = waypoint_vector[uav_wp_state.wp_index].y;
        uav_wp_state.waypoint[2] = waypoint_vector[uav_wp_state.wp_index].z;
        
        // 计算航点间的移动速度
        get_vel_from_waypoint(waypoint_vector[uav_wp_state.wp_index].x, waypoint_vector[uav_wp_state.wp_index].y);
        
        // 确保速度不为零（防止由于计算误差导致的悬停）
        float vel_norm = sqrt(uav_wp_state.velocity[0]*uav_wp_state.velocity[0] + uav_wp_state.velocity[1]*uav_wp_state.velocity[1]);
        if (vel_norm < 0.05) // 如果计算出的速度太小，使用最小速度
        {
            Logger::warning(node_name, ": Velocity too small, using minimum velocity");
            if (abs(dx) > ERR) {
                uav_wp_state.velocity[0] = (dx > 0) ? -0.1 : 0.1;
            }
            if (abs(dy) > ERR) {
                uav_wp_state.velocity[1] = (dy > 0) ? -0.1 : 0.1;
            }
        }
        
        // 发布无人机控制指令，移动模式为XyVelZPosYaw，XY控制速度，Z控制高度，偏航角固定
        control_cmd.header.stamp = ros::Time::now();
        // control_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYaw;
        control_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosVelYaw;

        control_cmd.desired_pos[0] = waypoint_vector[uav_wp_state.wp_index].x;
        control_cmd.desired_pos[1] = waypoint_vector[uav_wp_state.wp_index].y;
        control_cmd.desired_pos[2] = waypoint_vector[uav_wp_state.wp_index].z;

        control_cmd.desired_vel[0] = uav_wp_state.velocity[0];
        control_cmd.desired_vel[1] = uav_wp_state.velocity[1];
        control_cmd.desired_vel[2] = 0.0;
        
        // 根据偏航角类型计算航点间的偏航角
        uav_wp_state.yaw = get_yaw_from_waypoint(uav_wp.wp_yaw_type,
                                                    waypoint_vector[uav_wp_state.wp_index].x,
                                                    waypoint_vector[uav_wp_state.wp_index].y);
        control_cmd.desired_yaw = uav_wp_state.yaw;
        
        Logger::info(node_name, ": Publishing control cmd - vel: [", control_cmd.desired_vel[0], ", ", control_cmd.desired_vel[1], "], pos_z: ", control_cmd.desired_pos[2], ", yaw: ", control_cmd.desired_yaw);
        control_cmd_pub.publish(control_cmd);
    }

    return 0;
}

int Waypoint::smooth_trajectory_mission()
{
    if (!trajectory_ready || trajectory_segments.empty())
    {
        Logger::error(node_name, ": Smooth trajectory is not ready.");
        return 1;
    }

    double mission_time = (ros::Time::now() - trajectory_start_time).toSec();
    TrajectorySample sample = sample_smooth_trajectory(mission_time);

    control_cmd.header.stamp = ros::Time::now();
    control_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosVelYaw;
    control_cmd.desired_pos[0] = sample.pos.x();
    control_cmd.desired_pos[1] = sample.pos.y();
    control_cmd.desired_pos[2] = sample.pos.z();
    control_cmd.desired_vel[0] = sample.vel.x();
    control_cmd.desired_vel[1] = sample.vel.y();
    control_cmd.desired_vel[2] = sample.vel.z();
    control_cmd.desired_yaw = sample.yaw;
    control_cmd_pub.publish(control_cmd);

    uav_wp_state.waypoint[0] = sample.pos.x();
    uav_wp_state.waypoint[1] = sample.pos.y();
    uav_wp_state.waypoint[2] = sample.pos.z();
    uav_wp_state.velocity[0] = sample.vel.x();
    uav_wp_state.velocity[1] = sample.vel.y();
    uav_wp_state.yaw = sample.yaw;

    double segment_end_time = 0.0;
    for (size_t i = 0; i < trajectory_segments.size(); ++i)
    {
        segment_end_time = trajectory_segments[i].start_time + trajectory_segments[i].duration;
        if (mission_time <= segment_end_time)
        {
            uav_wp_state.wp_index = std::min<int>(i + 1, uav_wp_state.wp_num);
            break;
        }
    }

    if (sample.finished)
    {
        uav_wp_state.wp_index = uav_wp_state.wp_num;
        Eigen::Vector3d final_pos(
            waypoint_vector[uav_wp_state.wp_num].x,
            waypoint_vector[uav_wp_state.wp_num].y,
            waypoint_vector[uav_wp_state.wp_num].z);
        Eigen::Vector3d current_pos(uav_state.position[0], uav_state.position[1], uav_state.position[2]);
        double final_error = (current_pos - final_pos).norm();
        if (final_error < arrival_radius)
        {
            return 1;
        }
    }

    return 0;
}

void Waypoint::waypoint_callback(const sunray_msgs::WayPoint::ConstPtr &msg)
{
    // 正在任务中则拒收航点信息
    if(mission_start)
    {
        Logger::error("UAV already in a waypoint mission, send waypoint info later!");
        return;
    }

    Logger::warning("Get Waypoint!");

    if(msg->wp_points.size() != msg->wp_num)
    {
        Logger::warning("Wrong Waypoint Num!");
        return;
    }
    if(msg->wp_num == 0)
    {
        Logger::warning("Waypoint Num is zero!");
        return;
    }

    uav_wp = *msg;
    if (uav_wp.wp_move_vel <= 0.05)
    {
        Logger::warning("wp_move_vel is too small, fallback to max_vel: ", max_vel);
        uav_wp.wp_move_vel = max_vel;
    }
    mission_start = uav_wp.start;
    trajectory_ready = false;
    trajectory_segments.clear();
    waypoint_vector.clear();
    trajectory_total_time = 0.0;
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
        waypoint_vector[uav_wp_state.wp_num] = return_point;
    }

    Logger::warning("Read Waypoint!");
}

bool Waypoint::build_smooth_trajectory()
{
    trajectory_segments.clear();
    trajectory_total_time = 0.0;

    if (uav_wp_state.wp_num < 1)
    {
        Logger::error(node_name, ": No waypoint for smooth trajectory.");
        return false;
    }

    std::vector<Eigen::Vector3d> positions;
    std::vector<double> yaws;
    positions.reserve(uav_wp_state.wp_num + 1);
    yaws.reserve(uav_wp_state.wp_num + 1);

    positions.push_back(Eigen::Vector3d(uav_state.position[0], uav_state.position[1], uav_state.position[2]));
    yaws.push_back(uav_state.attitude[2]);
    for (int i = 1; i <= uav_wp_state.wp_num; ++i)
    {
        positions.push_back(Eigen::Vector3d(waypoint_vector[i].x, waypoint_vector[i].y, waypoint_vector[i].z));
        if (uav_wp.wp_yaw_type == 2)
        {
            Eigen::Vector3d direction = positions.back() - positions[positions.size() - 2];
            if (direction.head<2>().norm() > 0.05)
            {
                yaws.push_back(atan2(direction.y(), direction.x()));
            }
            else
            {
                yaws.push_back(yaws.back());
            }
        }
        else
        {
            yaws.push_back(waypoint_vector[i].yaw);
        }
    }

    const int point_num = positions.size();
    std::vector<double> durations(point_num - 1, min_segment_time);
    double cruise_vel = get_smooth_cruise_vel();
    double segment_min_time = std::max(0.1f, min_segment_time);

    for (int i = 0; i < point_num - 1; ++i)
    {
        double distance = (positions[i + 1] - positions[i]).norm();
        durations[i] = std::max(segment_min_time, distance / cruise_vel);
    }

    std::vector<Eigen::Vector3d> velocities(point_num, Eigen::Vector3d::Zero());
    for (int i = 1; i < point_num - 1; ++i)
    {
        // 中间航点使用前后两段平均速度，使轨迹一阶连续；起点和终点速度保持为 0。
        Eigen::Vector3d prev_vel = (positions[i] - positions[i - 1]) / durations[i - 1];
        Eigen::Vector3d next_vel = (positions[i + 1] - positions[i]) / durations[i];
        velocities[i] = 0.5 * (prev_vel + next_vel);
        if (velocities[i].norm() > cruise_vel)
        {
            velocities[i] = velocities[i].normalized() * cruise_vel;
        }
    }

    double start_time = 0.0;
    for (int i = 0; i < point_num - 1; ++i)
    {
        TrajectorySegment segment;
        segment.p0 = positions[i];
        segment.p1 = positions[i + 1];
        segment.v0 = velocities[i];
        segment.v1 = velocities[i + 1];
        segment.yaw0 = yaws[i];
        segment.yaw1 = yaws[i + 1];
        segment.start_time = start_time;
        segment.duration = durations[i];
        trajectory_segments.push_back(segment);
        start_time += durations[i];
    }

    trajectory_total_time = start_time;
    Logger::print_color(int(LogColor::green), node_name, ": Built smooth trajectory, segment_num=",
                        (int)trajectory_segments.size(), ", total_time=", trajectory_total_time, "s.");
    return true;
}

TrajectorySample Waypoint::sample_smooth_trajectory(double mission_time)
{
    TrajectorySample sample;
    if (trajectory_segments.empty())
    {
        sample.finished = true;
        return sample;
    }

    const TrajectorySegment *segment = &trajectory_segments.back();
    if (mission_time >= trajectory_total_time)
    {
        sample.pos = segment->p1;
        sample.vel = Eigen::Vector3d::Zero();
        sample.yaw = segment->yaw1;
        sample.finished = true;
        return sample;
    }

    for (size_t i = 0; i < trajectory_segments.size(); ++i)
    {
        double end_time = trajectory_segments[i].start_time + trajectory_segments[i].duration;
        if (mission_time <= end_time)
        {
            segment = &trajectory_segments[i];
            break;
        }
    }

    double local_t = std::max(0.0, mission_time - segment->start_time);
    double T = std::max(0.1, segment->duration);
    double s = std::min(1.0, std::max(0.0, local_t / T));
    double s2 = s * s;
    double s3 = s2 * s;

    double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    double h10 = s3 - 2.0 * s2 + s;
    double h01 = -2.0 * s3 + 3.0 * s2;
    double h11 = s3 - s2;
    // 三次 Hermite 曲线，直接输出 XyzPosVelYaw 需要的位置和速度前馈。
    sample.pos = h00 * segment->p0 + h10 * T * segment->v0 + h01 * segment->p1 + h11 * T * segment->v1;

    double dh00 = 6.0 * s2 - 6.0 * s;
    double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
    double dh01 = -6.0 * s2 + 6.0 * s;
    double dh11 = 3.0 * s2 - 2.0 * s;
    sample.vel = (dh00 * segment->p0 + dh10 * T * segment->v0 + dh01 * segment->p1 + dh11 * T * segment->v1) / T;
    sample.vel = limit_velocity(sample.vel, get_smooth_cruise_vel());

    sample.yaw = interpolate_yaw(segment->yaw0, segment->yaw1, s);
    sample.finished = false;
    return sample;
}

double Waypoint::normalize_yaw(double yaw)
{
    while (yaw > M_PI)
    {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI)
    {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

double Waypoint::interpolate_yaw(double yaw0, double yaw1, double s)
{
    double delta = normalize_yaw(yaw1 - yaw0);
    return normalize_yaw(yaw0 + delta * s);
}

double Waypoint::get_smooth_cruise_vel()
{
    double cruise_vel = uav_wp.wp_move_vel > 0.05 ? uav_wp.wp_move_vel : max_vel;
    if (max_vel > 0.05)
    {
        cruise_vel = std::min(cruise_vel, static_cast<double>(max_vel));
    }
    return std::max(0.05, cruise_vel);
}

Eigen::Vector3d Waypoint::limit_velocity(const Eigen::Vector3d &vel, double limit)
{
    if (limit <= 0.0 || vel.norm() <= limit)
    {
        return vel;
    }
    return vel.normalized() * limit;
}

// 无人机状态回调
void Waypoint::uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

void Waypoint::uav_init()
{
    // 开始执行航点时，无人机有几种可能性
    // 1、无人机在地上，未解锁
    // 2、无人机在地上，已解锁
    // 3、无人机在空中是CMD_CONTROL模式(PX4已切入OFFBOARD模式)
    // 4、无人机在空中自稳定or定点模式

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
void Waypoint::get_vel_from_waypoint(float point_x, float point_y)
{
    // 根据目标点和当前位置作差计算前往目标点的期望速度
    uav_wp_state.velocity[0] = (point_x - uav_state.position[0]) * kp_vel;
    uav_wp_state.velocity[1] = (point_y - uav_state.position[1]) * kp_vel;

    float vel_norm = sqrt(uav_wp_state.velocity[0] * uav_wp_state.velocity[0] + uav_wp_state.velocity[1] * uav_wp_state.velocity[1]);
    // 添加最小速度阈值，避免速度过小导致无人机不移动
    if (vel_norm < 0.1) 
    {
        Logger::warning(node_name, ": Calculated velocity too small: ", vel_norm, " m/s");
        return;
    }
    
    // 如果合速度大于最大速度，则重新计算为最大速度
    if (vel_norm > uav_wp.wp_move_vel)
    {
        uav_wp_state.velocity[0] = uav_wp_state.velocity[0] * uav_wp.wp_move_vel / vel_norm;
        uav_wp_state.velocity[1] = uav_wp_state.velocity[1] * uav_wp.wp_move_vel / vel_norm;
    }
    
    Logger::info(node_name, ": Target vel: [", uav_wp_state.velocity[0], ", ", uav_wp_state.velocity[1], "] m/s, norm: ", vel_norm);
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

    return uav_state.attitude[2];
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
        case sunray_msgs::WayPointState::FINISH:
            Logger::print_color(int(LogColor::green), "Mission_State: [ LAND ]");
            break;
        default:
            Logger::print_color(int(LogColor::red), "Mission_State: [ UNKNOWN ]");
            break;
    }
}

#endif
