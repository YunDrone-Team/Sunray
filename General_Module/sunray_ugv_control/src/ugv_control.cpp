#include "ugv_control.h"

void UGV_CONTROL::init(ros::NodeHandle &nh)
{
    // 【参数】编号
    nh.param<int>("ugv_id", ugv_id, 1);
    // 【参数】是否打印
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】设置获取数据源
    nh.param<int>("pose_source", pose_source, 2);
    // 【参数】悬停控制参数 - xy
    nh.param<float>("ugv_control_param/Kp_xy", ugv_control_param.Kp_xy, 1.4);
    // 【参数】悬停控制参数 - yaw
    nh.param<float>("ugv_control_param/Kp_yaw", ugv_control_param.Kp_yaw, 0.9);
    // 【参数】悬停控制参数 - max_vel_xy
    nh.param<float>("ugv_control_param/max_vel_xy", ugv_control_param.max_vel_xy, 0.5);
    // 【参数】悬停控制参数 - max_vel_yaw
    nh.param<float>("ugv_control_param/max_vel_yaw", ugv_control_param.max_vel_yaw, 50.0 / 180.0 * M_PI);
    // 【参数】地理围栏参数（超出围栏自动降落）
    nh.param<float>("ugv_geo_fence/max_x", ugv_geo_fence.max_x, 10.0);
    nh.param<float>("ugv_geo_fence/min_x", ugv_geo_fence.min_x, -10.0);
    nh.param<float>("ugv_geo_fence/max_y", ugv_geo_fence.max_y, 10.0);
    nh.param<float>("ugv_geo_fence/min_y", ugv_geo_fence.min_y, -10.0);
    // 地图分辨率
    nh.param<float>("resolution", resolution, 0.6);
    // 地图膨胀参数
    nh.param<float>("inflate", inflate, 0.2);

    // 计算地图网格数量
    int grid_w = static_cast<int>((ugv_geo_fence.max_x - ugv_geo_fence.min_x) / resolution) + 1;
    int grid_h = static_cast<int>((ugv_geo_fence.max_y - ugv_geo_fence.min_y) / resolution) + 1;
    // 初始化地图
    map_gen.init(nh, ugv_geo_fence.min_x, ugv_geo_fence.min_y, ugv_geo_fence.max_x, ugv_geo_fence.max_y, resolution, inflate);
    // 初始化A*算法
    astar.init(grid_w, grid_h);
    // 初始化变量
    goal_set = false;

    ugv_prefix = "ugv" + std::to_string(ugv_id);
    topic_prefix = "/" + ugv_prefix;
    // 根据 pose_source 参数选择数据源
    if (pose_source == 1)
    {
        // 【订阅】订阅动捕的数据(位置+速度) vrpn -> 本节点
        mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/" + ugv_prefix + "/pose", 1, &UGV_CONTROL::mocap_pos_cb, this);
        mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/" + ugv_prefix + "/twist", 1, &UGV_CONTROL::mocap_vel_cb, this);
        cout << GREEN << "Pose source: Mocap" << TAIL << endl;
    }
    else if (pose_source == 2)
    {
        // 【订阅】订阅gazebo Odom数据
        gazebo_odom_sub = nh.subscribe<nav_msgs::Odometry>("/car_odom", 1, &UGV_CONTROL::gazebo_odom_cb, this);
    }
    else
    {
        cout << RED << "Pose source: Unknown" << TAIL << endl;
    }

    // 【订阅】控制指令 外部控制节点 -> 本节点
    ugv_cmd_sub = nh.subscribe<sunray_msgs::UGVControlCMD>(topic_prefix + "/sunray_ugv/ugv_control_cmd", 10, &UGV_CONTROL::ugv_cmd_cb, this);
    // 【订阅】ugv电池的数据 ugv_driver -> 本节点
    battery_sub = nh.subscribe<std_msgs::Float32>(topic_prefix + "/sunray_ugv/battery", 1, &UGV_CONTROL::battery_cb, this);
    // 【订阅】目标点 move_base_simple -> 本节点
    planner_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &UGV_CONTROL::goal_point_cb, this);

    // 【发布】状态 本节点 -> 地面站/其他节点
    ugv_state_pub = nh.advertise<sunray_msgs::UGVState>(topic_prefix + "/sunray_ugv/ugv_state", 1);
    // 【发布】控制指令（机体系，单位：米/秒，Rad/秒）本节点 -> ugv_driver
    // ugv_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/sunray_ugv/" + ugv_prefix + "/cmd_vel", 1);
    ugv_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 【发布】无人车marker 本节点 -> RVIZ(仿真)
    ugv_mesh_pub = nh.advertise<visualization_msgs::Marker>(topic_prefix + "/sunray_ugv/mesh", 1);
    // 【发布】无人车运动轨迹  本节点 -> RVIZ(仿真)
    ugv_trajectory_pub = nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray_ugv/trajectory", 1);
    // 【发布】目标点marker 本节点 -> RVIZ(仿真)
    goal_point_pub = nh.advertise<visualization_msgs::Marker>(topic_prefix + "/sunray_ugv/goal_point_rviz", 1);
    // 【发布】速度方向 本节点 -> RVIZ(仿真)
    vel_rviz_pub = nh.advertise<geometry_msgs::TwistStamped>(topic_prefix + "/sunray_ugv/vel_rviz", 10);
    // 【发布】规划轨迹
    path_pub = nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray_ugv/plan_path", 1);

    // 【定时器】 定时发布ugv_state - 10Hz
    timer_state_pub = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_state, this);
    // 【定时器】 定时发布RVIZ显示相关话题(仿真) - 10Hz
    // timer_rivz = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_rviz, this);
    // 【定时器】 定时打印状态
    timer_debug = nh.createTimer(ros::Duration(3.0), &UGV_CONTROL::timercb_debug, this);
    // 【定时器】 定时更新A*算法
    timer_update_astar = nh.createTimer(ros::Duration(1), &UGV_CONTROL::timercb_update_astar, this);

    ugv_state.header.stamp = ros::Time::now();
    ugv_state.header.frame_id = "world";
    ugv_state.ugv_id = ugv_id;
    ugv_state.connected = false;
    ugv_state.odom_valid = false;
    ugv_state.position[0] = 0.0;
    ugv_state.position[1] = 0.0;
    ugv_state.velocity[0] = 0.0;
    ugv_state.velocity[1] = 0.0;
    ugv_state.yaw = 0.0;
    ugv_state.pos_setpoint[0] = 0.0;
    ugv_state.pos_setpoint[1] = 0.0;
    ugv_state.vel_setpoint[0] = 0.0;
    ugv_state.vel_setpoint[1] = 0.0;
    ugv_state.yaw_setpoint = 0.0;
    ugv_state.battery_state = -1.0;
    ugv_state.battery_percentage = -1.0;
    ugv_state.control_mode = sunray_msgs::UGVControlCMD::INIT;

    // 打印本节点参数，用于检查
    // printf_param();

    node_name = ros::this_node::getName();
    text_info.data = node_name + ugv_prefix + " init!";
    // text_info_pub.publish(text_info);
    cout << BLUE << text_info.data << TAIL << endl;
}

// 主循环函数
void UGV_CONTROL::mainloop()
{
    // 定位数据丢失情况下，不执行控制指令并直接返回，直到动捕恢复
    if (!ugv_state.odom_valid)
    {
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.linear.z = 0.0;
        desired_vel.angular.z = 0.0;
        ugv_cmd_vel_pub.publish(desired_vel);
        return;
    }

    // 每次进入主循环，先检查无人机是否超出地理围栏，超出的话则不发送任何指令并返回
    if (check_geo_fence())
    {
        return;
    }

    // 根据收到的控制指令进行相关计算，并生成对应的底层控制指令到ugv_cmd_vel_pub
    switch (current_ugv_cmd.cmd)
    {
    // INIT：不执行任何指令
    case sunray_msgs::UGVControlCMD::INIT:
        // 初始模式
        // do nothing
        break;

    // HOLD：悬停模式，切入该模式的瞬间，无人车在当前位置停止，即发送0速度
    case sunray_msgs::UGVControlCMD::HOLD:
        // 原地停止
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.linear.z = 0.0;
        desired_vel.angular.z = 0.0;
        ugv_cmd_vel_pub.publish(desired_vel);
        break;

    // POS_CONTROL：位置控制模式，无人车移动到期望的位置+偏航（期望位置由外部指令赋值）
    case sunray_msgs::UGVControlCMD::POS_CONTROL:
        // 位置控制算法
        // pos_control(current_ugv_cmd.desired_pos[0], current_ugv_cmd.desired_pos[1], current_ugv_cmd.desired_yaw);
        pos_control_diff(current_ugv_cmd.desired_pos[0], current_ugv_cmd.desired_pos[1], current_ugv_cmd.desired_yaw);
        break;

    // VEL_CONTROL_BODY：车体系速度控制，无人车按照期望的速度在车体系移动（期望速度由外部指令赋值）
    case sunray_msgs::UGVControlCMD::VEL_CONTROL_BODY:
        // 控制指令限幅（防止外部指令给了一个很大的数）
        desired_vel.linear.x = constrain_function(current_ugv_cmd.desired_vel[0], ugv_control_param.max_vel_xy, 0.0);
        desired_vel.linear.y = constrain_function(current_ugv_cmd.desired_vel[1], ugv_control_param.max_vel_xy, 0.0);
        desired_vel.angular.z = constrain_function(current_ugv_cmd.angular_vel, ugv_control_param.max_vel_yaw, 0.01);
        ugv_cmd_vel_pub.publish(desired_vel);
        break;

    // VEL_CONTROL_ENU：惯性系速度控制，无人车按照期望的速度在惯性系移动（期望速度由外部指令赋值）
    case sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU:
        // 由于UGV底层控制指令为车体系，所以需要将收到的惯性系速度转换为车体系速度
        desired_vel = enu_to_body(current_ugv_cmd.desired_vel[0], current_ugv_cmd.desired_vel[1]);
        ugv_cmd_vel_pub.publish(desired_vel);
        break;

    case sunray_msgs::UGVControlCMD::Point_Control_with_Astar:
        // A*算法
        path_control();
        break;

    default:
        break;
    }

    ugv_state_last = ugv_state;
}

void UGV_CONTROL::ugv_cmd_cb(const sunray_msgs::UGVControlCMD::ConstPtr &msg)
{
    current_ugv_cmd = *msg;

    switch (msg->cmd)
    {
    // 收到INIT指令
    case sunray_msgs::UGVControlCMD::INIT:
        text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: INIT!";
        cout << BLUE << text_info.data << TAIL << endl;
        break;
    // 收到HOLD指令
    case sunray_msgs::UGVControlCMD::HOLD:
        text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: HOLD!";
        cout << BLUE << text_info.data << TAIL << endl;
        break;
    // 收到POS_CONTROL指令
    case sunray_msgs::UGVControlCMD::POS_CONTROL:
        desired_position.x = msg->desired_pos[0];
        desired_position.y = msg->desired_pos[1];
        desired_yaw = msg->desired_yaw;
        //  text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: POS_CONTROL!";
        //  cout << BLUE << text_info.data << TAIL << endl;
        break;
    // 收到VEL_CONTROL_BODY指令：此处不做任何处理，在主循环中处理
    case sunray_msgs::UGVControlCMD::VEL_CONTROL_BODY:
        //  text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: VEL_CONTROL_BODY!";
        //  cout << BLUE << text_info.data << TAIL << endl;
        break;
    // 收到VEL_CONTROL_ENU指令：此处不做任何处理，在主循环中处理
    case sunray_msgs::UGVControlCMD::VEL_CONTROL_ENU:
        //  text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: VEL_CONTROL_ENU!";
        //  cout << BLUE << text_info.data << TAIL << endl;
        break;
    default:
        text_info.data = node_name + ": ugv_" + to_string(ugv_id) + " Get ugv_cmd: Wrong!";
        cout << RED << text_info.data << TAIL << endl;
        return;
        break;
    }
    // text_info_pub.publish(text_info);
}

// 惯性系->机体系
geometry_msgs::Twist UGV_CONTROL::enu_to_body(double vel_x, double vel_y)
{
    geometry_msgs::Twist body_cmd;
    // 惯性系 -> body frame
    float cmd_body[2];
    float cmd_enu[2];
    cmd_enu[0] = vel_x;
    cmd_enu[1] = vel_y;
    rotation_yaw(ugv_state.yaw, cmd_body, cmd_enu);
    body_cmd.linear.x = cmd_body[0];
    body_cmd.linear.y = cmd_body[1];
    body_cmd.linear.z = 0.0;
    body_cmd.angular.x = 0.0;
    body_cmd.angular.y = 0.0;
    // 控制指令计算：使用简易P控制 - YAW
    double yaw_error = get_yaw_error(current_ugv_cmd.desired_yaw, ugv_state.yaw);
    body_cmd.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    body_cmd.linear.x = constrain_function(body_cmd.linear.x, ugv_control_param.max_vel_xy, 0.0);
    body_cmd.linear.y = constrain_function(body_cmd.linear.y, ugv_control_param.max_vel_xy, 0.0);
    body_cmd.angular.z = constrain_function(body_cmd.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    return body_cmd;
}

double UGV_CONTROL::get_yaw_error(double yaw_ref, double yaw_now)
{
    double error = yaw_ref - yaw_now;

    if (error > M_PI)
    {
        error = error - 2 * M_PI;
    }
    else if (error < -M_PI)
    {
        error = error + 2 * M_PI;
    }

    return error;
}

// 位置控制算法
void UGV_CONTROL::pos_control(double x_ref, double y_ref, double yaw_ref)
{
    float cmd_body[2];
    float cmd_enu[2];
    // 控制指令计算：使用简易P控制 - XY
    cmd_enu[0] = (x_ref - ugv_state.position[0]) * ugv_control_param.Kp_xy;
    cmd_enu[1] = (y_ref - ugv_state.position[1]) * ugv_control_param.Kp_xy;
    // 惯性系 -> body frame
    rotation_yaw(ugv_state.yaw, cmd_body, cmd_enu);
    desired_vel.linear.x = cmd_body[0];
    desired_vel.linear.y = cmd_body[1];
    desired_vel.linear.z = 0.0;
    // YAW误差计算
    double yaw_error = get_yaw_error(yaw_ref, ugv_state.yaw);
    // 控制指令计算：使用简易P控制 - YAW
    desired_vel.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    desired_vel.linear.x = constrain_function(desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.linear.y = constrain_function(desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.angular.z = constrain_function(desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    // 发布控制指令
    ugv_cmd_vel_pub.publish(desired_vel);
}

// 位置控制，差速驱动
void UGV_CONTROL::pos_control_diff(double x_ref, double y_ref, double yaw_ref)
{
    // 如果已经到达目标点，则停止
    if (abs(x_ref - ugv_state.position[0]) < 0.1 && abs(y_ref - ugv_state.position[1]) < 0.1)
    {
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.angular.z = 0.0;
        ugv_cmd_vel_pub.publish(desired_vel);
        return;
    }
    // 计算目标点与当前点的差值
    double dx = x_ref - ugv_state.position[0];
    double dy = y_ref - ugv_state.position[1];
    // 计算目标点与当前点的角度
    double target_yaw = atan2(dy, dx);
    // 计算角度差
    double yaw_error = get_yaw_error(target_yaw, ugv_state.yaw);
    // 控制指令计算：使用简易P控制 - 差速控制只有X方向和角速率上的控制
    desired_vel.linear.x = ugv_control_param.Kp_xy * sqrt(dx * dx + dy * dy);
    desired_vel.linear.y = 0.0;
    desired_vel.linear.z = 0.0;
    // 控制指令计算：使用简易P控制 - YAW
    desired_vel.angular.z = yaw_error * ugv_control_param.Kp_yaw;
    // 如果yaw_error过大，则优先调整角度
    if (fabs(yaw_error) > M_PI / 3)
    {
        desired_vel.linear.x = 0.0;
    }
    // 控制指令限幅
    desired_vel.linear.x = constrain_function(desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.linear.y = constrain_function(desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.angular.z = constrain_function(desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);
    // 发布控制指令
    ugv_cmd_vel_pub.publish(desired_vel);
}

void UGV_CONTROL::path_control()
{
    double x_ref, y_ref;
    if (astar_path.size() > 0)
    {
        x_ref = astar_path[0].x * this->resolution + this->ugv_geo_fence.min_x;
        y_ref = astar_path[0].y * this->resolution + this->ugv_geo_fence.min_y;
        pos_control_diff(x_ref, y_ref, 0);
        // 当前位置到目标点需要的偏航角 效果不好 需要修改
        double yaw_ref = atan2(y_ref - ugv_state.position[1], x_ref - ugv_state.position[0]);
        pos_control(x_ref, y_ref, yaw_ref);
    }
    else
    {
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.angular.z = 0.0;
        ugv_cmd_vel_pub.publish(desired_vel);
        return;
    }
    if (abs(x_ref - ugv_state.position[0]) < 0.2 && abs(y_ref - ugv_state.position[1]) < 0.2)
    {
        std::cout << "astar_path size: " << astar_path.size() << std::endl;
        astar_path.erase(astar_path.begin());
    }
}

// 【坐标系旋转函数】- enu系到body系
void UGV_CONTROL::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    body_frame[0] = enu_frame[0] * cos(yaw_angle) + enu_frame[1] * sin(yaw_angle);
    body_frame[1] = -enu_frame[0] * sin(yaw_angle) + enu_frame[1] * cos(yaw_angle);
}

// 定时器回调函数：定时打印
void UGV_CONTROL::timercb_debug(const ros::TimerEvent &e)
{
    if (!flag_printf)
    {
        return;
    }
    cout << GREEN << ">>>>>>>>>>>>>> UGV [" << ugv_id << "] <<<<<<<<<<<<<<<" << TAIL << endl;
    // 固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    // 左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    if (11.3f < ugv_state.battery_state < 13.0f)
    {
        cout << GREEN << "Battery: " << ugv_state.battery_state << " [V] <<<<<<<<<<<<<" << TAIL << endl;
    }
    else if (11.0f < ugv_state.battery_state < 11.3f)
    {
        cout << YELLOW << "Battery: " << ugv_state.battery_state << " [V] <<<<<<<<<<<<<" << TAIL << endl;
    }
    else
    {
        cout << RED << "Low Battery: " << ugv_state.battery_state << " [V] <<<<<<<<<<<<<" << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y] : " << ugv_state.position[0] << " [ m ] " << ugv_state.position[1] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y] : " << ugv_state.velocity[0] << " [m/s] " << ugv_state.velocity[1] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [Yaw] : " << ugv_state.yaw * 180 / M_PI << " [deg] " << TAIL << endl;

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if (!ugv_state.odom_valid)
    {
        cout << RED << "Odom_valid: [Invalid] <<<<<<<<<<<<<" << TAIL << endl;
        return;
    }
}

// 定时器回调函数：定时发布ugv_state
void UGV_CONTROL::timercb_state(const ros::TimerEvent &e)
{
    // 发布 ugv_state
    ugv_state.header.stamp = ros::Time::now();

    if ((ros::Time::now() - get_odom_time).toSec() > ODOM_TIMEOUT)
    {
        ugv_state.odom_valid = false;
    }

    ugv_state.control_mode = current_ugv_cmd.cmd;
    ugv_state_pub.publish(ugv_state);
}

// 定时器回调函数：定时重置A*算法
void UGV_CONTROL::timercb_update_astar(const ros::TimerEvent &e)
{
    if (goal_set && current_ugv_cmd.cmd == sunray_msgs::UGVControlCMD::Point_Control_with_Astar)
    {
        int start_x = static_cast<int>((ugv_state.position[0] - this->ugv_geo_fence.min_x) / this->resolution);
        int start_y = static_cast<int>((ugv_state.position[1] - this->ugv_geo_fence.min_y) / this->resolution);
        int goal_x = static_cast<int>((planner_goal.pose.position.x - this->ugv_geo_fence.min_x) / this->resolution);
        int goal_y = (static_cast<int>((planner_goal.pose.position.y - this->ugv_geo_fence.min_y) / this->resolution));
        astar.setGoal(goal_x, goal_y);
        astar.setStart(start_x, start_y);
        astar.setGraph(map_gen.grid);
        astar.a_star_search();
        astar_path.clear();
        astar_path = astar.reconstruct_path();
        if (astar_path.size() > 0)
        {
            astar_path.erase(astar_path.begin());
        }

        planner_path.poses.clear();
        planner_path.header.stamp = ros::Time::now();
        planner_path.header.frame_id = "odom";
        for (const auto &loc : astar_path)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "odom";
            pose_stamped.pose.position.x = loc.x * this->resolution + this->ugv_geo_fence.min_x;
            pose_stamped.pose.position.y = loc.y * this->resolution + this->ugv_geo_fence.min_y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            planner_path.poses.push_back(pose_stamped);
        }
        path_pub.publish(planner_path);
    }
}

// 回调函数：动捕
void UGV_CONTROL::mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    get_odom_time = ros::Time::now(); // 记录时间戳，防止超时
    ugv_state.position[0] = msg->pose.position.x;
    ugv_state.position[1] = msg->pose.position.y;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d ugv_att = quaternion_to_euler(q_mocap);

    ugv_state.yaw = ugv_att.z();

    ugv_state.odom_valid = true;
}

void UGV_CONTROL::gazebo_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    ugv_state.position[0] = msg->pose.pose.position.x;
    ugv_state.position[1] = msg->pose.pose.position.y;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Vector3d ugv_att = quaternion_to_euler(q_mocap);

    ugv_state.yaw = ugv_att.z();
    ugv_state.odom_valid = true;
}

// 回调函数：动捕
void UGV_CONTROL::mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    ugv_state.velocity[0] = msg->twist.linear.x;
    ugv_state.velocity[1] = msg->twist.linear.y;
}

// 回调函数：电池电量
void UGV_CONTROL::battery_cb(const std_msgs::Float32::ConstPtr &msg)
{
    ugv_state.connected = true;
    ugv_state.battery_state = msg->data;
}

// 回调函数：move_base_goal 目标点, 用于A*算法
void UGV_CONTROL::goal_point_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_set = true;
    current_ugv_cmd.cmd = sunray_msgs::UGVControlCMD::Point_Control_with_Astar;
    planner_goal = *msg;
}

// // 定时器回调函数 - 定时发送RVIZ显示数据（仿真）
// void UGV_CONTROL::timercb_rviz(const ros::TimerEvent &e)
// {
//     // 发布智能机位置marker
//     visualization_msgs::Marker ugv_marker;
//     ugv_marker.header.frame_id = "world";
//     ugv_marker.header.stamp = ros::Time::now();
//     ugv_marker.ns = "mesh";
//     ugv_marker.id = 0;
//     ugv_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//     ugv_marker.scale.x = 0.05;
//     ugv_marker.scale.y = 0.05;
//     ugv_marker.scale.z = 0.05;
//     ugv_marker.action = visualization_msgs::Marker::ADD;
//     ugv_marker.pose.position.x = ugv_state.position[0];
//     ugv_marker.pose.position.y = ugv_state.position[1];
//     ugv_marker.pose.position.z = ugv_state.position[2];
//     ugv_marker.pose.orientation.w = ugv_state.attitude_q.w;
//     ugv_marker.pose.orientation.x = ugv_state.attitude_q.x;
//     ugv_marker.pose.orientation.y = ugv_state.attitude_q.y;
//     ugv_marker.pose.orientation.z = ugv_state.attitude_q.z;
//     ugv_marker.color = led_color;
//     ugv_marker.mesh_use_embedded_materials = false;
//     ugv_marker.mesh_resource = std::string("package://sunray_ugv/meshes/wheeltec.dae");
//     ugv_mesh_pub.publish(ugv_marker);

//     // 发布运动轨迹，用于rviz显示
//     geometry_msgs::PoseStamped ugv_pos;
//     ugv_pos.header.stamp = ros::Time::now();
//     ugv_pos.header.frame_id = "world";
//     ugv_pos.pose.position.x = ugv_state.position[0];
//     ugv_pos.pose.position.y = ugv_state.position[1];
//     ugv_pos.pose.position.z = ugv_state.position[2];
//     ugv_pos.pose.orientation = ugv_state.attitude_q;
//     pos_vector.insert(pos_vector.begin(), ugv_pos);
//     if (pos_vector.size() > TRA_WINDOW)
//     {
//         pos_vector.pop_back();
//     }
//     nav_msgs::Path ugv_trajectory;
//     ugv_trajectory.header.stamp = ros::Time::now();
//     ugv_trajectory.header.frame_id = "world";
//     ugv_trajectory.poses = pos_vector;
//     ugv_trajectory_pub.publish(ugv_trajectory);

//     // 发布当前执行速度的方向箭头
//     geometry_msgs::TwistStamped vel_rviz;
//     vel_rviz.header.stamp = ros::Time::now();
//     vel_rviz.header.frame_id = uav_prefix + "/base_link";
//     vel_rviz.twist.linear.x = desired_vel.linear.x;
//     vel_rviz.twist.linear.y = desired_vel.linear.y;
//     vel_rviz.twist.linear.z = desired_vel.linear.z;
//     vel_rviz.twist.angular.x = desired_vel.angular.x;
//     vel_rviz.twist.angular.y = desired_vel.angular.y;
//     vel_rviz.twist.angular.z = desired_vel.angular.z;
//     vel_rviz_pub.publish(vel_rviz);

//     // 发布当前目标点marker，仅针对位置控制模式
//     if(current_ugv_cmd.control_state == sunray_msgs::UGVControlCMD::POS_CONTROL)
//     {
//         // 发布目标点mesh
//         visualization_msgs::Marker goal_marker;
//         goal_marker.header.frame_id = "world";
//         goal_marker.header.stamp = ros::Time::now();
//         goal_marker.ns = "goal";
//         goal_marker.id = ugv_id;
//         goal_marker.type = visualization_msgs::Marker::SPHERE;
//         goal_marker.action = visualization_msgs::Marker::ADD;
//         goal_marker.pose.position.x = current_ugv_cmd.desired_pos.x;
//         goal_marker.pose.position.y = current_ugv_cmd.desired_pos.y;
//         goal_marker.pose.position.z = 0;
//         goal_marker.pose.orientation.x = 0.0;
//         goal_marker.pose.orientation.y = 0.0;
//         goal_marker.pose.orientation.z = 0.0;
//         goal_marker.pose.orientation.w = 1.0;
//         goal_marker.scale.x = 0.2;
//         goal_marker.scale.y = 0.2;
//         goal_marker.scale.z = 0.2;
//         goal_marker.color = led_color;
//         goal_marker.mesh_use_embedded_materials = false;
//         goal_point_pub.publish(goal_marker);
//     }

//     // 发布TF用于RVIZ显示
//     static tf2_ros::TransformBroadcaster broadcaster;
//     geometry_msgs::TransformStamped tfs;
//     //  |----头设置
//     tfs.header.frame_id = "world";       //相对于世界坐标系
//     tfs.header.stamp = ros::Time::now(); //时间戳
//     //  |----坐标系 ID
//     tfs.child_frame_id = "ugv_" + std::to_string(ugv_id) + "/base_link"; //子坐标系，无人机的坐标系
//     //  |----坐标系相对信息设置  偏移量  无人机相对于世界坐标系的坐标
//     tfs.transform.translation.x = ugv_state.position[0];
//     tfs.transform.translation.y = ugv_state.position[1];
//     tfs.transform.translation.z = ugv_state.position[2];
//     //  |--------- 四元数设置
//     tfs.transform.rotation = ugv_state.attitude_q;
//     //  |--------- 广播器发布数据
//     broadcaster.sendTransform(tfs);
// }

// 限制幅度函数
float UGV_CONTROL::constrain_function(float data, float Max, float Min)
{
    if (abs(data) > Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else if (abs(data) < Min)
    {
        return 0.0;
    }
    else
    {
        return data;
    }
}

// 地理围栏检查函数
bool UGV_CONTROL::check_geo_fence()
{
    // 安全检查，超出地理围栏自动降落,打印相关位置信息
    if (ugv_state.position[0] > ugv_geo_fence.max_x || ugv_state.position[0] < ugv_geo_fence.min_x ||
        ugv_state.position[1] > ugv_geo_fence.max_y || ugv_state.position[1] < ugv_geo_fence.min_y)
    {
        ROS_WARN_STREAM("ugv [" << ugv_id << "] out of geofence land! Position: ["
                                << ugv_state.position[0] << ", " << ugv_state.position[1] << "], Geofence: ["
                                << ugv_geo_fence.min_x << ", " << ugv_geo_fence.max_x << ", "
                                << ugv_geo_fence.min_y << ", " << ugv_geo_fence.max_y << "]");
        return 1;
    }
    return 0;
}

// 四元数转欧拉角
Eigen::Vector3d UGV_CONTROL::quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

// 打印参数
// void UGV_CONTROL::printf_param()
// {
//     cout << GREEN << ">>>>>>>>>>>>>>>>>>> UGV_CONTROL Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;

//     cout << GREEN << "ugv_id : " << ugv_id << "" << TAIL << endl;
//     cout << GREEN << "flag_printf : " << flag_printf << "" << TAIL << endl;

//     // 悬停控制参数
//     cout << GREEN << "Kp_xy : " << ugv_control_param.Kp_xy << TAIL << endl;
//     cout << GREEN << "Kp_yaw : " << ugv_control_param.Kp_yaw << TAIL << endl;
//     cout << GREEN << "max_vel_xy : " << ugv_control_param.max_vel_xy << " [m/s]" << TAIL << endl;
//     cout << GREEN << "max_vel_yaw : " << ugv_control_param.max_vel_yaw << " [rad/s]" << TAIL << endl;

//     // 地理围栏参数
//     cout << GREEN << "geo_fence max_x : " << ugv_geo_fence.max_x << " [m]" << TAIL << endl;
//     cout << GREEN << "geo_fence min_x : " << ugv_geo_fence.min_x << " [m]" << TAIL << endl;
//     cout << GREEN << "geo_fence max_y : " << ugv_geo_fence.max_y << " [m]" << TAIL << endl;
//     cout << GREEN << "geo_fence min_y : " << ugv_geo_fence.min_y << " [m]" << TAIL << endl;
// }