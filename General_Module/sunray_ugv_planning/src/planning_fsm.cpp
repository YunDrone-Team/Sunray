#include "planning_fsm.h"

namespace ugv_planning
{
// 初始化函数
void Planning_FSM::init(ros::NodeHandle& nh)
{
    // 读取参数
    // 无人车编号 1号无人车则为1
    nh.param("ugv_planning/ugv_id", ugv_id, 1);
    // 无人车高度
    nh.param("ugv_planning/ugv_height", ugv_height, 0.0);
    // A星算法 重规划频率 
    nh.param("ugv_planning/replan_time", replan_time, 1.0); 
    // 路径点发布频率
    nh.param("ugv_planning/path_point_pub_frequency", path_point_pub_frequency, 0.1); 
    // 选择地图更新方式：　0代表全局点云，1代表局部点云，2代表激光雷达scan数据, 3代表viobot
    nh.param("ugv_planning/map_input_source", map_input_source, 3); 

    ugv_name = "/ugv" + std::to_string(ugv_id);

    //【订阅】 根据map_input选择地图更新方式
    if(map_input_source == 0)
    {
        cout << GREEN << "Global pcl mode, subscirbe to "<< ugv_name << "/sunray/global_pcl" << TAIL <<endl;
        Gpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/sunray/global_pcl", 1, &Planning_FSM::Gpointcloud_cb, this);
    }else if(map_input_source == 1)
    {
        cout << GREEN << "Local pcl mode, subscirbe to "<< ugv_name << "/sunray/local_pcl" << TAIL <<endl;
        Lpointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(ugv_name + "/sunray/local_pcl", 1, &Planning_FSM::Lpointcloud_cb, this);
    }else if(map_input_source == 2)
    {
        cout << GREEN << "Laser scan mode, subscirbe to "<< ugv_name << "/sunray/laser_scan" << TAIL <<endl;
        laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>(ugv_name + "/sunray/laser_scan", 1, &Planning_FSM::laser_cb, this);
    }else if(map_input_source == 3)
    {
        cout << GREEN << "VIOBOT rdf point mode, subscirbe to " << "/viobot/pr_loop/rdf_points" << TAIL <<endl;
        viobot_pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/viobot/pr_loop/rdf_points", 1, &Planning_FSM::viobot_cb, this);
    }

    // 【订阅】目标点
    goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(ugv_name + "/sunray/goal", 1, &Planning_FSM::goal_cb, this);

    // 【订阅】无人车里程计
    ugv_odom_sub = nh.subscribe<nav_msgs::Odometry>("/viobot/pr_loop/odometry_rect", 10, &Planning_FSM::ugv_odom_cb, this);

    // 【发布】 路径控制指令 （发送至WheeltecRobot类）
    ugv_control_cmd_pub = nh.advertise<sunray_msgs::UGVControlCMD>(ugv_name + "/sunray/ugv_control_cmd", 1);

    // 【发布】路径用于显示（rviz显示）
    ros_path_pub   = nh.advertise<nav_msgs::Path>(ugv_name + "/sunray/path_cmd",  1); 

    // 【定时器】主循环执行
    mainloop_timer = nh.createTimer(ros::Duration(0.1), &Planning_FSM::mainloop_cb, this);        
    
    // 【定时器】路径点发布定时器
    path_point_pub_timer = nh.createTimer(ros::Duration(path_point_pub_frequency), &Planning_FSM::path_point_pub_cb, this);        

    // Astar algorithm
    Astar_ptr.reset(new Astar);
    Astar_ptr->init(nh);

    // 规划器状态参数初始化
    exec_state = EXEC_STATE::WAIT_GOAL;
    odom_ready = false;
    get_goal = false;
    map_ready = false;
    path_ok = false;
    counter_search = 0;
    yaw_ref = 0.0;

    // 初始化发布的指令
    UGV_CMD.header.stamp = ros::Time::now();
    UGV_CMD.cmd  = sunray_msgs::UGVControlCMD::Hold;
}

void Planning_FSM::goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    // 高度为恒定值
    goal_pos << msg->pose.position.x, msg->pose.position.y, ugv_height;

    get_goal = true;

    cout << GREEN << "Get a new manual goal: ["<< goal_pos(0) << ", "  << goal_pos(1)  << ", "  << goal_pos(2) << " ]"  << TAIL <<endl;
}

void Planning_FSM::ugv_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ready = true;

    //无人车里程计，用于建图
    ugv_odom = *msg;
    ugv_odom.child_frame_id = "base_link";
    ugv_odom.pose.pose.position.z = ugv_height;     //无人车固定高度
    ugv_yaw = 0.0;

    // 更新无人车初始位置、速度、加速度，用于规划
    start_pos << ugv_odom.pose.pose.position.x, ugv_odom.pose.pose.position.y, ugv_odom.pose.pose.position.z;
}

// 根据全局点云更新地图
// 情况：已知全局点云的场景、由SLAM实时获取的全局点云
void Planning_FSM::Gpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }
    map_ready = true;
    static int update_num=0;
    update_num++;
    // 此处改为根据循环时间计算的数值
    if(update_num == 5)
    {
        // 对Astar中的地图进行更新
        Astar_ptr->Occupy_map_ptr->map_update_gpcl(msg);
        update_num = 0;
    }
}

// 根据VIOBOT数据更新地图
void Planning_FSM::viobot_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }

    map_ready = true;
    // 对Astar中的地图进行更新（laser+odom）并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->map_update_viobot(msg, ugv_odom);
}

// 根据局部点云更新地图
// 情况：RGBD相机、三维激光雷达
void Planning_FSM::Lpointcloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }
    map_ready = true;
    Astar_ptr->Occupy_map_ptr->map_update_lpcl(msg, ugv_odom);
}

// 根据2维雷达数据更新地图
// 情况：2维激光雷达
void Planning_FSM::laser_cb(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!odom_ready) 
    {
        return;
    }

    map_ready = true;
    // 对Astar中的地图进行更新（laser+odom）并对地图进行膨胀
    Astar_ptr->Occupy_map_ptr->map_update_laser(msg, ugv_odom);
}

void Planning_FSM::path_point_pub_cb(const ros::TimerEvent& e)
{
    static int track_path_num = 0;

    if(!path_ok)
    {
        UGV_CMD.header.stamp = ros::Time::now();
        UGV_CMD.cmd         = sunray_msgs::UGVControlCMD::Hold;
        ugv_control_cmd_pub.publish(UGV_CMD); 
        return;
    }

    // 抵达终点
    if(cur_id == Num_total_wp - 1)
    {
        UGV_CMD.header.stamp = ros::Time::now();
        UGV_CMD.cmd              = sunray_msgs::UGVControlCMD::Point_Control;
        UGV_CMD.desired_pos[0]     = path_cmd.poses[cur_id].pose.position.x;
        UGV_CMD.desired_pos[1]     = path_cmd.poses[cur_id].pose.position.y;
        UGV_CMD.desired_yaw      = yaw_ref;
        ugv_control_cmd_pub.publish(UGV_CMD);
        cout << GREEN << ugv_name + " Path tracking: [ Reach the goal ]."  << TAIL <<endl;
        
        // 停止执行
        path_ok = false;

        float error_pos = sqrt ((path_cmd.poses[cur_id].pose.position.x - start_pos[0])*(path_cmd.poses[cur_id].pose.position.x - start_pos[0])                  
                                                               +    (path_cmd.poses[cur_id].pose.position.y - start_pos[1])*(path_cmd.poses[cur_id].pose.position.y - start_pos[1])     );
        float error_yaw = abs(yaw_ref - ugv_yaw);

        // 等待无人车移动至目标状态
        if(error_pos < 0.2 && error_yaw<0.1)
        {
            exec_state = EXEC_STATE::WAIT_GOAL;
        }
        return;
    }

    UGV_CMD.header.stamp = ros::Time::now();
    UGV_CMD.cmd              = sunray_msgs::UGVControlCMD::Point_Control_with_Astar;
    UGV_CMD.desired_pos[0]     = path_cmd.poses[cur_id].pose.position.x;
    UGV_CMD.desired_pos[1]     = path_cmd.poses[cur_id].pose.position.y;
    UGV_CMD.desired_yaw      =   yaw_ref;

    ugv_control_cmd_pub.publish(UGV_CMD); 
    cur_id = cur_id + 1;
}
 
// 主循环 
void Planning_FSM::mainloop_cb(const ros::TimerEvent& e)
{
    static int exec_num=0;
    exec_num++;

    // 检查当前状态，不满足规划条件则直接退出主循环
    if(!odom_ready || !map_ready)
    {
        // 此处改为根据循环时间计算的数值
        if(exec_num == 100)
        {
            if(!odom_ready)
            {
                cout << YELLOW << ugv_name + " Main loop init check: [ Need Odom ]."  << TAIL <<endl;
            }else if(!map_ready)
            {
                cout << YELLOW << ugv_name + " Main loop init check: [ Need sensor info ]."  << TAIL <<endl;
            }
            exec_num=0;
        }  
        return;
    }else
    {
        // 对检查的状态进行重置,此处要求无人车状态、传感器信息回调频率要高于本定时器
        odom_ready = false;
        if(map_input_source != 0)
        {
            map_ready = false;
        }

        if(exec_num >= 20)
        {
            // 状态打印
            printf_exec_state();
            exec_num=0;
        }  

    }
    
    switch (exec_state)
    {
        case EXEC_STATE::WAIT_GOAL:

            // 等待目标点，不执行路径追踪逻辑
            path_ok = false;           
        
            // 等待手动输入的目标值
            if(!get_goal)
            {
                if(exec_num == 100)
                {
                    cout << YELLOW << ugv_name + " Waiting for a new goal, subscirbe to "<< ugv_name << "/sunray/goal" << TAIL <<endl;
                }
            }else
            {
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLAN;
                get_goal = false;
            }

            break;
        
        case EXEC_STATE::PLAN:

            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if(astar_state==Astar::NO_PATH)
            {
                // 找不到路径：返回 WAIT_GOAL
                if(counter_search > 50)
                {
                    path_ok = false;
                    exec_state = EXEC_STATE::WAIT_GOAL;
                    counter_search = 0;
                }
                counter_search++;
                cout << RED << ugv_name + " Main loop Planning [ Planner can't find path ]" << TAIL <<endl;
            }
            else
            {
                path_ok = true;
                counter_search = 0;
                path_cmd = Astar_ptr->get_ros_path();
                // 路径中航点数目
                Num_total_wp = path_cmd.poses.size();
                cur_id = 1;
                tra_start_time = ros::Time::now();
                // 路径规划成功，进入PATH_TRACKING
                exec_state = EXEC_STATE::PATH_TRACKING;
                // 发布路劲用于rviz显示
                ros_path_pub.publish(path_cmd);
                cout << GREEN << ugv_name + " Main loop Planning [ Get a new path ]" << TAIL <<endl;
            }

            break;
        
        case EXEC_STATE::PATH_TRACKING:
        
            // 执行时间达到阈值，重新执行一次规划
            if(get_time_in_sec(tra_start_time) >= replan_time)
            {
                exec_state = EXEC_STATE::PLAN;
            }

            break;

        case EXEC_STATE::STOP:
            path_ok = false;
            UGV_CMD.header.stamp = ros::Time::now();
            UGV_CMD.cmd         = sunray_msgs::UGVControlCMD::Hold;
            ugv_control_cmd_pub.publish(UGV_CMD);
            break;
    }

}

// 【获取当前时间函数】 单位：秒
float Planning_FSM::get_time_in_sec(const ros::Time& begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void Planning_FSM::printf_exec_state()
{
    switch (exec_state)
    {
        case EXEC_STATE::WAIT_GOAL:
            cout << GREEN << ugv_name + " Main loop Exec_state: [ WAIT_GOAL ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PLAN: 
            cout << GREEN << ugv_name + " Main loop Exec_state: [ PLAN ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::PATH_TRACKING:
            cout << GREEN << ugv_name + " Main loop Exec_state: [ PATH_TRACKING ]."  << TAIL <<endl;
            break;
        case EXEC_STATE::STOP:
            cout << GREEN << ugv_name + " Main loop Exec_state: [ STOP ]."  << TAIL <<endl;
            break;  
    }    
}

int Planning_FSM::get_start_point_id(void)
{
    // 选择与当前无人车所在位置最近的点,并从该点开始追踪
    int id = 0;
    float distance_to_wp_min = abs(path_cmd.poses[0].pose.position.x - start_pos[0])
                                + abs(path_cmd.poses[0].pose.position.y - start_pos[1])
                                + abs(path_cmd.poses[0].pose.position.z - start_pos[2]);
    
    float distance_to_wp;

    for(int j=1; j<Num_total_wp;j++)
    {
        distance_to_wp = abs(path_cmd.poses[j].pose.position.x - start_pos[0])
                                + abs(path_cmd.poses[j].pose.position.y - start_pos[1])
                                + abs(path_cmd.poses[j].pose.position.z - start_pos[2]);
        
        if(distance_to_wp < distance_to_wp_min)
        {
            distance_to_wp_min = distance_to_wp;
            id = j;
        }
    }

    //　为防止出现回头的情况，此处对航点进行前馈处理
    if(id + 1 < Num_total_wp)
    {
        id = id + 1;
    }

    return id;
}

const int Planning_FSM::get_track_point_id()
{
    // 选择与当前无人车所在位置最近的点,并从该点开始追踪
    int id = get_start_point_id();

    if (id == Num_total_wp-1){ // 如果已经是终点
        return id;
    }

    double dist_sum = 0;

    double next_dist = sqrt(pow((path_cmd.poses[id+1].pose.position.x - path_cmd.poses[id].pose.position.x), 2)
        + pow((path_cmd.poses[id+1].pose.position.y - path_cmd.poses[id].pose.position.y), 2));

    while(dist_sum + next_dist < 1.2){
        // std::cout << next_dist << std::endl;
        id++;
        dist_sum += next_dist;
        if (!(id+1 < Num_total_wp)){
            next_dist = sqrt(pow((path_cmd.poses[id+1].pose.position.x - path_cmd.poses[id].pose.position.x), 2)
                + pow((path_cmd.poses[id+1].pose.position.y - path_cmd.poses[id].pose.position.y), 2));
        }
        else{
            continue;
        }
    }

    return id;

}

}