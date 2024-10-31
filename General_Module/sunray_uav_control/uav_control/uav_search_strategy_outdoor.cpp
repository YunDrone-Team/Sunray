#include "uav_search_strategy.h"

uav_search_strategy::uav_search_strategy(ros::NodeHandle &nh_)
{
    // 【参数】无人机编号
    nh_.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机名称
    nh_.param<std::string>("uav_name", uav_name, "uav");
    nh_.param<string>("vision_source",vision_source,"VO"); // 默认VO

    uav_name = "/" + uav_name + std::to_string(uav_id);


    // 【参数】起飞指令，该指令为true，则只需要解锁就行，后续指令会自动执行；否则，则需要手动输入
    nh_.param<int>("takeoff", takeoff, 0);
    

    // ROS_INFO("uav_name=%s",uav_name);
    pub_trajectory_ = nh_.createTimer(ros::Duration(0.1), &uav_search_strategy::pub_trajectory_cb, this);
    setup_pub_ = nh_.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup",1);
    cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd",1);
    yolov8_detect_id_sub_ = nh_.subscribe(uav_name + "/detection_id", 1, &uav_search_strategy::detect_id_cb, this);

    uav_rc_in_sub = nh_.subscribe(uav_name + "/mavros/rc/in", 1, &uav_search_strategy::pc4_rc_in_cb, this);

    // stage话题发布
    stage_pub_ = nh_.advertise<std_msgs::Int16>(uav_name + "/stage", 1);
    // 无人机搜索状态发布
    uav_search_state_pub_ = nh_.advertise<std_msgs::Int16>(uav_name + "/uav_search_state", 1);
    uav_search_state.data = 0;
    uav_search_state_all_sub_ = nh_.subscribe(uav_name + "/uav_search_state_all", 1, &uav_search_strategy::uav_search_state_all_cb, this);
    uav_search_state_all = 0;
 
    // 回复信号订阅
    send_sub_ = nh_.subscribe(uav_name + "/send", 1, &uav_search_strategy::send_cb, this);
    // 前视相机检测的boxes坐标订阅
    boxes_sub_ = nh_.subscribe(uav_name + "/front_detection", 1, &uav_search_strategy::boxes_cb, this) ;

    yolov8_detect_pose_pub_ = nh_.advertise<sunray_msgs::detection>(uav_name + "/detection", 1);

    // 订阅无人机yolo检测到的坐标
    uav_targetPose_sub = nh_.subscribe("/targetPos", 1, &uav_search_strategy::uav_targetPose_cb, this);

    uav_true_pose_pub_ = nh_.advertise<sunray_msgs::targetPos>(uav_name + "/true_pose", 1);
    uav_init_pose_pub_ = nh_.advertise<sunray_msgs::targetPos>(uav_name + "/init_pose", 1);

    // 初始化yolov8_pose_为3个boxes
    for (int i = 0; i < 3; ++i) {
        sunray_msgs::targetBoxes box;
        box.x = 0.0; // 设置初始x坐标
        box.y = 0.0; // 设置初始y坐标
        yolov8_detect_pose_.boxes.push_back(box);
    }
    
    // 【订阅】无人机当前速度 坐标系:ENU系 (PX4 -> sunray_matlab)
    px4_velocity_sub = nh_.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 1, &uav_search_strategy::local_vel_ned_cb, this);

    // 【订阅】无人机起飞指令
    uav_takeoff_sub_ = nh_.subscribe("/takeoff", 1, &uav_search_strategy::takeoff_cb, this);

    lidar_height_sub_ = nh_.subscribe<sensor_msgs::Range>(uav_name + "/mavros/distance_sensor/hrlv_ez4_pub", 1, &uav_search_strategy::lidar_height_cb, this);
    

    // 【参数】外部定位数据来源
    nh_.param<int>("external_source", external_source, sunray_msgs::ExternalOdom::GAZEBO);

    if (external_source == sunray_msgs::ExternalOdom::VINS)
    {
        // 【订阅】 vins话题订阅
        if(vision_source == "VIO")
            vins_sub_ = nh_.subscribe("/vins_estimator/imu_propagate", 1, &uav_search_strategy::vins_cb, this);
        else if(vision_source == "VO")
            vins_sub_ = nh_.subscribe("/vins_estimator/odometry", 1, &uav_search_strategy::vins_cb, this);
        // 初始化无人机gazebo坐标
        init_pose = Eigen::Vector3d(0,0,0);
        std::cout << GREEN << "External Odom: [ VINS ] " << TAIL << std::endl;
    }
    else if(external_source == sunray_msgs::ExternalOdom::GAZEBO)
    {
        // 【订阅】gazebo仿真真值
        gazebo_sub_ = nh_.subscribe<nav_msgs::Odometry>(uav_name + "/sunray/gazebo_pose", 1, &uav_search_strategy::gazebo_cb, this);
        // 初始化无人机gazebo坐标
        init_pose = Eigen::Vector3d(0,0,0);
        std::cout << GREEN << "External Odom: [ GAZEBO ] " << TAIL << std::endl;
    }
    else if(external_source == sunray_msgs::ExternalOdom::MOCAP)
    {
        // 【订阅】MOCAP仿真真值
        // 【订阅】mocap pose
        mocap_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node" + uav_name + "/pose", 1, &uav_search_strategy::mocap_pos_cb, this);
        // 【订阅】mocap Twist
        mocap_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node" + uav_name + "/twist", 1, &uav_search_strategy::mocap_vel_cb, this);
        std::cout << GREEN << "External Odom: [ MOCAP ] " << TAIL << std::endl;
    }
    else
    {
        cout << RED << node_name << ": wrong external_source param, no external location information input!" << TAIL << endl;
    }

    // 控制模式
    nh_.param<bool>("velocity_ctrl_mode", velocity_ctrl_mode, false); // true:速度控制，false:位置控制

    // 读取区域的 w,h 初始化edges_poses_
    nh_.param<float>("square_width", square_width, 6);
    nh_.param<float>("square_height", square_height, 6);
    nh_.param<float>("step", step, 3);
    nh_.param<float>("height_step",height_step,3);
    nh_.param<float>("back_step", back_step, 0.5);

    nh_.param<int>("uav_group", uav_group, 1); // 默认为1组
    // 【参数】默认起飞高度
    nh_.param<float>("Takeoff_height", Takeoff_height, 0.5);

    nh_.param<float>("State1_height", State1_height, 0.5);
    nh_.param<float>("State2_height", State2_height, 0.5);
    nh_.param<float>("height_up", height_up, 1.5);//飞机飞行高度的上限
    nh_.param<bool>("enable_hight_lidar", enable_hight_lidar, false);

    // 前往搜索区域的纵向步长
    nh_.param<float>("service_height", service_height, 5);
    // 整个场地的宽度
    nh_.param<float>("total_width", total_width, 10);
    

    // 搜索策略
    nh_.param<std::string>("search_strategy", search_strategy, "h_search");

    // 获取电子围栏
    nh_.param<float>("x_min", x_min, -10.0);
    nh_.param<float>("x_max", x_max, 10.0);
    nh_.param<float>("y_min", y_min, -10.0);
    nh_.param<float>("y_max", y_max, 10.0);
    nh_.param<float>("z_min", z_min, -1.0);
    nh_.param<float>("z_max", z_max, 3.0);

    // boxes_poses_.resize(5);

    // edges_poses_.push_back(Eigen::Vector3d(0, 0, Takeoff_height)); 
    // edges_poses_.push_back(Eigen::Vector3d(0, -step, Takeoff_height)); // 初始化到(0,-6,0.5)的位置
    if(search_strategy == "h_search")
    {
        edges_poses_.push_back(Eigen::Vector3d(-back_step, 0, State1_height)); 
        edges_poses_.push_back(Eigen::Vector3d(-back_step, -step, State1_height)); // 初始化到(0,-6,0.5)的位置
    }
    else if(search_strategy == "v_search")
    {
        edges_poses_.push_back(Eigen::Vector3d(0, -back_step, State1_height));
        edges_poses_.push_back(Eigen::Vector3d(height_step, -back_step, State1_height));
    }
    else if(search_strategy == "v_center_search")
    {
        edges_poses_.push_back(Eigen::Vector3d(square_height - height_step, 0, State1_height));
        edges_poses_.push_back(Eigen::Vector3d(0, 0, State1_height));
    }
    else if(search_strategy == "v_center_search_new")
    {
        edges_poses_.push_back(Eigen::Vector3d(square_height - 3, 0, State1_height));
        edges_poses_.push_back(Eigen::Vector3d(0, 0, State1_height));
    }
    
    uav_state = 0; // uav在搜索的哪一个阶段进入搜索box阶段

    // for(int i=0; i < 20;i++)
    // {
    //     filter_pose_buff_index.push_back(i);
    //     filter_pose_buff.push_back(Eigen::Vector3d(0,0,0)); // 初始化
    // }
    buff_len = 10;
    filter_pose_buff.resize(buff_len);
    pose_buff_mean.resize(buff_len);
    for(int i=0;i<buff_len;i++)
    {
        pose_buff_mean[i].push_back(Eigen::Vector3d(0,0,0));
    }

    print_info();

}

void uav_search_strategy::takeoff_cb(const std_msgs::Int16::ConstPtr & msg)
{
    if (takeoff == 0)
    {
        takeoff = msg->data;
        std::cout << GREEN << "已经接收起飞指令" << TAIL << std::endl;
        /*
        1：到达起飞高度，可以进行横向搜索
        2：到达横向搜索目标，可以接收前视相机发布的boxes坐标
        3：完全接收到boxes坐标
        */ 
    }
}


void uav_search_strategy::uav_targetPose_cb(const sunray_msgs::targetPos::ConstPtr & msg)
{
    // if(msg->target_id == uav_group)
    // {
    //     // 收到的id与当前group对应
    //     if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0)
    //     {
    //         // 自身没有检测到坐标，接收其他无人机检测到的id
    //         yolov8_detect_pose_.boxes[uav_group - 1].x = msg->pose_x;
    //         yolov8_detect_pose_.boxes[uav_group - 1].y = msg->pose_y;
    //     }
    // }
     // 将收到的targetPos都存起来，但只用和uav_group相同的那一个坐标
    // {
        // 收到的id与当前group对应
        if(yolov8_detect_pose_.boxes[msg->target_id - 1].x == 0.0 && yolov8_detect_pose_.boxes[msg->target_id - 1].y == 0.0)
        {
            // 自身没有检测到坐标，接收其他无人机检测到的id
            yolov8_detect_pose_.boxes[msg->target_id - 1].x = msg->pose_x; // 此时的msg->pose_x是在飞机起飞点为原点坐标系下的，还需要将接收到的坐标转到后面开始搜索时的坐标系下(在后面程序里面转，这里不用转)
            yolov8_detect_pose_.boxes[msg->target_id - 1].y = msg->pose_y;
            std::cout << GREEN << "Get Pose:" << yolov8_detect_pose_.boxes[msg->target_id - 1].x << "," << yolov8_detect_pose_.boxes[msg->target_id - 1].y << TAIL << std::endl;
        }
    // }
}

void uav_search_strategy::pc4_rc_in_cb(const mavros_msgs::RCIn::ConstPtr & msg)
{
    if((msg->channels[6] > 1000 || msg->channels[7] > 2000) && uav_ready_go) // 只有准备好之后，才接收通道值
    {
        uav_search_state.data = 1;
        uav_search_state_pub_.publish(uav_search_state); // 丢失
        std::cout << RED << "飞机已经切为手控!" << std::endl;
        uav_state = -1; // 
    }
}

void uav_search_strategy::uav_search_state_all_cb(const std_msgs::Int16::ConstPtr & msg)
{
    uav_search_state_all = msg->data;
    if(uav_search_state_all)
    {
        std::cout << GREEN << "其他无人机搜索完成！"<< TAIL << std::endl;
    }
}

void uav_search_strategy::pose_buff_clear()
{
    std::cout << RED << "清空pose_buff" << TAIL << std::endl;
    for(int i=0;i<buff_len;i++)
    {
        pose_buff_mean[i][0] = Eigen::Vector3d(0,0,0);
        filter_pose_buff[i].clear();
    }
    std::cout << GREEN << "buff_size=" << filter_pose_buff[0].size() << TAIL <<std::endl; 
}


void uav_search_strategy::boxes_cb(const sunray_msgs::front_detect::ConstPtr & msg)
{
    // 在存储的时候，还需要判断给box坐标是否在当前的栅格之类，不在的话，就下一次栅格去跑
    if(!msg->boxes.empty() && uav_ready_go)
    {
        float dis = 2; // 距离阈值
        if(search_strategy == "h_search")
        {
            for (const auto& box : msg->boxes)
            {
                std::cout << GREEN << "box:" << box.x << "," << box.y << TAIL << std::endl;
                if(box.x + box.y < 1) // 不满足条件，跳过
                    continue;
                
                // 收到的box坐标飞机起飞时的对应的世界坐标，需要转换到无人机开始搜索时的坐标系下
                Eigen::Vector3d coordinate(box.x, box.y, State2_height);
                coordinate[0] -=  init_pose[0];
                coordinate[1] -=  init_pose[1];
                std::cout << YELLOW << "收到的box坐标转换到uav搜索时的坐标系下为：" << coordinate.transpose() << TAIL << std::endl;

                if((coordinate[0] - (vins_pose[0] + back_step)) < height_step && (coordinate[0] > vins_pose[0])
                && (coordinate[1] - vins_pose[1]) < step && (coordinate[1] > vins_pose[1])) // box 在当前坐标的左前方
                // boxes坐标与当前坐标的x差值如果小于一个步长，添加进去
                {
                    if(target_num > 4) // 超过5个跳过
                        continue;
                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                break;
                            }
                        }
                    }
                    if(isNear)
                    {
                        // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                        continue;
                    }

                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);

                    std::cout << GREEN << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                    
                }
                else
                {
                    std::cout << RED << "转换后的box坐标超出搜索边界！" << TAIL << std::endl;
                }
            }
        }
        else if(search_strategy == "v_search")
        {
            for (const auto& box : msg->boxes)
            {
                std::cout << GREEN << "box:" << box.x << "," << box.y << TAIL << std::endl;
                if(box.x + box.y < 1) // 不满足条件，跳过
                    continue;
                
                // 收到的box坐标飞机起飞时的对应的世界坐标，需要转换到无人机开始搜索时的坐标系下
                Eigen::Vector3d coordinate(box.x, box.y, State2_height);
                coordinate[0] -=  init_pose[0];
                coordinate[1] -=  init_pose[1];
                std::cout << YELLOW << "收到的box坐标转换到uav搜索时的坐标系下为：" << coordinate.transpose() << TAIL << std::endl;

                if((coordinate[0] - vins_pose[0]) < height_step && (coordinate[0] > vins_pose[0]) 
                && (vins_pose[1] - coordinate[1]) < step && (coordinate[1] < vins_pose[1]) // box 在当前坐标的右前方
                && v_search_rounds == 0) // boxes坐标与当前坐标的x差值如果小于一个步长，添加进去
                {
                    if(target_num > 4)
                        continue;
                    
                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                break;
                            }
                        }
                    }
                    if(isNear)
                    {
                        // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                        continue;
                    }

                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);
                    std::cout << GREEN << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                }
                else if((vins_pose[0] - coordinate[0]) < height_step && (coordinate[0] < vins_pose[0]) 
                && (vins_pose[1] - coordinate[1]) < step && (coordinate[1] < vins_pose[1]) // box 在当前坐标的左前方
                && v_search_rounds == 1)
                {
                    if(target_num > 4)
                        continue;

                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                break;
                            }
                        }
                    }
                    if(isNear)
                        continue;
                    
                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);
                    std::cout << GREEN << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                
                }
                else
                {
                    std::cout << RED << "转换后的box坐标超出搜索边界！" << TAIL << std::endl;
                }
            }
        }
        else if(search_strategy == "v_center_search")
        {
            for (const auto& box : msg->boxes)
            {
                // if(timer_cnt == 9)
                //     std::cout << GREEN << "box:" << box.x << "," << box.y << TAIL << std::endl;
                if(box.x + box.y < 1) // 不满足条件，跳过
                    continue;

                // 收到的box坐标飞机起飞时的对应的世界坐标，需要转换到无人机开始搜索时的坐标系下
                Eigen::Vector3d coordinate(box.x, box.y, State2_height);
                coordinate[0] -=  init_pose[0];
                coordinate[1] -=  init_pose[1];

                
                if(timer_cnt == 9)
                    std::cout << YELLOW << "收到的box坐标转换到uav搜索时的坐标系下为：" << coordinate.transpose() << TAIL << std::endl;
                if( (coordinate[0] > vins_pose[0]) && (coordinate[0] <= square_height)
                && (coordinate[1] >= -(y_max - y_min) / 4) && (coordinate[1] <= (y_max - y_min) / 4) // y坐标在当前长条内
                && v_search_rounds == 0 && state != 2) // 在第一轮前进环节，且不在平移
                {
                    if(target_num > 4)
                        continue;
                    // 还需要判断该点与boxes_poses_存储的坐标是否距离小于一个阈值
                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) 
                        { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                std::cout << RED << "pose:" << coordinate.transpose() << "与" << bbox.transpose() << "离得太近，舍弃！" << TAIL << std::endl; 
                                break;
                            }
                        }
                    }
                    if(isNear)
                    {
                        // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                        continue;
                    }
                    
                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);

                    std::cout << RED << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                    
                }
                else if( (coordinate[0] > vins_pose[0]) && (coordinate[0] <= square_height)
                && (coordinate[1] <= 0) && (coordinate[1] >= -3*(y_max - y_min) / 4) // y坐标在当前长条内
                && v_search_rounds == 0 && state == 2) // 在第一轮前进环节，且在平移
                {
                    if(target_num > 4)
                        continue;
                    // 还需要判断该点与boxes_poses_存储的坐标是否距离小于一个阈值
                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                std::cout << RED << "pose:" << coordinate.transpose() << "与" << bbox.transpose() << "离得太近，舍弃！" << TAIL << std::endl; 

                                break;
                            }
                        }
                    }
                    if(isNear)
                    {
                        // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                        continue;
                    }

                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);

                    std::cout << RED << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                    
                }
                else if( (coordinate[0] < vins_pose[0]) && coordinate[0] >= 0
                && (coordinate[1] <= -(y_max - y_min) / 4) && (coordinate[1] >= -3*(y_max - y_min) / 4) // y坐标在当前长条内
                && v_search_rounds == 1) // 在第二轮前进环节
                {
                    if(target_num > 4)
                        continue;
                    // 还需要判断该点与boxes_poses_存储的坐标是否距离小于一个阈值
                    bool isNear = false;
                    if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                    { 
                        for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                            if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                isNear = true;
                                std::cout << RED << "pose:" << coordinate.transpose() << "与" << bbox.transpose() << "离得太近，舍弃！" << TAIL << std::endl; 

                                break;
                            }
                        }
                    }
                    if(isNear)
                    {
                        // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                        continue;
                    }
                        

                    boxes_poses_.push_back(coordinate);
                    box_pose_buff.push_back(coordinate);

                    std::cout << GREEN << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                    target_num++;
                
                }
                else
                {
                    // std::cout << RED << "转换后的box坐标超出搜索边界！" << TAIL << std::endl;
                }
            }
        }
        else if(search_strategy == "v_center_search_new")
        {

            int index = 0;
            bool store_ok = false;
            
            for (const auto& box : msg->boxes)
            {
                // if(timer_cnt == 9)
                //     std::cout << GREEN << "box:" << box.x << "," << box.y << TAIL << std::endl;
                
                // 收到的box坐标飞机起飞时的对应的世界坐标，需要转换到无人机开始搜索时的坐标系下
                Eigen::Vector3d coordinate(box.x, box.y, State2_height);

                filter_pose_buff[index].push_back(coordinate); // 将所有坐标都存起来
                if(filter_pose_buff[index].size() == 20)
                {
                    for(const auto & vec:filter_pose_buff[index])
                    {
                        pose_buff_mean[index][0] += vec;
                    }
                    pose_buff_mean[index][0] /= 20;
                    std::cout << RED << "收到20个坐标" << TAIL << std::endl;
                    store_ok = true;
                }
                index ++;
            }

            if(store_ok)
            {
                for(int i=0;i<pose_buff_mean.size();i++)
                {
                    Eigen::Vector3d coordinate;
                    coordinate = pose_buff_mean[i][0];
                    if(abs(coordinate[0]) < 0.1 && abs(coordinate[1]) < 0.1) // 不满足条件，跳过
                        continue;

                    coordinate[0] -=  init_pose[0];
                    coordinate[1] -=  init_pose[1];
                    
                    if(timer_cnt == 9)
                        std::cout << YELLOW << "收到的box坐标转换到uav搜索时的坐标系下为：" << coordinate.transpose() << TAIL << std::endl;
                    if( (coordinate[0] > vins_pose[0]) && (coordinate[0] <= square_height)
                    // && (coordinate[1] <= 0) && (coordinate[1] >= -(y_max - y_min) / 2) // y坐标在当前长条内
                    && (coordinate[1] <= 0) && (coordinate[1] >= -(square_width) / 2) // y坐标在当前长条内
                    
                    && v_search_rounds == 0) // 在第一轮前进环节
                    {
                        if(target_num > 4)
                            continue;
                        // 还需要判断该点与boxes_poses_存储的坐标是否距离小于一个阈值
                        bool isNear = false;
                        if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                        { 
                            for (const Eigen::Vector3d& bbox : box_pose_buff) 
                            { // 遍历boxes_poses_，计算距离
                                if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                    isNear = true;
                                    std::cout << RED << "pose:" << coordinate.transpose() << "与" << bbox.transpose() << "离得太近，舍弃！" << TAIL << std::endl; 
                                    break;
                                }
                            }
                        }
                        if(isNear)
                        {
                            // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                            continue;
                        }
                        
                        boxes_poses_.push_back(coordinate);
                        box_pose_buff.push_back(coordinate);

                        std::cout << RED << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                        target_num++;
                        
                    }
                    
                    else if( (coordinate[0] < vins_pose[0]) && coordinate[0] >= 0
                    && (coordinate[1] <= -square_width / 2) && (coordinate[1] >= -(y_max - y_min)) // y坐标在当前长条内
                    && v_search_rounds == 1) // 在第二轮前进环节
                    {
                        if(target_num > 4)
                            continue;
                        // 还需要判断该点与boxes_poses_存储的坐标是否距离小于一个阈值
                        bool isNear = false;
                        if(!box_pose_buff.empty()) // 搜到的box坐标和已有的box坐标距离较小，跳过
                        { 
                            for (const Eigen::Vector3d& bbox : box_pose_buff) { // 遍历boxes_poses_，计算距离
                                if ((coordinate - bbox).squaredNorm() < dis) { // 如果距离小于步长，则认为太近
                                    isNear = true;
                                    std::cout << RED << "pose:" << coordinate.transpose() << "与" << bbox.transpose() << "离得太近，舍弃！" << TAIL << std::endl; 

                                    break;
                                }
                            }
                        }
                        if(isNear)
                        {
                            // std::cout << RED << "在boxes_poses_中已经有和当前坐标距离相近的点了！！！" << TAIL << std::endl;
                            continue;
                        }
                            

                        boxes_poses_.push_back(coordinate);
                        box_pose_buff.push_back(coordinate);

                        std::cout << GREEN << "收到boxes_" << target_num << "：(" << coordinate[0] << "," << coordinate[1] << ")" << std::endl;
                        target_num++;
                    
                    }
                    else
                    {
                        // std::cout << RED << "转换后的box坐标超出搜索边界！" << TAIL << std::endl;
                    }
                }
            }
        }
        
    }
    // else
    //     std::cout << RED << "boxes are empty!" << std::endl;
}

void uav_search_strategy::send_cb(const std_msgs::Int16::ConstPtr & msg)
{
    if (send == 0)
    {
        send = msg->data;
        std::cout  << "已经接收send=" << send << std::endl;
        /*
        1：到达起飞高度，可以进行横向搜索
        2：到达横向搜索目标，可以接收前视相机发布的boxes坐标
        3：完全接收到boxes坐标
        */ 
    }
}

void uav_search_strategy::vins_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
    get_vins_stamp = ros::Time::now(); // 记录时间戳，防止超时
    geometry_msgs::Pose pose = msg->pose.pose;

    
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    vins_rpy = Eigen::Vector3d(roll, pitch, yaw);
    
    if(vision_source == "VIO")
    {
        double temp = pose.position.y;
        pose.position.y = -pose.position.x;
        pose.position.x = temp;

        roll = roll + M_PI/2;
        vins_rpy = Eigen::Vector3d(pitch, -roll, yaw);
    }

    if(init_pose[0] == 0 && init_pose[1] == 0 && init_pose[2] == 0 && uav_ready_go)
    {
        init_pose = Eigen::Vector3d(pose.position.x, pose.position.y, 0);
        // std::cout << RED << "UAV_" << uav_id << "init_pose = " << init_pose.transpose() << TAIL << std::endl;
    }

    vins_pose = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z) - init_pose; // 将vins坐标转到世界坐标系下
    if(enable_hight_lidar)
        vins_pose[2] = lidar_height;
    // std::cout << RED << "vinsPose:" << vins_pose.transpose() <<TAIL << std::endl;

    uav_search_state_pub_.publish(uav_search_state); // 200hz频率发布无人机状态
}

void uav_search_strategy::gazebo_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
    get_gazebo_stamp = ros::Time::now(); // 记录时间戳，防止超时
    geometry_msgs::Pose pose = msg->pose.pose;

    if(init_pose[0] == 0 && init_pose[1] == 0 && init_pose[2] == 0 && uav_ready_go)
    {
        init_pose = Eigen::Vector3d(pose.position.x, pose.position.y, 0);
        std::cout << RED << "UAV_" << uav_id << "init_pose = " << init_pose.transpose() << TAIL << std::endl;
    }
    // double temp = pose.position.y;
    // pose.position.y = -pose.position.x;
    // pose.position.x = temp;

    vins_pose = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z) - init_pose; // 将vins坐标转到世界坐标系下

    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    vins_rpy = Eigen::Vector3d(roll, pitch, yaw);

    uav_search_state_pub_.publish(uav_search_state); // 200hz频率发布无人机状态
}

void uav_search_strategy::mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    get_mocap_stamp = ros::Time::now(); // 记录时间戳，防止超时
    geometry_msgs::Pose pose = msg->pose;

    vins_pose = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z); // 将vins坐标转到世界坐标系下
    if(enable_hight_lidar)
        vins_pose[2] = lidar_height;

    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    vins_rpy = Eigen::Vector3d(roll, pitch, yaw);

    uav_search_state_pub_.publish(uav_search_state); // 200hz频率发布无人机状态

}

void uav_search_strategy::mocap_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    Eigen::Vector3d vel = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    std::cout << GREEN << "mocap_vel = " << vel.transpose() << std::endl;
}

void uav_search_strategy::local_vel_ned_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_from_autopilot = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}

void uav_search_strategy::lidar_height_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    lidar_height = msg->range;
    //std::cout << "height = " << lidar_height << std::endl;
}

void uav_search_strategy::detect_id_cb(const std_msgs::Int16::ConstPtr& msg) 
{
    // if(!land) // 当前无人机搜索结束，并且没有落地
    // {
    //     // 收到detect_id话题，判断是不是当前组
    //     yolov8_detect_pose_.boxes[uav_group - 1].x = 9;
    //     yolov8_detect_pose_.boxes[uav_group - 1].y = -6;
    //     std::cout << RED << "检测到目标id为：" << uav_group << " 其坐标为：" << yolov8_detect_pose_.boxes[uav_group - 1].x << "," << yolov8_detect_pose_.boxes[uav_group - 1].y << TAIL << std::endl;
    //     return;
    // }
    // 处理接收到的消息
    if(uav_ready_go && ( (state != 3 && (search_strategy == "h_search")) || 
        (state != 4 && (search_strategy == "v_search")) || 
        (state != 4 && (search_strategy == "v_center_search")) || 
        (state != 4 && (search_strategy == "v_center_search_new"))
        )) // 在覆盖阶段，不需要开启yolo了
    {
        // 如果窗口已满，移除最早的数据 
        if (window.size() == 10) {
            int oldest = window.front();
            window.pop_front();
            positions.pop_front(); // 移除对应的坐标
            // 减少计数
            if (--counts[oldest] == 0) {
                counts.erase(oldest);
            }
        }
        
        // 添加新数据到窗口
        window.push_back(msg->data);
        // 添加新坐标到列表
        positions.push_back(vins_pose);
        // 增加计数
        counts[msg->data]++;
        // yolov8_detect_id = 0;
        // 检查窗口中是否有5次以上是同一个数字
        for (const auto& num : counts) {
            if (num.second >= 2) {
                // 计算平均坐标
                Eigen::Vector3d avg_position = calculateAveragePosition(num.first);
                ROS_INFO("Target %d is detected at average position (%f, %f, %f)", num.first, avg_position.x(), avg_position.y(), avg_position.z());
                yolov8_detect_id = num.first;
                // 将对应的平均坐标存入yolo_detect_pose里面
                if(yolov8_detect_pose_.boxes[yolov8_detect_id - 1].x == 0 && yolov8_detect_pose_.boxes[yolov8_detect_id - 1].y == 0)
                {

                    yolov8_detect_pose_.boxes[yolov8_detect_id - 1].x = avg_position[0] + init_pose[0]; // + init_pose[0] 还原到uav起飞时的坐标系下
                    yolov8_detect_pose_.boxes[yolov8_detect_id - 1].y = avg_position[1] + init_pose[1]; // + init_pose[1] 还原到uav起飞时的坐标系下

                    // 转换为detection消息类型
                    sunray_msgs::detection det;
                    det.id = yolov8_detect_id;
                    det.pose[0] = yolov8_detect_pose_.boxes[yolov8_detect_id - 1].x;
                    det.pose[1] = yolov8_detect_pose_.boxes[yolov8_detect_id - 1].y;
                    det.pose[2] = 0;

                    yolov8_detect_pose_pub_.publish(det); // 发布该靶标的坐标
                    std::cout << RED << "检测到目标id为：" << yolov8_detect_id << " 其坐标为：" << yolov8_detect_pose_.boxes[yolov8_detect_id - 1].x << "," << yolov8_detect_pose_.boxes[yolov8_detect_id - 1].y << TAIL << std::endl;
                }
                // 重置计数器和窗口
                counts.clear();
                window.clear();
                positions.clear();
                break;
            }
        }
    }
    
}



// 计算给定ID的平均坐标
Eigen::Vector3d uav_search_strategy::calculateAveragePosition(int id) 
{
    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    int count = 0;
    auto it = positions.begin();
    for (const auto& w : window) {
        if (w == id) {
            sum += *it;
            ++count;
            ++it; // 移动到下一个坐标
        } else {
            ++it; // 移动到下一个坐标，但不计入平均
        }
    }
    if (count > 0) {
        return sum / count;
    } else {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}

bool uav_search_strategy::is_to_target_pose(Eigen::Vector3d current, Eigen::Vector3d target, double distance_th, bool choice)
{
    // 判断当前点是否到目标点
    Eigen::Vector3d diff;
    diff = target - current;
    if(choice) // 根据距离来判断是否到达目标点
    {
        // std::cout << GREEN << "当前点" << current.transpose() << "到目标点" << target.transpose() << "的距离为" << diff.norm() << std::endl;
        if(diff.norm() < distance_th)   
            return true;
    }
    else // 根据三个轴与目标点的差值来判断是否到了目标点
    {
        if(abs(diff[0]) < 0.2 && abs(diff[1]) < 0.2 && abs(diff[2]) < 0.15)
        {
            // std::cout << "diff ..." << diff.transpose() << std::endl;
            return true;
        }
            
    }
    return false;
}

bool uav_search_strategy::is_out_fence(Eigen::Vector3d current)
{
    if(search_strategy == "h_search") // 横向搜索
    {
        if(state == 3)
        {
            if(current[2] > z_max || current[2] < z_min)
                return true;//在覆盖阶段考虑z轴即可
            return false; 
        }

        // 默认为左边角点起飞
        if(current[0] < x_min || current[0] > x_max 
            || current[1] < y_min || current[1] > y_max
            || current[2] < z_min || current[2] > z_max)
            {
                std::cout << RED << "超出地理围栏: (" << x_min << "," << x_max << ")，(" <<
                y_min << "," << y_max << ")，(" << z_min << "," << z_max << ")" << std::endl;
                return true; // 在电子围栏外
            }
        
        return false;
    }
    else if(search_strategy == "v_search") // 纵向搜索
    {
        if(state == 4)
        {
            if(current[2] > z_max || current[2] < z_min)
                return true;//在覆盖阶段考虑z轴即可
            return false; 
        }
            
        // 默认为左边角点起飞
        if(current[0] < x_min || current[0] > x_max 
            || current[1] < y_min || current[1] > y_max
            || current[2] < z_min || current[2] > z_max)
            {
                std::cout << RED << "超出地理围栏: (" << x_min << "," << x_max << ")，(" <<
                y_min << "," << y_max << ")，(" << z_min << "," << z_max << ")" << std::endl;
                return true; // 在电子围栏外
            }
        
        return false;
    }
    else if(search_strategy == "v_center_search") // 纵向居中搜索
    {
        if(state == 4)
        {
            if(current[2] > z_max || current[2] < z_min)
                return true;//在覆盖阶段考虑z轴即可
            return false; 
        }
            
        // 默认为中点起飞
        if(current[0] < x_min || current[0] > x_max 
            || current[1] > (y_max - y_min) / 4 || current[1] < (-3*(y_max - y_min) / 4 - 1)// y坐标在长条外
            || current[2] < z_min || current[2] > z_max)
            {
                std::cout << RED << "超出地理围栏: (" << x_min << "," << x_max << ")，(" <<
                (-3*(y_max - y_min) / 4 - 1) << "," << (y_max - y_min) / 4 << ")，(" << z_min << "," << z_max << ")" << std::endl;
                return true; // 在电子围栏外
            }
        
        return false;
    }
    else if(search_strategy == "v_center_search_new") // 纵向居左搜索
    {
        if(state == 4)
        {
            if(current[2] > z_max || current[2] < z_min)
                return true;//在覆盖阶段考虑z轴即可
            return false; 
        }
            
        // 默认为居左起飞
        if(current[0] < x_min || current[0] > x_max 
            || current[1] > y_max || current[1] < y_min // y坐标在长条外
            || current[2] < z_min || current[2] > z_max)
            {
                std::cout << RED << "超出地理围栏: (" << x_min << "," << x_max << ")，(" <<
                y_min << "," << y_max << ")，(" << z_min << "," << z_max << ")" << std::endl;
                return true; // 在电子围栏外
            }
        
        return false;
    }
}

bool uav_search_strategy::check_timeout()
{
    if (external_source == sunray_msgs::ExternalOdom::MOCAP)
    {
        if((ros::Time::now() - get_mocap_stamp).toSec()>MOCAP_TIMEOUT){
            cout << RED << "Odom Timeut: [ MOCAP ] " << TAIL << endl;
            return true;
        }
    }
    else if (external_source == sunray_msgs::ExternalOdom::VINS)
    {
        if((ros::Time::now() - get_vins_stamp).toSec()>VINS_TIMEOUT){
            cout << RED << "Odom Timeut: [ VINS ] " << TAIL << endl;
            return true;
        }
    }
    else if (external_source == sunray_msgs::ExternalOdom::GAZEBO)
    {
        if((ros::Time::now() - get_gazebo_stamp).toSec()>GAZEBO_TIMEOUT){
            cout << RED << "Odom Timeut: [ GAZEBO ] " << TAIL << endl;
            return true;
        }
    }
    return false;
}

// 最后没有等待其他无人机搜索完成信号
// 横向搜索
void uav_search_strategy::h_search()
{
    if(state == 1)
    {
        // std::cout << YELLOW << "在横向搜索阶段" << std::endl;
        box_sort_flag = 0;
        // if(!boxes_poses_.empty())
        //     boxes_poses_.clear();
        // 横向搜索阶段
        if(!edges_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_) && stage.data == 0)
            {
                stage.data = 1;
                stage_pub_.publish(stage); //横向搜索，发送一个stage=1(开启搜索)的信号
                target_num = 0;
                std::cout << GREEN << "到达目标点，发布stage=" << stage.data << "申请横向搜索！" << std::endl;
            }
            // std::cout << RED <<  "没有到达目标点，stage=" << stage.data << std::endl;
        }

        if (send == 1)
        {
            
            // 接收可以搜索信号
            if(!edges_poses_.empty())
            {
                if(is_to_target_pose(vins_pose, pos_))
                {
                    if(edges_poses_.size() > 1)
                        edges_poses_.erase(edges_poses_.begin());
                    pos_ = edges_poses_[0];
                }
            }
            if(is_to_target_pose(vins_pose, pos_))
            {
                // std::cout << "vin = " << vins_pose.transpose() << " pose_" << pos_.transpose() << std::endl;
                stage.data = 2;
                stage_pub_.publish(stage); // 发布接收boxes_pose命令
                send = 0;
                std::cout << GREEN <<  "发布stage=" << stage.data << "可以搜索boxes！" << std::endl;
                // edges_poses_.erase(begin());
            }
            // send = 1;
        }
        else if(send == 2) // 有boxes
        {
            state = 2;
            send = 0;
        }
        else if(send == -1) // 没有boxes
        {                     
            // edges_poses_.erase(edges_poses_.begin());
            // 计算下一个edges_pose的坐标
            // double x = ((edges_poses_[0][0] / height_step) + 1 ) * height_step; // 这里的height_step是一个纵向栅格的长度
            float x = (((pos_[0] + back_step) / height_step) + 1) * height_step - back_step; // 这里的height_step是一个纵向栅格的长度
            // std::cout << "double x=" << ((pos_[0] + back_step) / height_step) << "," << ((pos_[0] + back_step) / height_step) + 1  << std::endl;
            float y = (pos_[1] < -step/2) ? -step : 0;

            // 判断x与区域的高的差值是否小于step，如果小于step，那么就不用再向前推进了，直接把edges_poses_中的坐标点跑完就够了
            if((square_height - x) >= 0) 
            {
                // std::cout << GREEN << "继续向前推进！" << x << std::endl;
                // 距离还够往前推进
                Eigen::Vector3d edges_pose(x,y,State1_height);
                edges_poses_.push_back(edges_pose);
                y = (pos_[1] < -step/2) ? 0 : -step; // 得到下一个edge_pose
                edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                edges_poses_.erase(edges_poses_.begin());

                pos_ = edges_poses_[0];
                stage.data = 0;
                send = 0;
            }
            else
            {
                // std::cout << GREEN << "不能向前推进！" << std::endl;
                if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值
                {
                    if(uav_search_state_all)
                    {
                        uav_search_state.data = 1; // 当前无人机搜索状态
                        std::cout << RED << "搜索任务完成，直接land！" << std::endl;
                        pos_[0] = vins_pose[0];
                        pos_[1] = vins_pose[1];
                        pos_[2] = 0.;
                        edges_poses_.clear(); // 清空
                        cmd_.cmd = 3;
                        cmd_pub_.publish(cmd_);
                        mode_ = 0;
                        land = 1;
                        
                    }
                    else
                    {
                        // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                        std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                        uav_search_state.data = 1; // 已经搜索完成
                    }
                }
                else
                {
                    // 整个搜索过程结束，开始覆盖靶标,直接进入state = 3
                    state = 3;
                    edges_poses_.clear();
                }
                // // 直接land，或者其他操作
                // pos_[0] = vins_pose[0];
                // pos_[1] = vins_pose[1];
                // pos_[2] = 0.;
                // edges_poses_.clear(); // 清空
                // cmd_.cmd = 3;
                // cmd_pub_.publish(cmd_);
                // mode_ = 0;
            } 
        }
    }
    else if(state == 2)
    {
        // std::cout << YELLOW << "在搜索boxes阶段" << std::endl;
        if(box_sort_flag == 0)
        {
            // 开始跑航迹点，按照距离远近来跑航迹点
            // 将boxes坐标与自身坐标vins_pose求距离，然后进行排序，
            // 创建一个索引数组
            if(!boxes_poses_.empty())
            {
                std::vector<size_t> indices(boxes_poses_.size());
                // 初始化索引数组
                std::iota(indices.begin(), indices.end(), 0);

                // 根据距离pos_排序索引
                std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                    return (boxes_poses_[a] - pos_).squaredNorm() < (boxes_poses_[b] - pos_).squaredNorm();
                }); 
                target_poses_.clear();
                // 将排序后的坐标存入target_poses_
                for (size_t index : indices) {
                    target_poses_.push_back(boxes_poses_[index]);
                }
                boxes_poses_.clear(); // 每次接收到消息之后，先把上一次存的坐标清空

                box_sort_flag = 1;
                pos_ = target_poses_[0];
                std::cout << RED << "pose_" << pos_.transpose() << std::endl;
            }
            else
            {
                std::cout << RED << "boxes_poses_为空！" << std::endl;
            }
        }
        
        
        if(!target_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_))
            {
                // 开启yolo检测
                // 盘旋1s，
                yolo_detect_timer ++;
                std::cout << YELLOW << "盘旋时间为：" << yolo_detect_timer << std::endl;
                // yolo检测完成,下一个点
                if (yolo_detect_timer == 10) 
                {

                    target_poses_.erase(target_poses_.begin()); // 飞机在靶标上空盘旋1s
                    std::cout << RED << "target_poses_.size = " << target_poses_.size() << TAIL << std::endl;
                    std::cout << GREEN << "时间到了，可以跑下一个点了！" << std::endl;
                    yolo_detect_timer = 0;
                    if(!target_poses_.empty())
                        pos_ = target_poses_[0];
                }
                // 不需要手动移动后面的元素，erase方法会自动完成这一操作
            }
        }
        else // target_pose为空
        {
            // Eigen::Vector3d temp = pos_;
            // std::cout << "pose_pose:" << temp.transpose() << std::endl;
            // edges_poses_.clear();
            // // 计算下一个edges_pose的坐标
            // double x = ((temp[0] / height_step) + 1 ) * height_step; // 这里的height_step是一个纵向栅格的长度
            // std::cout << "double_x=" << x << std::endl;
            std::vector<Eigen::Vector3d> temp = edges_poses_;
            // std::cout << "pose_pose:" << temp.transpose() << std::endl;
            edges_poses_.clear();
            // 计算下一个edges_pose的坐标
            // double x = ((temp[0][0] / height_step) + 1 ) * height_step; // 这里的height_step是一个纵向栅格的长度
            float x = (floor(((pos_[0] + back_step) / height_step))  + 1) * height_step - back_step; // 这里的height_step是一个纵向栅格的长度

            // std::cout << "double_x=" << (pos_[0] + back_step) << "," << floor((pos_[0] + back_step) / height_step) << std::endl;
            float y = pos_[1] < -step/2 ? -step : 0;
            // if(edges_poses_[0][1] < -step/2)
            //     y = -step;
            
            // 判断x与区域的高的差值是否小于1，如果小于1，那么就不用再向前推进了，直接land
            if((square_height - x) >= (/*step + */ 0)) 
            {
                state = 1; // 运行edge_poses
                stage.data = 0;
            
                Eigen::Vector3d edges_pose(x,y,State1_height);
                edges_poses_.push_back(edges_pose);
                y = pos_[1] < -step/2 ? 0 : -step;
                // if(edges_poses_[0][1] < -step/2)
                //     y = 0;
                // else
                //     y = -step;
                // y = (edges_poses_[0][1] < -step/2) : 0 ? ; // 得到下一个edge_pose
                edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                pos_ = edges_poses_[0];
                // state = 1;
                
            }
            else
            {
                // 不能继续向前推进了并且无与自身group相对应靶标坐标，直接land，或者其他操作
                if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值
                {
                    if(uav_search_state_all)
                    {
                        uav_search_state.data = 1; // 当前无人机搜索状态
                        std::cout << RED << "搜索任务完成，直接land！" << std::endl;
                        pos_[0] = vins_pose[0];
                        pos_[1] = vins_pose[1];
                        pos_[2] = 0.;
                        target_poses_.clear(); // 清空
                        cmd_.cmd = 3;
                        cmd_pub_.publish(cmd_);
                        mode_ = 0;
                        land = 1;
      
                    }
                    else
                    {
                        // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                        std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                        uav_search_state.data = 1; // 已经搜索完成
                    }
                }
                else
                {
                    // 整个搜索过程结束，开始覆盖靶标,直接进入state = 3
                    state = 3;
                    target_poses_.clear();
                }
            }

        }
    }
    else if(state == 3)
    {
        std::cout << YELLOW << "在前往目标点阶段" << std::endl;
        // 在下视相机检测时，已经将识别到的靶标坐标转换到uav起飞时的坐标系下，现在无人机取覆盖这个靶标时，需要返回到自己开始搜索时的坐标系下
        pos_ = Eigen::Vector3d(yolov8_detect_pose_.boxes[uav_group - 1].x - init_pose[0], yolov8_detect_pose_.boxes[uav_group - 1].y - init_pose[1], Takeoff_height);
        if(is_to_target_pose(vins_pose, pos_))
        {
            // 直接land
            uav_search_state.data = 1; // 当前无人机搜索状态
            std::cout << RED << "detect pose OK !" << std::endl;
            pos_[0] = vins_pose[0];
            pos_[1] = vins_pose[1];
            pos_[2] = 0.;
            cmd_.cmd = 3;
            cmd_pub_.publish(cmd_);
            land = 1;
            mode_ = 0;
        }
    }
    
    if(edges_poses_.empty() && target_poses_.empty() && yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0
        && uav_search_state_all)
    { // targte_pose 为空，直接land
        uav_search_state.data = 1; // 当前无人机搜索状态
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
        std::cout << RED << "target pose is empty!" << std::endl;
    }
    else if(!is_out_fence(vins_pose) && !land) // 在电子围栏内
    {
        // 速度控制
        if(velocity_ctrl_mode)
        {
            double time = ros::Time::now().toSec();
            double dt = time - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            cmd_.desired_yaw = 0;
            cmd_pub_.publish(cmd_);
            last_time = time;
            // std::cout << RED << "cur_x_vel = " << vel_from_autopilot[0] << " cur_y_vel = " << vel_from_autopilot[1] << " cur_z_vel = " << vel_from_autopilot[2] << std::endl;
            // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
        }
        else // 位置控制
        {
            cmd_.cmd = 4;
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;    
            cmd_.desired_yaw = 0;
            cmd_pub_.publish(cmd_);
        }   
    }
    else // 在电子围栏外
    {
        uav_search_state.data = 1; // 当前无人机搜索状态
        std::cout << RED << "在电子围栏外！" << std::endl;
        // 不能继续向前推进了，直接land，或者其他操作
        pos_[0] = vins_pose[0];
        pos_[1] = vins_pose[1];
        pos_[2] = 0.;
        target_poses_.clear(); // 清空
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
    }
    timer_cnt += 1;
    if(timer_cnt == 10)
    {
        timer_cnt = 0;
        std::cout << YELLOW << "当前点pose:" << std::fixed << std::setprecision(2) <<  vins_pose.transpose() << "-----> 目标点pose:" << std::fixed << std::setprecision(2) << pos_.transpose() << std::endl;
        
        if (!edges_poses_.empty())
            std::cout << YELLOW << "edge_poses:" << std::fixed << std::setprecision(2) << edges_poses_[0].transpose() << std::endl;
        if (!target_poses_.empty())
            std::cout << YELLOW << " target_poses:" << std::fixed << std::setprecision(2) << target_poses_[0].transpose() << std::endl; // 注意这里应该使用target_poses_
        if(!box_pose_buff.empty())
        {
            int i = 0;
            for(auto bbox : box_pose_buff)
            {
                std::cout << RED << " box_pose_buff[" << i << "] = " << std::fixed << std::setprecision(2) << bbox.transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
                i ++;
            }
        }

        // 输出yolov8坐标
        if(yolov8_detect_id > 0)
        {
            std::cout << BLUE << " yolov8_pose:(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].y << ")" << std::endl;
        }
    }
}

// 纵向搜索
void uav_search_strategy::v_search()
{
    if(state == 1) // 调整机头旋转90度
    {
        cmd_.cmd = 4; // 机头转90度
        cmd_.desired_pos[0] = pos_[0];
        cmd_.desired_pos[1] = pos_[1];
        cmd_.desired_pos[2] = pos_[2];
        cmd_.enable_yawRate = false;
        if(timer_cnt == 1)
        {
            cmd_.desired_yaw = max((vins_rpy[2] - M_PI / 12) , -M_PI / 2);
        }
        std::cout << GREEN << "timer_cnt = " << timer_cnt << "s set angle is  " << cmd_.desired_yaw << TAIL << std::endl;

        cmd_pub_.publish(cmd_);

        if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.01)
        {
            state = 2;
            std::cout << RED << "机头转90度完成！" << TAIL << std::endl;
        }
        std::cout << RED << "current_yaw: " << vins_rpy.transpose() << TAIL << std::endl;
    }
    if(state == 2) // 纵向移动
    {
        // std::cout << YELLOW << "在纵向搜索阶段" << TAIL << std::endl;
        box_sort_flag = 0;
        if(!edges_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_) && stage.data == 0)
            {
                stage.data = 1;
                stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                target_num = 0;
                std::cout << GREEN << "到达目标点，发布stage=" << stage.data << "申请纵向搜索！" << std::endl;
            }
        }
        
        if(send == 1)
        {
            // 接收可以搜索信号
            if(!edges_poses_.empty())
            {
                if(is_to_target_pose(vins_pose, pos_))
                {
                    if(edges_poses_.size() > 1)
                    {
                        edges_poses_.erase(edges_poses_.begin());
                    }
                    else
                    {
                        send = -1;
                    }
                    pos_ = edges_poses_[0];
                }
            }
            if(is_to_target_pose(vins_pose, pos_))
            {   
                // std::cout << "vin = " << vins_pose.transpose() << " pose_" << pos_.transpose() << std::endl;
                stage.data = 2;
                stage_pub_.publish(stage); // 发布接收boxes_pose命令
                send = 0;
                std::cout << GREEN <<  "发布stage=" << stage.data << "可以搜索boxes！" << std::endl;
                // edges_poses_.erase(begin());
            }
        }
        else if (send == 2) // 接收到box
        {
            state = 3;
            send = 0;
        }
        else if(send == -1) // 没有boxes
        {   
            float x,y;
            if(v_search_rounds == 0)   // 第一轮前进               
            {
                x = ((pos_[0] / height_step) + 1) * height_step; // 这里的height_step是一个纵向栅格的长度
                y = 0;
            }
            else // 第二轮返回
            {
                x = ((pos_[0] / height_step) - 1) * height_step; // 这里的height_step是一个纵向栅格的长度
                y = -step;
            }

            // 判断x与区域的高的差值是否小于step，如果小于step，那么就不用再向前推进了，直接把edges_poses_中的坐标点跑完就够了
            if((square_height - x) >= 0 && x >= 0) 
            {
                // std::cout << GREEN << "继续向前推进！" << x << std::endl;
                // 距离还够往前推进
                Eigen::Vector3d edges_pose(x,y,State1_height);
                edges_poses_.push_back(edges_pose);
                if(v_search_rounds == 0) // 第一轮前进  
                    x = ((pos_[0] / height_step) + 2) * height_step;
                else // 第二轮返回
                    x = ((pos_[0] / height_step) - 2) * height_step;
                
                if(x >= 0 && x <= square_height) // 避免第二轮返回时，x轴跑出赛道
                    edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));
                
                edges_poses_.erase(edges_poses_.begin()); // 将当前飞机的坐标点删除，并不是计算到的x,y

                pos_ = edges_poses_[0];
                stage.data = 0;
                send = 0;
            }
            else if((square_height - x) < 0 && v_search_rounds == 0) 
            {
                // 第一轮已经到头了，开始返航
                std::cout << GREEN << "第一轮搜索完成，开始返回进行第二轮搜索！" << TAIL << std::endl;
                // 计算edges_poses
                x = pos_[0]; // 这里的height_step是一个纵向栅格的长度
                y = -step;
                edges_poses_.clear();
                edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                x = ((pos_[0] / height_step) - 1) * height_step;
                if(x >= 0)
                    edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                pos_ = edges_poses_[0];
                v_search_rounds = 1;
                stage.data = 0;
                send = 0;
            }
            else if(v_search_rounds == 1 && x < 0)
            {
                std::cout << GREEN << "不能向前推进！" << TAIL << std::endl;

                // // 等待其他无人机信号
                // uav_search_state.data = 1; // 当前无人机搜索状态
                
                if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值
                {
                    if(uav_search_state_all) // 其他无人机已经搜索完成，那么说明该机对应的靶标搜索失败，直接降落
                    {
                        uav_search_state.data = 1; // 当前无人机搜索状态
                        std::cout << RED << "搜索任务完成，直接land！" << TAIL << std::endl;
                        pos_[0] = vins_pose[0];
                        pos_[1] = vins_pose[1];
                        pos_[2] = 0.;
                        edges_poses_.clear(); // 清空
                        cmd_.cmd = 3;
                        cmd_pub_.publish(cmd_);
                        mode_ = 0;
                        land = 1;
                    }
                    else
                    {
                        // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                        std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                        uav_search_state.data = 1; // 已经搜索完成
                    }
                }
                else
                {
                    // 整个搜索过程结束，开始覆盖靶标,直接进入state = 4
                    state = 4;
                    edges_poses_.clear();
                }
                // // 直接land，或者其他操作
                // pos_[0] = vins_pose[0];
                // pos_[1] = vins_pose[1];
                // pos_[2] = 0.;
                // edges_poses_.clear(); // 清空
                // cmd_.cmd = 3;
                // cmd_pub_.publish(cmd_);
                // mode_ = 0;
            } 
        }

    }
    else if(state == 3) // 遍历box
    {
        // std::cout << YELLOW << "在搜索boxes阶段" << TAIL << std::endl;
        if(box_sort_flag == 0)
        {
            // 开始跑航迹点，按照距离远近来跑航迹点
            // 将boxes坐标与自身坐标vins_pose求距离，然后进行排序，
            // 创建一个索引数组
            if(!boxes_poses_.empty())
            {
                std::vector<size_t> indices(boxes_poses_.size());
                // 初始化索引数组
                std::iota(indices.begin(), indices.end(), 0);

                // 根据距离pos_排序索引
                std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                    return (boxes_poses_[a] - pos_).squaredNorm() < (boxes_poses_[b] - pos_).squaredNorm();
                }); 
                target_poses_.clear();
                // 将排序后的坐标存入target_poses_
                for (size_t index : indices) {
                    target_poses_.push_back(boxes_poses_[index]);
                }
                boxes_poses_.clear(); // 每次接收到消息之后，先把上一次存的坐标清空

                box_sort_flag = 1;
                pos_ = target_poses_[0];
                std::cout << RED << "pose_" << pos_.transpose() << TAIL << std::endl;
            }
            else
            {
                std::cout << RED << "boxes_poses_为空！" << TAIL << std::endl;
            }
        }
    
        if(!target_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_))
            {
                // 开启yolo检测
                // 盘旋1s，
                yolo_detect_timer ++;
                std::cout << YELLOW << "盘旋时间为：" << yolo_detect_timer << TAIL << std::endl;
                // yolo检测完成,下一个点
                if (yolo_detect_timer == 10) 
                {

                    target_poses_.erase(target_poses_.begin()); // 飞机在靶标上空盘旋1s
                    std::cout << GREEN << "时间到了，可以跑下一个点了！" << TAIL << std::endl;
                    yolo_detect_timer = 0;
                    if(!target_poses_.empty())
                        pos_ = target_poses_[0];
                }
                // 不需要手动移动后面的元素，erase方法会自动完成这一操作
            }
        }
        else // target_pose为空
        {
            
            std::vector<Eigen::Vector3d> temp = edges_poses_;
            // std::cout << "pose_pose:" << temp.transpose() << TAIL << std::endl;
            edges_poses_.clear();
            // 计算下一个edges_pose的坐标
            float x,y;
            if(v_search_rounds == 0) // 第一轮前进
            {
                x = (floor((pos_[0] / height_step)) + 1) * height_step; // 这里的height_step是一个纵向栅格的长度
                y = 0;
            }
            else // 第二轮返回
            {
                x = (floor((pos_[0] / height_step)) - 0) * height_step; // 这里的height_step是一个纵向栅格的长度
                y = -step;
            }
                
 
            // 判断x与区域的高的差值是否小于1，如果小于1，那么就不用再向前推进了，直接land
            if((square_height - x) >= 0 && x > 0) 
            {
                state = 2; // 运行edge_poses
                stage.data = 0;
            
                Eigen::Vector3d edges_pose(x,y,State1_height);
                edges_poses_.push_back(edges_pose);
                if(v_search_rounds == 0) // 第一轮前进  
                    x = (floor((pos_[0] / height_step)) + 2) * height_step;
                else // 第二轮返回
                    x = (floor((pos_[0] / height_step)) - 1) * height_step;
                
                if(x >= 0 && x <= square_height) // 避免第二轮返回时，x轴跑出赛道
                    edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                pos_ = edges_poses_[0];

            }
            else if((square_height - x) < 0 && v_search_rounds == 0)
            {
                // 第一轮已经到头了，开始返航
                std::cout << GREEN << "第一轮搜索完成，开始返回进行第二轮搜索！" << TAIL << std::endl;
                // 计算edges_poses
                x = pos_[0]; // 这里的height_step是一个纵向栅格的长度
                y = -step;
                edges_poses_.clear();
                edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));
                
                x = (floor((pos_[0] / height_step)) - 1) * height_step;
                if(x >= 0) // 避免第二轮返回时，x轴跑出赛道
                    edges_poses_.push_back(Eigen::Vector3d(x,y,State1_height));

                pos_ = edges_poses_[0];
                v_search_rounds = 1;

                state = 2; // 运行edge_poses
                stage.data = 0;
            }
            else if(v_search_rounds == 1 && x <= 0)
            {
                std::cout << GREEN << "不能向前推进！" << TAIL << std::endl;

                // 等待其他无人机信号

                // 不能继续向前推进了并且无与自身group相对应靶标坐标，直接land，或者其他操作
                if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值，并且其他无人机都搜索完成
                {
                    if(uav_search_state_all) // 其他无人机已经搜索完成，那么说明该机对应的靶标搜索失败，直接降落
                    {
                        uav_search_state.data = 1; // 当前无人机搜索状态
                        std::cout << RED << "搜索任务完成，直接land！" << TAIL << std::endl;
                        pos_[0] = vins_pose[0];
                        pos_[1] = vins_pose[1];
                        pos_[2] = 0.;
                        target_poses_.clear(); // 清空
                        cmd_.cmd = 3;
                        cmd_pub_.publish(cmd_);
                        mode_ = 0;
                        land = 1;
                    }
                    else
                    {
                        // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                        std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                        uav_search_state.data = 1; // 已经搜索完成
                    }


                }
                else
                {
                    // 整个搜索过程结束，开始覆盖靶标,直接进入state = 4
                    state = 4;
                    target_poses_.clear();
                }
            }

        }
    }
    else if(state == 4) // 覆盖靶标   
    {
        std::cout << YELLOW << "在前往目标点阶段" << TAIL << std::endl;
        // 在下视相机检测时，已经将识别到的靶标坐标转换到uav起飞时的坐标系下，现在无人机取覆盖这个靶标时，需要返回到自己开始搜索时的坐标系下
        pos_ = Eigen::Vector3d(yolov8_detect_pose_.boxes[uav_group - 1].x - init_pose[0], yolov8_detect_pose_.boxes[uav_group - 1].y - init_pose[1], Takeoff_height);
        
        if(is_to_target_pose(vins_pose, pos_))
        {
            // 直接land
            uav_search_state.data = 1; // 当前无人机搜索状态
            std::cout << RED << "detect pose OK !" << TAIL << std::endl;
            pos_[0] = vins_pose[0];
            pos_[1] = vins_pose[1];
            pos_[2] = 0.;
            cmd_.cmd = 3;
            cmd_pub_.publish(cmd_);
            land = 1;
            mode_ = 0;
        }
    }

    if(edges_poses_.empty() && target_poses_.empty() && yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0
        && uav_search_state_all)
    { // targte_pose 为空，直接land
        uav_search_state.data = 1; // 当前无人机搜索状态
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
        std::cout << RED << "target pose is empty!" << TAIL << std::endl;
    }
    else if(!is_out_fence(vins_pose) && !land) // 在电子围栏内
    {
        // 速度控制
        if(velocity_ctrl_mode)
        {
            double time = ros::Time::now().toSec();
            double dt = time - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            if(state != 1)
                cmd_.desired_yaw = -M_PI / 2;
            cmd_pub_.publish(cmd_);
            last_time = time;
            // std::cout << RED << "cur_x_vel = " << vel_from_autopilot[0] << " cur_y_vel = " << vel_from_autopilot[1] << " cur_z_vel = " << vel_from_autopilot[2] << std::endl;
            // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
        }
        else // 位置控制
        {
            cmd_.cmd = 4;
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            if(state != 1)
                cmd_.desired_yaw = -M_PI / 2;
            cmd_pub_.publish(cmd_);
        }   
    }
    else // 在电子围栏外
    {
        uav_search_state.data = 1; // 当前无人机搜索状态
        std::cout << RED << "在电子围栏外！" << TAIL << std::endl;
        // 不能继续向前推进了，直接land，或者其他操作
        pos_[0] = vins_pose[0];
        pos_[1] = vins_pose[1];
        pos_[2] = 0.;
        target_poses_.clear(); // 清空
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
    }
    timer_cnt += 1;
    if(timer_cnt == 10)
    {
        timer_cnt = 0;
        std::cout << YELLOW << "当前点pose:" << std::fixed << std::setprecision(2) <<  vins_pose.transpose() << "-----> 目标点pose:" << std::fixed << std::setprecision(2) << pos_.transpose() << TAIL << std::endl;
        
        if (!edges_poses_.empty())
            std::cout << YELLOW << "edge_poses:" << std::fixed << std::setprecision(2) << edges_poses_[0].transpose() << TAIL << std::endl;
        if (!target_poses_.empty())
            std::cout << YELLOW << " target_poses:" << std::fixed << std::setprecision(2) << target_poses_[0].transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
        if(!box_pose_buff.empty())
        {
            int i = 0;
            for(auto bbox : box_pose_buff)
            {
                std::cout << RED << " box_pose_buff[" << i << "] = " << std::fixed << std::setprecision(2) << bbox.transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
                i ++;
            }
        }
        // 输出yolov8坐标
        if(yolov8_detect_id > 0)
        {
            std::cout << BLUE << " yolov8_pose:(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].y << ")" << TAIL << std::endl;
        }
    }

}


void uav_search_strategy::v_center_search()
{
    
    if(state == 1) // 纵向移动
    {
        if(is_to_target_pose(vins_pose, pos_) && stage.data == 0)
        {
            stage.data = 1;
            stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
            target_num = 0;
            send = 0;
            std::cout << GREEN << "到达目标点，发布stage=" << stage.data << "申请纵向搜索！" << std::endl;
            // std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
            // edges_poses_.erase(--edges_poses_.end());
            
        }
        if(send == 1)
        {
            if(v_search_rounds == 0) // 第一回合
            {
                // 先判断是否有box可以跑
                if(!boxes_poses_.empty())
                {
                    state = 3;
                    box_sort_flag = 0;
                }
                else // 没有box可以跑，则跑edge_pose
                {
                    // 接收可以搜索信号
                    if(!edges_poses_.empty())
                    {
                        if(is_to_target_pose(vins_pose, pos_))
                        {
                            if(edges_poses_.size() > 1)
                            {
                                std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
                                edges_poses_.erase(--edges_poses_.end());
                                pos_ = edges_poses_[edges_poses_.size() - 1]; // 取最后一个
                            }
                            else // 只有一个
                            {
                                state = 5;
                                std::cout << RED << "state=1处到达第一回合的终点！开始旋转..." << TAIL << std::endl;
                                edges_poses_.clear();
                            }
                        }
                    }
                    else
                    {
                        state = 5;
                        std::cout << RED << "state=1处到达第一回合的终点！开始旋转90°..." << TAIL << std::endl;
                    }
                }
            }
            else // 第二回合
            {
                // 先判断是否有box可以跑
                if(!boxes_poses_.empty())
                {
                    state = 3;
                    box_sort_flag = 0;
                }
                else
                {
                    if(!edges_poses_.empty())
                    {
                        if(is_to_target_pose(vins_pose, pos_))
                        {
                            if(edges_poses_.size() > 1)
                            {
                                std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
                                edges_poses_.erase(--edges_poses_.end());
                                pos_ = edges_poses_[edges_poses_.size() - 1];
                            }
                            else // 只有一个
                            {
                                if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值
                                {
                                    if(uav_search_state_all) // 其他无人机已经搜索完成，那么说明该机对应的靶标搜索失败，直接降落
                                    {
                                        uav_search_state.data = 1; // 当前无人机搜索状态
                                        std::cout << RED << "搜索任务完成，直接land！" << TAIL << std::endl;
                                        pos_[0] = vins_pose[0];
                                        pos_[1] = vins_pose[1];
                                        pos_[2] = 0.;
                                        edges_poses_.clear(); // 清空
                                        cmd_.cmd = 3;
                                        cmd_pub_.publish(cmd_);
                                        mode_ = 0;
                                        land = 1;
                                    }
                                    else
                                    {
                                        // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                                        std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                                        uav_search_state.data = 1; // 已经搜索完成
                                    }
                                }
                                else
                                {
                                    // 整个搜索过程结束，开始覆盖靶标,直接进入state = 4
                                    state = 4;
                                    edges_poses_.clear();
                                }
                            }
                        }
                    }
                }
                
            }
        }
    }
    else if(state == 2) // 横向搜索
    {
        // 先判断是否有box可以跑
        if(!boxes_poses_.empty())
        {
            state = 3;
            box_sort_flag = 0;
        }
        else
        {
            if(edges_poses_.empty())
            {
                // 添加pose
                pos_[1] = -square_width / 2;
                edges_poses_.push_back(pos_);
            }
            if(is_to_target_pose(vins_pose, pos_))
            {
                state = 6;
                std::cout << RED << "state=2处到达横向搜索的终点！开始旋转90°..." << TAIL << std::endl;
                edges_poses_.erase(edges_poses_.begin());
            }
        }
   
    }

    else if(state == 3) // 遍历box
    {
        // std::cout << YELLOW << "在搜索boxes阶段" << TAIL << std::endl;
        if(box_sort_flag == 0)
        {
            // 开始跑航迹点，按照距离远近来跑航迹点
            // 将boxes坐标与自身坐标vins_pose求距离，然后进行排序，
            // 创建一个索引数组
            if(!boxes_poses_.empty())
            {
                std::vector<size_t> indices(boxes_poses_.size());
                // 初始化索引数组
                std::iota(indices.begin(), indices.end(), 0);

                // 根据距离pos_排序索引
                std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                    return (boxes_poses_[a] - vins_pose).squaredNorm() < (boxes_poses_[b] - vins_pose).squaredNorm();
                }); 
                target_poses_.clear();
                // 将排序后的坐标存入target_poses_
                for (size_t index : indices) {
                    target_poses_.push_back(boxes_poses_[index]);
                }
                boxes_poses_.clear(); // 每次接收到消息之后，先把上一次存的坐标清空

                box_sort_flag = 1;
                pos_ = target_poses_[0];
                std::cout << RED << "pose_" << pos_.transpose() << TAIL << std::endl;
            }
            else
            {
                std::cout << RED << "boxes_poses_为空！" << TAIL << std::endl;
            }
        }
    
        if(!target_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_))
            {
                // 开启yolo检测
                // 盘旋1s，
                yolo_detect_timer ++;
                std::cout << YELLOW << "盘旋时间为：" << yolo_detect_timer << TAIL << std::endl;
                // yolo检测完成,下一个点
                if (yolo_detect_timer == 10) 
                {
                    target_poses_.erase(target_poses_.begin()); // 飞机在靶标上空盘旋1s
                    std::cout << GREEN << "时间到了，可以跑下一个点了！" << TAIL << std::endl;
                    yolo_detect_timer = 0;
                    if(!target_poses_.empty())
                        pos_ = target_poses_[0];
                }
                // 不需要手动移动后面的元素，erase方法会自动完成这一操作
            }
        }
        else // target_pose为空
        {
            if(fabs(vins_rpy[2]) < 0.1) // 在state = 1阶段接收到box
            {
                if(vins_pose[0] > (square_height - height_step))
                {
                    pos_[0] = square_height - height_step;
                    pos_[1] = vins_pose[1] < 0 ? pos_[1] : 0; 
                }
                else
                {
                    // pos_[0] = ;
                    pos_[1] = 0;
                    state = 1;
                    target_num = 0;
                    edges_poses_.push_back(pos_);
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                }
                if(is_to_target_pose(vins_pose, pos_) && pos_[0] == (square_height - height_step))
                {
                    state = 5;
                    std::cout << RED << "state=3处到达第一回合的终点！开始旋转..." << TAIL << std::endl;
                }
            }
            else if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.1) // 在state = 2阶段接收到box
            {
                pos_[1] = -square_width / 2;
                if(is_to_target_pose(vins_pose, pos_))
                {
                    state = 6;
                    std::cout << RED << "state=3处到达横向搜索的终点！开始旋转90°..." << TAIL << std::endl;
                }
            }
            else if(v_search_rounds == 1) // 在返回阶段接收到box
            {
                if(vins_pose[0] > height_step)
                {
                    pos_[1] = -square_width / 2; 
                    edges_poses_.push_back(pos_);
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                }
                else
                {
                    edges_poses_.clear(); // 替换原来的第二回合终点
                    edges_poses_.push_back(pos_);
                    state = 1;
                    target_num = 0;
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                }

                if(is_to_target_pose(vins_pose, pos_) && pos_[0] > height_step)
                {
                    state = 1;
                    target_num = 0;
                    std::cout << RED << "state=3处回到中线上，继续搜索..." << TAIL << std::endl;
                }
            }

        }
    }
    else if(state == 4) // 覆盖靶标   
    {
        std::cout << YELLOW << "在前往目标点：" << pos_.transpose() << TAIL << std::endl;
        // 在下视相机检测时，已经将识别到的靶标坐标转换到uav起飞时的坐标系下，现在无人机取覆盖这个靶标时，需要返回到自己开始搜索时的坐标系下
        pos_ = Eigen::Vector3d(yolov8_detect_pose_.boxes[uav_group - 1].x - init_pose[0], yolov8_detect_pose_.boxes[uav_group - 1].y - init_pose[1], Takeoff_height);
        
        if(is_to_target_pose(vins_pose, pos_))
        {
            // 直接land
            uav_search_state.data = 1; // 当前无人机搜索状态
            std::cout << RED << "detect pose OK !" << TAIL << std::endl;
            pos_[0] = vins_pose[0];
            pos_[1] = vins_pose[1];
            pos_[2] = 0.;
            cmd_.cmd = 3;
            cmd_pub_.publish(cmd_);
            land = 1;
            mode_ = 0;
        }
    }

    else if(state == 5) // 调整机头旋转90度
    {
        if(velocity_ctrl_mode)
        {
            double time = ros::Time::now().toSec();
            double dt = time - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            
            last_time = time;
        }
        else
        {
            cmd_.cmd = 4; // 机头转90度
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
        }
        
        if(timer_cnt == 1)
        {
            cmd_.desired_yaw = max((vins_rpy[2] - M_PI / 12) , -M_PI / 2);
        }
        std::cout << GREEN << "timer_cnt = " << timer_cnt << "s set angle is  " << cmd_.desired_yaw << TAIL << std::endl;

        cmd_pub_.publish(cmd_);

        if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.01)
        {
            state = 2;
            edges_poses_.clear();
            target_poses_.clear();
            boxes_poses_.clear();
            std::cout << RED << "机头转90度完成！开始横向搜索..." << TAIL << std::endl;
        }
        std::cout << RED << "current_yaw: " << vins_rpy.transpose() << TAIL << std::endl;
    }
    else if(state == 6) // 调整机头旋转90度
    {
        if(velocity_ctrl_mode)
        {
            double time = ros::Time::now().toSec();
            double dt = time - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            
            last_time = time;
        }
        else
        {
            cmd_.cmd = 4; // 机头转90度
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
        }
        
        if(timer_cnt == 1)
        {
            cmd_.desired_yaw = max((vins_rpy[2] - M_PI / 12) , -M_PI);
        }
        std::cout << GREEN << "timer_cnt = " << timer_cnt << "s set angle is  " << cmd_.desired_yaw << TAIL << std::endl;

        cmd_pub_.publish(cmd_);

        if(fabs(vins_rpy[2] - (-M_PI)) < 0.01)
        {
            state = 1;
            v_search_rounds = 1;
            edges_poses_.clear();
            target_poses_.clear();
            boxes_poses_.clear();
            target_num = 0;
            pos_[0] = height_step;
            edges_poses_.push_back(pos_);
            std::cout << RED << "机头转180度完成！开始进行第二回合搜索..." << TAIL << std::endl;
        }
        std::cout << RED << "current_yaw: " << vins_rpy.transpose() << TAIL << std::endl;
    }


    if(edges_poses_.empty() && target_poses_.empty() && yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0
        && uav_search_state_all)
    { // targte_pose 为空，直接land
        uav_search_state.data = 1; // 当前无人机搜索状态
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
        std::cout << RED << "target pose is empty!" << TAIL << std::endl;
    }
    else if(!is_out_fence(vins_pose) && !land) // 在电子围栏内
    {
        if(state != 5 && state != 6)
        {
            // 速度控制
            if(velocity_ctrl_mode)
            {
                double time = ros::Time::now().toSec();
                double dt = time - last_time;
                Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
                cmd_.cmd = 6; // 速度控制
                cmd_.desired_vel[0] = vel[0];
                cmd_.desired_vel[1] = vel[1];
                cmd_.desired_vel[2] = vel[2];
                // cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                
                if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.1) // 在横向搜索阶段
                {
                    cmd_.desired_yaw = -M_PI / 2;
                    // std::cout << RED << "set angle is " << -M_PI / 2 << TAIL << std::endl;
                }
                else if(fabs(vins_rpy[2] - (-M_PI)) < 0.1 || v_search_rounds == 1)
                {
                    cmd_.desired_yaw = -M_PI;
                    // std::cout << RED << "set angle is " << -M_PI << TAIL << std::endl;
                }
                else
                {
                    cmd_.desired_yaw = 0;
                    // std::cout << RED << "set angle is " << 0 << TAIL << std::endl;
                }
                
                cmd_pub_.publish(cmd_);
                last_time = time;
                // std::cout << RED << "cur_x_vel = " << vel_from_autopilot[0] << " cur_y_vel = " << vel_from_autopilot[1] << " cur_z_vel = " << vel_from_autopilot[2] << std::endl;
                // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
            }
            else // 位置控制
            {
                cmd_.cmd = 4;
                cmd_.desired_pos[0] = pos_[0];
                cmd_.desired_pos[1] = pos_[1];
                cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                if(state != 1)
                    cmd_.desired_yaw = -M_PI / 2;
                cmd_pub_.publish(cmd_);
            }   
        }
    }
    else // 在电子围栏外
    {
        uav_search_state.data = 1; // 当前无人机搜索状态
        std::cout << RED << "在电子围栏外！" << TAIL << std::endl;
        // 不能继续向前推进了，直接land，或者其他操作
        pos_[0] = vins_pose[0];
        pos_[1] = vins_pose[1];
        pos_[2] = 0.;
        target_poses_.clear(); // 清空
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
    }
    timer_cnt += 1;
    if(timer_cnt == 10)
    {
        timer_cnt = 0;
        std::cout << RED << "num = " << target_num << TAIL << std::endl;
        std::cout << YELLOW << "当前点pose:" << std::fixed << std::setprecision(2) <<  vins_pose.transpose() << "-----> 目标点pose:" << std::fixed << std::setprecision(2) << pos_.transpose() << TAIL << std::endl;
        
        if (!edges_poses_.empty())
            std::cout << YELLOW << "edge_poses:" << std::fixed << std::setprecision(2) << edges_poses_[0].transpose() << TAIL << std::endl;
        if (!target_poses_.empty())
            std::cout << YELLOW << " target_poses:" << std::fixed << std::setprecision(2) << target_poses_[0].transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
        if(!box_pose_buff.empty())
        {
            int i = 0;
            for(auto bbox : box_pose_buff)
            {
                std::cout << RED << " box_pose_buff[" << i << "] = " << std::fixed << std::setprecision(2) << bbox.transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
                i ++;
            }
        }
            
        // 输出yolov8坐标
        if(yolov8_detect_id > 0)
        {
            std::cout << BLUE << " yolov8_pose:(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[0].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[1].y << ")" << ",(" << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[2].y << ")" << TAIL << std::endl;
        }
    }

}

void uav_search_strategy::v_center_search_new()
{
    if(is_to_target_pose(vins_pose, pos_) && stage.data == 0)
    {
        // 开始旋转90度
        state = 5;
        std::cout << RED << "开始旋转90°..." << TAIL << std::endl;

        // std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
        // edges_poses_.erase(--edges_poses_.end());
        // pos_ = edges_poses_[edges_poses_.size() - 1];
    }
    // if(send == 1 && state == 0)
    // {
    //     // 开始旋转90度
    //     state = 5;
    //     std::cout << RED << "开始旋转90°..." << TAIL << std::endl;
    // }
    // 发布stage=1
    if(is_to_target_pose(vins_pose, pos_) && (state != 3 && uav_state == 1 ) && stage.data == 2)
    {
        stage.data = 1;
        stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
        std::cout << GREEN << "send stage = 1" << TAIL << std::endl;
    }

    if(state == 1 && send == 1  && !land) // 纵向移动
    {
        if(v_search_rounds == 0) // 第一回合
        {
            // 先判断是否有box可以跑
            if(!boxes_poses_.empty())
            {
                state = 3;
                box_sort_flag = 0;
                uav_state = 1;
            }
            else // 没有box可以跑，则跑edge_pose
            {
                // 接收可以搜索信号
                if(!edges_poses_.empty())
                {
                    if(is_to_target_pose(vins_pose, pos_))
                    {
                        if(edges_poses_.size() > 1)
                        {
                            edges_poses_.erase(--edges_poses_.end());
                            pos_ = edges_poses_[edges_poses_.size() - 1]; // 取最后一个
                        }
                        else // 只有一个
                        {
                            state = 2;
                            std::cout << RED << "state=1处到达第一回合的终点！开始横向搜索..." << TAIL << std::endl;
                            edges_poses_.clear();
                        }
                    }
                }
                else // 为空，到达第一回合终点，向右移动
                {
                    state = 2;
                    std::cout << RED << "state=1处到达第一回合的终点！开始横向搜索..." << TAIL << std::endl;
                }
            }
        }
        else // 第二回合
        {
            // 先判断是否有box可以跑
            if(!boxes_poses_.empty())
            {
                state = 3;
                box_sort_flag = 0;
                uav_state = 3;
            }
            else
            {
                if(!edges_poses_.empty())
                {
                    if(is_to_target_pose(vins_pose, pos_))
                    {
                        if(edges_poses_.size() > 1)
                        {
                            std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
                            edges_poses_.erase(--edges_poses_.end());
                            pos_ = edges_poses_[edges_poses_.size() - 1];
                        }
                        else // 只有一个
                        {
                            if(yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0) // 为初始值
                            {
                                if(uav_search_state_all) // 其他无人机已经搜索完成，那么说明该机对应的靶标搜索失败，直接降落
                                {
                                    uav_search_state.data = 1; // 当前无人机搜索状态
                                    std::cout << RED << "搜索任务完成，直接land！" << TAIL << std::endl;
                                    pos_[0] = vins_pose[0];
                                    pos_[1] = vins_pose[1];
                                    pos_[2] = 0.;
                                    edges_poses_.clear(); // 清空
                                    cmd_.cmd = 3;
                                    cmd_pub_.publish(cmd_);
                                    mode_ = 0;
                                    land = 1;
                                }
                                else
                                {
                                    // 其他无人机还在搜索，那么需要等到其他无人机搜索完成后再开始该机的搜索
                                    std::cout << RED << "其他无人机还在搜索，等待中..." << TAIL << std::endl;
                                    uav_search_state.data = 1; // 已经搜索完成
                                }
                            }
                            else
                            {
                                // 整个搜索过程结束，开始覆盖靶标,直接进入state = 4
                                state = 4;
                                edges_poses_.clear();
                            }
                        }
                    }
                }
            }
            
        }
    }
    else if(state == 2 && !land) // 横向搜索
    {
        // 先判断是否有box可以跑
        if(!boxes_poses_.empty())
        {
            state = 3;
            box_sort_flag = 0;
            uav_state = 2;
        }
        else
        {
            if(edges_poses_.empty())
            {
                // 添加pose
                pos_[1] = -square_width / 2;
                edges_poses_.push_back(pos_);
            }
            if(is_to_target_pose(vins_pose, pos_))
            {
                state = 1;
                v_search_rounds = 1;
                edges_poses_.clear();
                target_poses_.clear();
                boxes_poses_.clear();
                pose_buff_clear();
                // for(auto& vec:filter_pose_buff)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                // for(auto& vec:pose_buff_mean)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                std::cout << RED << "box 清空" << TAIL << std::endl;
                target_num = 0;
                pos_[0] = 3;
                edges_poses_.push_back(pos_);
                std::cout << RED << "到达横向搜索终点！开始进行第二回合搜索..." << TAIL << std::endl;
            }
        }
   
    }

    else if(state == 3 && !land) // 遍历box
    {
        // std::cout << YELLOW << "在搜索boxes阶段" << TAIL << std::endl;
        if(box_sort_flag == 0)
        {
            // 发送一个stage=2
            stage.data = 2;
            stage_pub_.publish(stage); // 发送一个stage=2(清除box)的信号，为下一次检测做准备
            // 开始跑航迹点，按照距离远近来跑航迹点
            // 将boxes坐标与自身坐标vins_pose求距离，然后进行排序，
            // 创建一个索引数组
            if(!boxes_poses_.empty())
            {
                std::vector<size_t> indices(boxes_poses_.size());
                // 初始化索引数组
                std::iota(indices.begin(), indices.end(), 0);

                // 根据距离pos_排序索引
                std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                    return (boxes_poses_[a] - vins_pose).squaredNorm() < (boxes_poses_[b] - vins_pose).squaredNorm();
                }); 
                target_poses_.clear();
                // 将排序后的坐标存入target_poses_
                for (size_t index : indices) {
                    target_poses_.push_back(boxes_poses_[index]);
                }
                boxes_poses_.clear(); // 每次接收到消息之后，先把上一次存的坐标清空
                // for(auto& vec:filter_pose_buff)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                // for(auto& vec:pose_buff_mean)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                std::cout << RED << "box 清空" << TAIL << std::endl;

                box_sort_flag = 1;
                pos_ = target_poses_[0];
                std::cout << RED << "pose_" << pos_.transpose() << TAIL << std::endl;
            }
            else
            {
                std::cout << RED << "boxes_poses_为空！" << TAIL << std::endl;
            }
        }
    
        if(!target_poses_.empty())
        {
            if(is_to_target_pose(vins_pose, pos_))
            {
                // 开启yolo检测
                // 盘旋1s，
                yolo_detect_timer ++;
                std::cout << YELLOW << "盘旋时间为：" << yolo_detect_timer << TAIL << std::endl;
                // yolo检测完成,下一个点
                if (yolo_detect_timer == 20) 
                {
                    target_poses_.erase(target_poses_.begin()); // 飞机在靶标上空盘旋1s
                    std::cout << GREEN << "时间到了，可以跑下一个点了！" << TAIL << std::endl;
                    yolo_detect_timer = 0;
                    if(!target_poses_.empty())
                        pos_ = target_poses_[0];
                }
                // 不需要手动移动后面的元素，erase方法会自动完成这一操作
            }
        }
        else // target_pose为空
        {
            target_poses_.clear();
            boxes_poses_.clear();
            pose_buff_clear();

            // // 发送一个stage=1
            if(stage.data != 2)
            {
                stage.data = 2;
                stage_pub_.publish(stage); // 发送一个stage=2(清除box)的信号，为下一次检测做准备
            }

            if(uav_state == 1) // 在state = 1阶段接收到box
            {
                if(vins_pose[0] > (square_height - 3))
                {
                    pos_[0] = square_height - 3;
                    pos_[1] = vins_pose[1] < 0 ? pos_[1] : 0; 
                }
                else
                {
                    // pos_[0] = ;
                    pos_[1] = 0;
                    state = 1;
                    target_num = 0;
                    edges_poses_.push_back(pos_);
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                    // stage.data = 1;
                    // stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                }
                if(is_to_target_pose(vins_pose, pos_) && pos_[0] == (square_height - 3))
                {
                    state = 2;
                    // stage.data = 1;
                    // stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                    std::cout << RED << "state=1处到达第一回合的终点！开始横向移动..." << TAIL << std::endl;
                }
            }
            else if(uav_state == 2) // 在state = 2阶段接收到box
            {
                pos_[1] = -square_width / 2;
                if(is_to_target_pose(vins_pose, pos_))
                {
                    state = 1;
                    v_search_rounds = 1;
                    edges_poses_.clear();
                    target_poses_.clear();
                    boxes_poses_.clear();
                    // for(auto& vec:filter_pose_buff)
                    // {
                    //     for(auto & v:vec)
                    //     {
                    //         v.setZero();
                    //     }
                    // }
                    // for(auto& vec:pose_buff_mean)
                    // {
                    //     for(auto & v:vec)
                    //     {
                    //         v.setZero();
                    //     }
                    // }
                    std::cout << RED << "box 清空" << TAIL << std::endl;
                    target_num = 0;
                    pos_[0] = 3;
                    edges_poses_.push_back(pos_);
                    // stage.data = 1;
                    // stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                    std::cout << RED << "到达横向搜索终点！开始进行第二回合搜索..." << TAIL << std::endl;
                }
            }
            else if( uav_state == 3) // 在返回阶段接收到box
            {
                if(vins_pose[0] > 3)
                {
                    pos_[1] = -square_width / 2; 
                    edges_poses_.push_back(pos_);
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                }
                else
                {
                    edges_poses_.clear(); // 利用最后一个box的坐标替换原来的第二回合终点
                    edges_poses_.push_back(pos_);
                    state = 1;
                    target_num = 0;
                    std::cout << BLUE << "增加坐标：" << pos_.transpose() << TAIL << std::endl;
                }

                if(is_to_target_pose(vins_pose, pos_) && pos_[0] > 3)
                {
                    state = 1;
                    target_num = 0;
                    // stage.data = 1;
                    // stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                    std::cout << RED << "state=3处回到中线上，继续搜索..." << TAIL << std::endl;
                }
            }

        }
    }
    else if(state == 4 && !land) // 覆盖靶标   
    {
        std::cout << YELLOW << "在前往目标点：" << pos_.transpose() << TAIL << std::endl;
        // 在下视相机检测时，已经将识别到的靶标坐标转换到uav起飞时的坐标系下，现在无人机取覆盖这个靶标时，需要返回到自己开始搜索时的坐标系下
        pos_ = Eigen::Vector3d(yolov8_detect_pose_.boxes[uav_group - 1].x - init_pose[0], yolov8_detect_pose_.boxes[uav_group - 1].y - init_pose[1], Takeoff_height);
        
        if(is_to_target_pose(vins_pose, pos_))
        {
            yolo_detect_timer ++;
            std::cout << GREEN << "到达目标点，准备降落..." << TAIL << std::endl;
            if(yolo_detect_timer == 10)
            {
                yolo_detect_timer = 0;
                // 直接land
                uav_search_state.data = 1; // 当前无人机搜索状态
                std::cout << RED << "detect pose OK !" << TAIL << std::endl;
                pos_[0] = vins_pose[0];
                pos_[1] = vins_pose[1];
                pos_[2] = 0.;
                cmd_.cmd = 3;
                cmd_pub_.publish(cmd_);
                land = 1;
                mode_ = 0;
            }
        }
    }

    else if(state == 5 && !land) // 调整机头旋转90度
    {
        if(velocity_ctrl_mode)
        {
            double time_ = ros::Time::now().toSec();
            double dt = time_ - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            
            last_time = time_;
        }
        else
        {
            cmd_.cmd = 4; // 机头转90度
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
        }
        
        if(timer_cnt % 10 == 0) // 隔1s设置一个角度
        {
            cmd_.desired_yaw = max((vins_rpy[2] - M_PI / 24) , -M_PI / 2);
        }
        // std::cout << GREEN << "timer_cnt = " << timer_cnt << "s set angle is  " << cmd_.desired_yaw << TAIL << std::endl;

        cmd_pub_.publish(cmd_);


        if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.01)
        {
            // std::cout << BLUE << "删除坐标：" << edges_poses_[edges_poses_.size() - 1].transpose() << TAIL << std::endl;
            // edges_poses_.erase(--edges_poses_.end());
            // pos_ = edges_poses_[edges_poses_.size() - 1];
            // std::cout << RED << "机头转90度完成！悬停2s..." << TAIL << std::endl;
            yolo_detect_timer ++;
            if(yolo_detect_timer == 20)
            {
                state = 1;
                yolo_detect_timer = 0;
                target_poses_.clear();
                boxes_poses_.clear();
                // for(auto& vec:filter_pose_buff)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                // for(auto& vec:pose_buff_mean)
                // {
                //     for(auto & v:vec)
                //     {
                //         v.setZero();
                //     }
                // }
                std::cout << RED << "box 清空" << TAIL << std::endl;

                box_pose_buff.clear();
                stage.data = 1;
                stage_pub_.publish(stage); //纵向搜索，发送一个stage=1(开启搜索)的信号
                target_num = 0;
                std::cout << GREEN << "悬停时间到，发布stage=" << stage.data << "申请纵向搜索！" << std::endl;
            }
        }
        std::cout << RED << "current_yaw: " << vins_rpy.transpose() << TAIL << std::endl;
    }

    if(edges_poses_.empty() && target_poses_.empty() && yolov8_detect_pose_.boxes[uav_group - 1].x == 0 && yolov8_detect_pose_.boxes[uav_group - 1].y == 0
        && uav_search_state_all && !land)
    { // targte_pose 为空，直接land
        uav_search_state.data = 1; // 当前无人机搜索状态
        pos_[0] = vins_pose[0];
        pos_[1] = vins_pose[1];
        pos_[2] = 0.;
        cmd_.cmd = 3;
        cmd_pub_.publish(cmd_);
        mode_ = 0;
        land = 1;
        std::cout << RED << "target pose is empty!" << TAIL << std::endl;
    }
    else if(!is_out_fence(vins_pose) && !land) // 在电子围栏内
    {
        if(state != 5)
        {
            // 速度控制
            if(velocity_ctrl_mode)
            {
                double time = ros::Time::now().toSec();
                double dt = time - last_time;
                Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
                cmd_.cmd = 6; // 速度控制
                cmd_.desired_vel[0] = vel[0];
                cmd_.desired_vel[1] = vel[1];
                cmd_.desired_vel[2] = vel[2];
                // cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                cmd_.desired_yaw = 0;
                if(fabs(vins_rpy[2] - (-M_PI / 2)) < 0.1) // 在横向搜索阶段
                {
                    cmd_.desired_yaw = -M_PI / 2;
                    // std::cout << RED << "set angle is " << -M_PI / 2 << TAIL << std::endl;
                }

                cmd_pub_.publish(cmd_);
                last_time = time;
                // std::cout << RED << "cur_x_vel = " << vel_from_autopilot[0] << " cur_y_vel = " << vel_from_autopilot[1] << " cur_z_vel = " << vel_from_autopilot[2] << std::endl;
                // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
            }
            else // 位置控制
            {
                cmd_.cmd = 4;
                cmd_.desired_pos[0] = pos_[0];
                cmd_.desired_pos[1] = pos_[1];
                cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                cmd_.desired_yaw = -M_PI / 2;
                cmd_pub_.publish(cmd_);
            }   
        }
    }
    else // 在电子围栏外
    {
        if(!land)
        {
            uav_search_state.data = 1; // 当前无人机搜索状态
            std::cout << RED << "在电子围栏外！" << TAIL << std::endl;
            // 不能继续向前推进了，直接land，或者其他操作
            pos_[0] = vins_pose[0];
            pos_[1] = vins_pose[1];
            pos_[2] = 0.;
            target_poses_.clear(); // 清空
            cmd_.cmd = 3;
            cmd_pub_.publish(cmd_);
            mode_ = 0;
            land = 1;
        }
    }
    timer_cnt += 1;

    if(timer_cnt == 20)
    {
        timer_cnt = 0;
        std::cout << RED << "num = " << target_num << TAIL << std::endl;
        std::cout << YELLOW << "当前点pose:" << std::fixed << std::setprecision(2) <<  vins_pose.transpose() << "-----> 目标点pose:" << std::fixed << std::setprecision(2) << pos_.transpose() << TAIL << std::endl;
        
        if (!edges_poses_.empty())
            std::cout << YELLOW << "edge_poses:" << std::fixed << std::setprecision(2) << edges_poses_[0].transpose() << TAIL << std::endl;
        if (!target_poses_.empty())
            std::cout << YELLOW << " target_poses:" << std::fixed << std::setprecision(2) << target_poses_[0].transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
        if(!box_pose_buff.empty())
        {
            int i = 0;
            for(auto bbox : box_pose_buff)
            {
                std::cout << RED << " box_pose_buff[" << i << "] = " << std::fixed << std::setprecision(2) << bbox.transpose() << TAIL << std::endl; // 注意这里应该使用target_poses_
                i ++;
            }
        }
            
        // 输出yolov8坐标
        if(!yolov8_detect_pose_.boxes.empty())
        {
            int i = 0;
            for(i=0; i<3;i++)
            {
                if(yolov8_detect_pose_.boxes[i].x == 0 && yolov8_detect_pose_.boxes[i].y == 0)
                    continue;
                std::cout << BLUE << " yolov8_pose[" << i << "] = " << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[i].x << "," << std::fixed << std::setprecision(2) << yolov8_detect_pose_.boxes[i].y << ")" << TAIL << std::endl; // 注意这里应该使用target_poses_
            }
        }
    }

}


// 搜索开始前的服务函数
void uav_search_strategy::ready_go()
{
    // 根据uav_id来决定搜索时，负责哪一块区域
    if(!uav_ready_go)
    {
        pos_ = Eigen::Vector3d(service_height, (3 - uav_id) * square_width, State1_height); // 注这里用于测试用，设置的2，因为实际测试的uav为1和3号
        // if(uav_id < 5) // 朝y正半轴 1,2,3,4
        //     pos_ = Eigen::Vector3d(service_height, (5 - uav_id) * square_width, State1_height);
        // else // 朝y负半轴
        //     pos_ = Eigen::Vector3d(service_height, -(uav_id - 5) * square_width, State1_height);
        
        std::cout << BLUE << "当前点：" << vins_pose.transpose() << "---> 搜索区域起始点：" << pos_.transpose() << TAIL << std::endl;
        if(is_to_target_pose(vins_pose, pos_))
        {
            std::cout << GREEN << uav_name << "已经到达搜索区域，盘旋2s..." << TAIL << std::endl;
            yolo_detect_timer ++;
            if(yolo_detect_timer == 20)
            {
                yolo_detect_timer = 0;
                uav_ready_go = true;
                std::cout << GREEN << uav_name << "盘旋时间到，准备搜索..." << TAIL << std::endl;
            }
            // 到达搜索区域
        }

        // 速度控制
        if(velocity_ctrl_mode)
        {
            double time = ros::Time::now().toSec();
            double dt = time - last_time;
            Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
            cmd_.cmd = 6; // 速度控制
            cmd_.desired_vel[0] = vel[0];
            cmd_.desired_vel[1] = vel[1];
            cmd_.desired_vel[2] = vel[2];
            // cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            cmd_.desired_yaw = 0;
            cmd_pub_.publish(cmd_);
            last_time = time;
            // std::cout << RED << "cur_x_vel = " << vel_from_autopilot[0] << " cur_y_vel = " << vel_from_autopilot[1] << " cur_z_vel = " << vel_from_autopilot[2] << std::endl;
            // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
        }
        else // 位置控制
        {
            cmd_.cmd = 4;
            cmd_.desired_pos[0] = pos_[0];
            cmd_.desired_pos[1] = pos_[1];
            cmd_.desired_pos[2] = pos_[2];
            cmd_.enable_yawRate = false;
            cmd_.desired_yaw = 0; 
            cmd_pub_.publish(cmd_);
        }   

        if(uav_ready_go)
        {
            if(search_strategy != "v_center_search" && search_strategy != "v_search" && search_strategy != "h_search" && search_strategy != "v_center_search_new")
            {
                std::cout << RED << "search_strategy模式错误，请重新输入：(v_search,h_search,v_center_search,v_center_search_new)" << TAIL << std::endl;
                return;
            }
            if(search_strategy == "v_center_search")
            {
                pos_ = edges_poses_[edges_poses_.size() - 1]; // 取最后一个
                state = 1;
            }
            else if(search_strategy == "v_center_search_new")
            {
                pos_ = edges_poses_[edges_poses_.size() - 1]; // 取最后一个
            }
            else
            {
                pos_ = edges_poses_[0];
                state = 1;
            }
            
            stage.data = 0;
            last_time = ros::Time::now().toSec(); // 记录上一次时间
        }
    }
    else // 达到搜索区域
    {
        if(search_strategy == "h_search" && uav_state != -1)
            h_search(); // 横向搜索
        else if(search_strategy == "v_search" && uav_state != -1)
            v_search(); // 纵向搜索
        else if(search_strategy == "v_center_search" && uav_state != -1)
            v_center_search(); // 纵向搜索
        else if(search_strategy == "v_center_search_new" && uav_state != -1)
             v_center_search_new(); // 纵向搜索
    }

}


void uav_search_strategy::pub_trajectory_cb(const ros::TimerEvent& e) 
{
    sunray_msgs::targetPos P;
    P.target_id = uav_id;
    P.pose_x = vins_pose[0];
    P.pose_y = vins_pose[1];
    P.pose_z = vins_pose[2];
    uav_true_pose_pub_.publish(P); // 发布无人机的实时坐标
    // sunray_msgs::targetPos P;
    P.pose_x = init_pose[0];
    P.pose_y = init_pose[1];
    P.pose_z = init_pose[2];
    uav_init_pose_pub_.publish(P); // 发布开始搜索时的坐标
    // std::cout<<ros::Time::now().toSec()<<std::endl;

    switch (mode_) {
        case 1: // takeoff
        {
            pos_ = Eigen::Vector3d(0,0,Takeoff_height); // 设置起飞点
            if(velocity_ctrl_mode & 0) // 速度控制
            {
                // pos_ = Eigen::Vector3d(0,0,Takeoff_height); // 设置起飞点
                double time = ros::Time::now().toSec();
                double dt = time - last_time;
                Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
                cmd_.cmd = 6; // 速度控制
                cmd_.desired_vel[0] = vel[0];
                cmd_.desired_vel[1] = vel[1];
                cmd_.desired_vel[2] = vel[2];
                // cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                cmd_.desired_yaw = 0;
                cmd_pub_.publish(cmd_);
                last_time = time;
            }
            
            if(is_to_target_pose(vins_pose, pos_))
            {
                std::cout << GREEN << "悬停2s..." << TAIL << std::endl;
                yolo_detect_timer ++;
                if(yolo_detect_timer == 20)
                {
                    yolo_detect_timer = 0;
                    std::cout << GREEN << "可以进行下一步操作了！" << TAIL << std::endl;

                    if(takeoff) // 接收到起飞指令，直接进入搜索阶段，无需手动
                    {
                        mode_ = 3;
                        last_time = ros::Time::now().toSec(); // 记录上一次时间
                        std::cout << RED << "开始搜索..." << TAIL << std::endl;
                    }
                }    
            }

        }
        break;
        case 2: // move pose
        {
            if(velocity_ctrl_mode) // 速度控制
            {
                double time = ros::Time::now().toSec();
                double dt = time - last_time;
                Eigen::Vector3d vel = vel_ctrl.calculate_velocity(vins_pose, pos_, dt, vel_from_autopilot);
                cmd_.cmd = 6; // 速度控制
                cmd_.desired_vel[0] = vel[0];
                cmd_.desired_vel[1] = vel[1];
                cmd_.desired_vel[2] = vel[2];
                // cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                cmd_.desired_yaw = 0;
                cmd_pub_.publish(cmd_);
                last_time = time;
                // std::cout << GREEN << "current_pose = " << vins_pose.transpose() << "------>target_pose = " << pos_.transpose() << std::endl;
                // std::cout << RED << "x_vel = " << vel[0] << " y_vel = " << vel[1] << " z_vel = " << vel[2] << std::endl;
            }
            else // 位置控制
            {
                cmd_.cmd = 4;
                cmd_.desired_pos[0] = pos_[0];
                cmd_.desired_pos[1] = pos_[1];
                cmd_.desired_pos[2] = pos_[2];
                cmd_.enable_yawRate = false;
                cmd_.desired_yaw = 0;
                cmd_pub_.publish(cmd_);
            }
            break;
        }
        case 3: // coverage pose
        {
            ready_go();
            break;
        }
        default:
            break;
    }
}



void uav_search_strategy::run() 
{ 
    while (ros::ok()) {
        // print_info();
        if(search_strategy != "v_center_search" && search_strategy != "v_search" && search_strategy != "h_search" && search_strategy != "v_center_search_new")
        {
            std::cout << RED << "search_strategy模式为：" << search_strategy << " 请重新输入：(v_search,h_search,v_center_search,v_center_search_new)" << TAIL << std::endl;
        }
        else if(!check_timeout())
        {
            int tmp;
            std::cout << "Please select the operation mode: 1 arming 2 takeoff 3 move 4 land 5 seeparam" << std::endl;
            // if(abs(vins_pose[2] - Takeoff_height) < 0.15)
            //     std::cout << GREEN << "飞机到达起飞高度,可以进行下一步操作了！" << TAIL << std::endl;
            std::cin >> tmp;
            
            switch (tmp) {
                case 1: // arm
                    int arming;
                    std::cout << "Please select Operation: 1 arm 0 disarm" << std::endl;
                    std::cin >> arming;
                    if (arming!= 1 && arming!= 0) {
                        std::cout << "input error" << std::endl;
                    } 
                    else 
                    {
                        setup_.cmd = 0;
                        setup_.arming = arming;
                        setup_pub_.publish(setup_);
                        if (arming == 1){
                            std::cout << "Arming!" << std::endl;
                            sleep(1);
                            setup_.cmd = 3;
                            setup_.control_state = "CMD_CONTROL";
                            setup_pub_.publish(setup_);
                            std::cout << "Set cmd control mode!" << std::endl;
                            if(takeoff) // 如果有takeoff指令，则等待2s后直接起飞，无需手动
                            {
                                sleep(2); // 延迟2s后直接起飞
                                mode_ = 1;
                                last_time = ros::Time::now().toSec(); // 记录上一次时间
                                cmd_.cmd = 1;
                                cmd_pub_.publish(cmd_);
                            }
                        }else{
                            std::cout << "Disarming!" << std::endl;
                        } 
                    }
                    if(!takeoff)
                        mode_ = 0;
                    
                    break;
                case 2: // takeoff
                {
                    mode_ = 1;
                    last_time = ros::Time::now().toSec(); // 记录上一次时间
                    
                    // 位置控制
                    cmd_.cmd = 1;
                    cmd_pub_.publish(cmd_);
                    // cmd_.cmd = 4;
                    // cmd_.desired_pos[0] = pos_[0];
                    // cmd_.desired_pos[1] = pos_[1];
                    // cmd_.desired_pos[2] = pos_[2];
                    // cmd_.enable_yawRate = false;
                    // cmd_.desired_yaw = 0;
                    // cmd_pub_.publish(cmd_);
                    
                }
                break;
                case 3: // move
                    // if (abs(vins_pose[2] - Takeoff_height) > 0.2) // 判断移动高度是否足够
                    // {
                    //     std::cout << "高度不够，不允许move" << std::endl;
                    // }
                    // else{ // 高度够了
                    int op;
                    std::cout << "Please select Operation 1: move pose 2: coverage pose" << std::endl;
                    std::cin >> op;
                    if (op == 1) {
                        std::cout << "input x (m)" << std::endl;
                        std::cin >> input[0];
                        std::cout << "input y (m)" << std::endl;
                        std::cin >> input[1];
                        std::cout << "input z (m)" << std::endl;
                        std::cin >> input[2];

                        last_time = ros::Time::now().toSec(); // 记录上一次时间

                        if (abs(input[0]) < square_height && abs(input[1]) < square_width && abs(input[2]) < height_up) {
                            mode_ = 2; // 位置控制 or 速度控制
                            pos_ = input;
                        } else{
                            mode_ = 0;
                            std::cout << "The input pose is out of range!" << std::endl;
                        }

                    } else if (op == 2) { // 开始覆盖

                        // if(search_strategy == "v_center_search")
                        // {
                        //     pos_ = v_center_search_str.pose[v_center_search_str.pose.size() - 1]; // 取最后一个
                        // }
                        // else
                        //     pos_ = edges_poses_[0];
                        mode_ = 3;
                        // state = 1;
                        // stage.data = 0;
                        last_time = ros::Time::now().toSec(); // 记录上一次时间

                    }
                    // }
                    break;
                case 4: // land
                    uav_search_state.data = 1; // 当前无人机搜索状态
                    cmd_.cmd = 3;
                    cmd_pub_.publish(cmd_);
                    mode_ = 0;
                    break;
                case 5:
                    print_info();
                    break;
                default:
                    break;
            }
        }

    } 
}

void uav_search_strategy::start() 
{
    run_thread_ = boost::thread(&uav_search_strategy::run, this); // Start the run thread
}

void uav_search_strategy::print_info()
{
    std::cout << GREEN << ">>>>>>>>>>>>>>>> UAV Search Outdoor Param <<<<<<<<<<<<<<<<" << TAIL << std::endl;
    std::cout << GREEN << "uav_id                    : " << uav_id << " " << TAIL << std::endl;
    std::cout << GREEN << "external_source           : " << external_source << " " << TAIL << std::endl;  
    std::cout << GREEN << "vision_source             : " << vision_source << " " << TAIL << std::endl;  
    
    std::cout << GREEN << "search_strategy           : " << search_strategy << " " << TAIL << std::endl;
    
    std::cout << GREEN << "total_width               : " << total_width << " " << TAIL << std::endl;

    std::cout << GREEN << "service_height            : " << service_height << " " << TAIL << std::endl;
    
    
    std::cout << GREEN << "square_width              : " << square_width << " " << TAIL << std::endl;
    std::cout << GREEN << "square_height             : " << square_height << " " << TAIL << std::endl;
    std::cout << GREEN << "step                      : " << step << " " << TAIL << std::endl;
    std::cout << GREEN << "height_step               : " << height_step << " " << TAIL << std::endl;
    std::cout << GREEN << "back_step                 : " << back_step << " " << TAIL << std::endl;
    
    std::cout << GREEN << "enable_hight_lidar        : " << enable_hight_lidar << " " << TAIL << std::endl;
    
    std::cout << GREEN << "Takeoff_height            : " << Takeoff_height << " [m] " << TAIL << std::endl;
    std::cout << GREEN << "State1_height             : " << State1_height << " [m] " << TAIL << std::endl;
    std::cout << GREEN << "State2_height             : " << State2_height << " [m/s] " << TAIL << std::endl;
    
    std::cout << GREEN << "geo_fence_x : " << x_min << " [m]  to  " << x_max << " [m]" << TAIL << std::endl;
    std::cout << GREEN << "geo_fence_y : " << y_min << " [m]  to  " << y_max << " [m]" << TAIL << std::endl;
    std::cout << GREEN << "geo_fence_z : " << z_min << " [m]  to  " << z_max << " [m]" << TAIL << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_search_strategy");
    ros::NodeHandle nh("~");
    uav_search_strategy tp(nh);
    tp.start(); // Start the run thread
    ros::spin(); // Keep the main thread alive
    return 0;
}
