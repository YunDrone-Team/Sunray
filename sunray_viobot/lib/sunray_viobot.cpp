#include "sunray_viobot.h"

void VIOBOT::init(ros::NodeHandle& nh, bool if_pritf)
{
    flag_printf = if_pritf;
    // nh.param<string>("uav_name", uav_name, "none");
    // nh.param<int>("uav_id", uav_id, 0);

    // string topic_prefix = "/"  + uav_name + std::to_string(uav_id);

    // 【发布】vio state
    viobot_state_pub = nh.advertise<sunray_msgs::ViobotState>("/viobot/viobot_state", 2);
    // 【发布】stereo2算法启动、关闭、重置操作 to /viobot节点
    pub_stereo2_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/viobot/stereo2_ctrl", 2);
    // 【订阅】算法状态 from /viobot节点
    viobot_algo_status_sub = nh.subscribe("/viobot/algo_status", 1, &VIOBOT::viobot_algo_status_cb, this);
    // 【订阅】imu信息 from /viobot节点
    viobot_imu_sub = nh.subscribe("/viobot/imu", 1,  &VIOBOT::viobot_imu_cb, this);
    // 【订阅】左目图像信息 from /viobot节点
    viobot_image_left_sub = nh.subscribe("/viobot/image_left", 1,  &VIOBOT::viobot_image_left_cb, this);
    // 【订阅】右目图像信息 from /viobot节点
    viobot_image_right_sub = nh.subscribe("/viobot/image_right", 1,  &VIOBOT::viobot_image_right_cb, this);
    // 【订阅】左目相机参数 from /viobot节点
    viobot_camera_left_info_sub = nh.subscribe("/viobot/camera_left_info", 1,  &VIOBOT::viobot_camera_left_info_cb, this);
    // 【订阅】右目相机参数 from /viobot节点
    viobot_camera_right_info_sub = nh.subscribe("/viobot/camera_right_info", 1,  &VIOBOT::viobot_camera_right_info_cb, this);
    // 【订阅】里程计数据 from /stereo2节点
    stereo2_odometry_rect_sub = nh.subscribe("/viobot/pr_loop/odometry_rect", 1,  &VIOBOT::stereo2_odometry_rect_cb, this);
    // 【订阅】原始点云数据 from /stereo2节点
    stereo2_points_sub = nh.subscribe("/viobot/pr_loop/points", 1,  &VIOBOT::stereo2_points_cb, this);
    // 【订阅】RDF点云数据 from /stereo2节点
    stereo2_points_rdf_sub = nh.subscribe("/viobot/pr_loop/points_rdf", 1,  &VIOBOT::stereo2_points_rdf_cb, this);
    // 【订阅】带特征点的图片 from /stereo2节点
    stereo2_warped_img_sub = nh.subscribe("/viobot/zc_stereo2/warped_img", 1,  &VIOBOT::stereo2_warped_img_cb, this);
    // 【定时器】定时打印
    debug_timer = nh.createTimer(ros::Duration(1.0),  &VIOBOT::debug_timer_cb, this);

    node_name = ros::this_node::getName();
    cout << GREEN << node_name << " - VIOBOT init! " << TAIL << endl;
}

void VIOBOT::shutdown_stereo2()
{
    // 关闭算法
    while(algo_status.algo_status != "ready")
    {
        cout << YELLOW << "shutdown stereo2... " << TAIL << endl;
        stereo2_ctrl.algo_enable = false;
        stereo2_ctrl.algo_reboot = false;
        stereo2_ctrl.algo_reset = false;
        pub_stereo2_ctrl.publish(stereo2_ctrl);

        ros::spinOnce();
        sleep(2.0);
    }
}

bool VIOBOT::start_stereo2()
{
    // 首先是确认已关闭算法（多次启动的情况）
    while(ros::ok() && algo_status.algo_status != "ready")
    {
        cout << YELLOW << "stereo2 is runing, stop it firstly... " << TAIL << endl;
        stereo2_ctrl.algo_enable = false;
        stereo2_ctrl.algo_reboot = false;
        stereo2_ctrl.algo_reset = false;
        pub_stereo2_ctrl.publish(stereo2_ctrl);

        ros::spinOnce();
        sleep(2.0);
    }

    // 启动算法
    while(ros::ok() && algo_status.algo_status != "stereo2_running")
    {
        cout << YELLOW << "start stereo2... " << TAIL << endl;
        stereo2_ctrl.algo_enable = true;
        stereo2_ctrl.algo_reboot = false;
        stereo2_ctrl.algo_reset = false;
        pub_stereo2_ctrl.publish(stereo2_ctrl);

        ros::spinOnce();
        sleep(2.0);
    }

    cout << BLUE << "algo_status: [" << algo_status.algo_status << "]" << TAIL << endl;

    return 1;
}

void VIOBOT::debug_timer_cb(const ros::TimerEvent &e)
{
    if(!flag_printf)
        return;
    
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(NUM_POINT);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << BLUE << "algo_status: [" << algo_status.algo_status << "]" << TAIL << endl;

    if(algo_status.algo_status == "stereo2_running")
    {
        cout << GREEN << "----> stereo2_odometry_rect: " << TAIL << endl;
        cout << GREEN << "Pos [X Y Z] : " << odom_rect.pose.pose.position.x << " [ m ] " << odom_rect.pose.pose.position.y << " [ m ] " << odom_rect.pose.pose.position.z << " [ m ] " << TAIL << endl;
        cout << GREEN << "Vel [X Y Z] : " << odom_rect.twist.twist.linear.x << " [m/s] " << odom_rect.twist.twist.linear.y << " [m/s] " << odom_rect.twist.twist.linear.z << " [m/s] " << TAIL << endl;
        cout << GREEN << "Att [R P Y] : " << euler_viobot[0] * 180 / M_PI << " [deg] " << euler_viobot[1] * 180 / M_PI << " [deg] " << euler_viobot[2] * 180 / M_PI << " [deg] " << TAIL << endl;
    }

}

void VIOBOT::viobot_algo_status_cb(const system_ctrl::algo_status::ConstPtr &msg)
{
    algo_status = *msg;
}

void VIOBOT::viobot_camera_left_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    // cout << GREEN << "camera_left_info: " << msg  << TAIL << endl;
}

void VIOBOT::viobot_camera_right_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    // cout << GREEN << "camera_right_info: " << msg  << TAIL << endl;
}

void VIOBOT::viobot_image_right_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    // cout << GREEN << "image_right: " << msg  << TAIL << endl;
}

void VIOBOT::viobot_image_left_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    // cout << GREEN << "image_left: " << msg  << TAIL << endl;
}

void VIOBOT::viobot_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    // cout << GREEN << "imu: " << msg  << TAIL << endl;
}

void VIOBOT::stereo2_odometry_rect_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_rect = *msg;

    tf::Quaternion original_quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf::Quaternion quat_rotate;
    quat_rotate.setRPY(0, -M_PI/2, M_PI/2);
    tf::Quaternion final_quat = original_quat * quat_rotate;
    q_viobot = Eigen::Quaterniond(final_quat.w(), final_quat.x(), final_quat.y(), final_quat.z());
    euler_viobot = quaternion_to_euler_viobot(q_viobot);

    viobot_pos[0] = odom_rect.pose.pose.position.x;
    viobot_pos[1] = odom_rect.pose.pose.position.y;
    viobot_pos[2] = odom_rect.pose.pose.position.z;
    viobot_vel[0] = odom_rect.twist.twist.linear.x;
    viobot_vel[1] = odom_rect.twist.twist.linear.y;
    viobot_vel[2] = odom_rect.twist.twist.linear.z;
    viobot_yaw = euler_viobot[2];
    q_viobot_msg.x = q_viobot.x();
    q_viobot_msg.y = q_viobot.y();
    q_viobot_msg.z = q_viobot.z();
    q_viobot_msg.w = q_viobot.w();

    if(algo_status.algo_status == "stereo2_running")
    {
        viobot_state.vio_start = true;
    }else
    {
        viobot_state.vio_start = false;
    }
    viobot_state.position[0] = viobot_pos[0];
    viobot_state.position[1] = viobot_pos[1];
    viobot_state.position[2] = viobot_pos[2];
    viobot_state.velocity[0] = viobot_vel[0];
    viobot_state.velocity[1] = viobot_vel[1];
    viobot_state.velocity[2] = viobot_vel[2];
    viobot_state.attitude[0] = euler_viobot[0];
    viobot_state.attitude[1] = euler_viobot[1];
    viobot_state.attitude[2] = euler_viobot[2];
    viobot_state.attitude_q = q_viobot_msg;
    viobot_state_pub.publish(viobot_state);
}

void VIOBOT::stereo2_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // cout << GREEN << "stereo2_points_cb: " << TAIL << endl;
}
void VIOBOT::stereo2_points_rdf_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    rdf_point = *msg;
    rdf_point_ptr = msg;
    // cout << GREEN << "stereo2_points_rdf_cb: " << TAIL << endl;
}
void VIOBOT::stereo2_warped_img_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    // cout << GREEN << "stereo2_warped_img_cb: " << TAIL << endl;
}


Eigen::Vector3d VIOBOT::quaternion_to_euler_viobot(const Eigen::Quaterniond &q) 
{
    Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();

    //将Eigen::Matrix3d 手动转换为 tf::Matrix3x3
    tf::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                    rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                    rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

    double yaw, pitch, roll;
    tf_rotation_matrix.getRPY(roll, pitch, yaw);

    Eigen::Vector3d euler;
    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;

    return euler;
}
