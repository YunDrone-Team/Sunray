/*
本程序功能：
    1.在不使用Sunray地面站和其他模块的情况下，实现对单个无人机的基本控制功能
      包括：起飞、降落、悬停、位置控制、速度控制、加速度控制、姿态控制等
      以及简单的轨迹生成（圆形轨迹，8字轨迹等）
    2.可通过终端输入命令实现对无人机的控制
    3.注意：只能控制单机，无论输入什么控制指令，都会发送给所有的无人机。此程序仅作内部测试使用
*/
#include "ros_msg_utils.h"
#include "traj_generator.h"

using namespace std;
#define TRA_WINDOW 2000
sunray_msgs::UAVControlCMD uav_cmd;
sunray_msgs::UAVSetup uav_setup;
std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
sunray_msgs::UAVState uav_state;
int uav_num;

bool flag = false;
int Trjectory_mode;
int trajectory_total_time;
string node_name;
std::vector<ros::Publisher> uav_command_pub;
std::vector<ros::Publisher> uav_setup_pub;
std::vector<ros::Publisher> ref_trajectory_pub;
void auto_takeoff();
void mySigintHandler(int sig)
{
    ROS_INFO("[uav_command_pub] exit...");
    ros::shutdown();
    exit(0);
}
// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "uav_command_pub");
    ros::NodeHandle nh("~");
    signal(SIGINT, mySigintHandler);

    node_name = ros::this_node::getName();
    int uav_id;
    string uav_name{"uav"};
    string topic_prefix{""};
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    nh.param("uav_num", uav_num, 1);
    uav_name = "/" + uav_name + to_string(uav_id);

    // 【订阅】无人机状态
    ros::Subscriber uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);

    // 【发布】
    for (int i = 0; i < uav_num; i++)
    {
        uav_name = "uav" + std::to_string(i + 1);
        topic_prefix = "/" + uav_name;
        uav_command_pub.push_back(nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1));
        uav_setup_pub.push_back(nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1));
        ref_trajectory_pub.push_back(nh.advertise<nav_msgs::Path>(topic_prefix + "/sunray/reference_trajectory", 1));
    }

    //用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
    TRAJ_GENERATOR traj_generator;
    traj_generator.init(nh);

    int input = 0;
    float state_desired[4];
    bool yaw_rate = false;

    uav_cmd.header.stamp = ros::Time::now();
    uav_cmd.cmd = 3;
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

    float time_trajectory = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV Terminal Control<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << ">>> COMBO CMD: "
                << YELLOW << " 999 " << GREEN << "一键解锁、起飞、切换至OFFBOARD模式并移动至[0,0,1]位置"<< TAIL << endl;
        cout << GREEN << ">>> UAV SETUP CMD: "
                << YELLOW << " 901 " << GREEN << "ARM or DISARM,"
                << YELLOW << " 902 " << GREEN << "SET_PX4_MODE,"
                << YELLOW << " 903 " << GREEN << "REBOOT_PX4,"
                << YELLOW << " 904 " << GREEN << "SET_CONTROL_MODE,"
                << YELLOW << " 905 " << GREEN << "EMERGENCY_KILL"<< TAIL << endl;
        cout << GREEN << ">>> UAV CONTROL CMD: "<< TAIL << endl;
        cout << GREEN << "基础移动模式:"
                << YELLOW << " 1 " << GREEN << "XYZ_POS,"
                << YELLOW << " 4 " << GREEN << "XyzPosYaw,"
                << YELLOW << " 14 " << GREEN << "XyzPosYawBody,"
                << YELLOW << " 17 " << GREEN << "GlobalPos"  << TAIL << endl;
        // cout << GREEN << "自定义控制器:" 
        //         << YELLOW << " 50 " << GREEN << "CTRL_XyzPos,"
        //         << YELLOW << " 51 " << GREEN << "CTRL_Traj"  << TAIL << endl;
        cout << GREEN << "特殊指令:" 
                << YELLOW << " 30 " << GREEN << "Point," 
                << YELLOW << " 100 " << GREEN << "Takeoff,"
                << YELLOW << " 101 " << GREEN << "Land,"
                << YELLOW << " 102 " << GREEN << "Hover,"
                << YELLOW << " 103 " << GREEN << "Waypoint,"
                << YELLOW << " 104 " << GREEN << "Return" << TAIL << endl;
        cout << YELLOW << "Please input :" << TAIL << endl;
        cin >> input;

        switch (input)
        {
        case 999:
        {
            auto_takeoff();
            break;
        }
        case 901:
        {
            int arming;
            cout << BLUE << "Please select Operation: 1 arm 0 disarm" << TAIL << endl;
            cin >> arming;
            if (arming == 1)
            {
                uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
                for (int i = 0; i < uav_num; i++)
                {
                    uav_setup_pub[i].publish(uav_setup);
                }
            }
            else if (arming == 0)
            {
                uav_setup.cmd = sunray_msgs::UAVSetup::DISARM;
                for (int i = 0; i < uav_num; i++)
                {
                    uav_setup_pub[i].publish(uav_setup);
                }
            }
            else
            {
                cout << BLUE << "input error" << TAIL << endl;
            }
            break;
        }
        case 902:
        {
            cout << BLUE << "set px4 mode: 0 for OFFBOARD, 1 for AUTO.LAND, 2 for POSCTL, 3 for AUTO.RTL" << TAIL << endl;
            int tmp;
            cin >> tmp;
            uav_setup.cmd  = sunray_msgs::UAVSetup::SET_PX4_MODE;
            if(tmp == 0){
                uav_setup.px4_mode = "OFFBOARD";
            }else if(tmp == 1){
                uav_setup.px4_mode = "AUTO.LAND";
            }
            else if(tmp == 2){
                uav_setup.px4_mode = "POSCTL";
            }
            else if(tmp == 3){
                uav_setup.px4_mode = "AUTO.RTL";
            }
            else{
                cout << "Mode error " << endl;
            }
            cout << BLUE << "px4 mode: " << uav_setup.px4_mode << TAIL << endl;
            for (int i = 0; i < uav_num; i++)
            {
                uav_setup_pub[i].publish(uav_setup);
            }            
            break;
        }
        case 903:
        {
            uav_setup.cmd = sunray_msgs::UAVSetup::REBOOT_PX4;
            for (int i = 0; i < uav_num; i++)
            {
                uav_setup_pub[i].publish(uav_setup);
            }
            break;
        }
        case 904:
        {
            int control_mode;
            cout << BLUE << "SET_CONTROL_MODE: 0 INIT, 1 RC_CONTROL, 2 CMD_CONTROL, 3 LAND_CONTROL, 4 WITHOUT_CONTROL" << TAIL << endl;
            cin >> control_mode;
            if (control_mode == 0)
                uav_setup.control_mode = "INIT";
            else if (control_mode == 1)
                uav_setup.control_mode = "RC_CONTROL";
            else if (control_mode == 2)
                uav_setup.control_mode = "CMD_CONTROL";
            else if (control_mode == 3)
                uav_setup.control_mode = "LAND_CONTROL";
            else if (control_mode == 4)
                uav_setup.control_mode = "WITHOUT_CONTROL";
            else
            {
                cout << RED << "input error" << TAIL << endl;
                break;
            }
            uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
            for (int i = 0; i < uav_num; i++)
            {
                uav_setup_pub[i].publish(uav_setup);
            }
            break;
        }
        case 905:
        {
            uav_setup.cmd = sunray_msgs::UAVSetup::EMERGENCY_KILL;
            for (int i = 0; i < uav_num; i++)
            {
                uav_setup_pub[i].publish(uav_setup);
            }
            break;
        }
        case 1:
            cout << BLUE << "XyzPos: 惯性系定点控制,保持当前偏航角" << endl;
            cout << BLUE << "请输入期望的[X Y Z]" << endl;
            cout << BLUE << "desired_pos: --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_pos: --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_pos: --- z [m]" << endl;
            cin >> state_desired[2];

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPos;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            break;
        case 4:
        {
            cout << BLUE << "XyzPosYaw: 惯性系定点控制,带偏航角" << endl;
            cout << BLUE << "请输入期望的[X Y Z YAW]" << endl;
            cout << BLUE << "desired_pos --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_pos --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_pos: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired_att: --- yaw [deg]:" << endl;
            cin >> state_desired[3];
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3]; 
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }

            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "desired_yaw : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }
        case 14:
        {
            cout << BLUE << "XyzPosYawBody: 机体坐标系XYZ位置 带偏航角" << endl;
            cout << BLUE << "请输入期望的[X Y Z YAW]" << endl;
            cout << BLUE << "desired_pos --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_pos --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_pos: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired_att: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令 Z轴速度设为 0，因为我们要指定高度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawBody;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }

            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }

        case 17:
        {
            cout << BLUE << "GlobalPos: 全局坐标 经纬度海拔" << endl;
            cout << BLUE << "请输入期望的latitude、longitude、altitude、yaw" << endl;
            cout << BLUE << "latitude:" << endl;
            cin >> state_desired[0];
            cout << BLUE << "longitude:" << endl;
            cin >> state_desired[1];
            // 高度为相对高度
            cout << BLUE << "altitude: --- z [m]" << endl;
            cin >> state_desired[2]; // 高度（Z轴）
            cout << BLUE << "desired_att: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::GlobalPos;
            uav_cmd.latitude = state_desired[0];
            uav_cmd.longitude = state_desired[1];
            uav_cmd.altitude = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }

            cout << BLUE << "latitude longitude altitude: " << state_desired[0] << " " << state_desired[1] << " " << state_desired[2] << " " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }

        case 30:
        {
            cout << BLUE << "Point: 路径规划的目标点" << endl;
            cout << BLUE << "请输入期望的[X Y Z YAW]" << endl;
            cout << BLUE << "desired_pos --- x [m] " << endl;
            cin >> state_desired[0];
            cout << BLUE << "desired_pos --- y [m]" << endl;
            cin >> state_desired[1];
            cout << BLUE << "desired_pos: --- z [m]" << endl;
            cin >> state_desired[2];
            cout << BLUE << "desired_att: --- yaw [deg]:" << endl;
            cin >> state_desired[3];                            // 固定yaw
            state_desired[3] = state_desired[3] / 180.0 * M_PI; // 转换为弧度

            // 填充 UAV 指令 Z轴速度设为 0，因为我们要指定高度
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Point;
            uav_cmd.desired_pos[0] = state_desired[0];
            uav_cmd.desired_pos[1] = state_desired[1];
            uav_cmd.desired_pos[2] = state_desired[2];
            uav_cmd.desired_yaw = state_desired[3];
            uav_cmd.desired_yaw_rate = 0.0;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "pos_des [X Y Z] : " << state_desired[0] << " [ m ] " << state_desired[1] << " [ m ] " << state_desired[2] << " [ m ] " << endl;
            cout << BLUE << "yaw_des : " << state_desired[3] / M_PI * 180.0 << " [ deg ] " << endl;
            break;
        }
        case 100:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "Takeoff" << endl;
            break;
        case 101:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "Land" << endl;
            break;
        case 102:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "Hover" << endl;
            break;
        case 103:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Waypoint;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "Waypoint" << endl;
            break;
        case 104:
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Return;
            for (int i = 0; i < uav_num; i++)
            {
                uav_command_pub[i].publish(uav_cmd);
            }
            cout << BLUE << "Return" << endl;
            break;
        }

        ros::Duration(0.5).sleep();
    }
    return 0;
}

void auto_takeoff()
{
    int times = 0;
    while (ros::ok() && !uav_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (times++ > 5)
            Logger::print_color(int(LogColor::red), node_name, ": Wait for UAV connect...");
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV connected!");

    // 切换到指令控制模式(同时，PX4模式将切换至OFFBOARD模式)
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::CMD_CONTROL)
    {

        uav_setup.cmd = sunray_msgs::UAVSetup::SET_CONTROL_MODE;
        uav_setup.control_mode = "CMD_CONTROL";
        for (int i = 0; i < uav_num; i++)
        {
            uav_setup_pub[i].publish(uav_setup);
        }
        Logger::print_color(int(LogColor::green), node_name, ": SET_CONTROL_MODE - [CMD_CONTROL]. ");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": UAV control_mode set to [CMD_CONTROL] successfully!");

    // 解锁无人机
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 5 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 4 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 3 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 2 sec...");
    ros::Duration(1.0).sleep();
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV in 1 sec...");
    ros::Duration(1.0).sleep();
    while (ros::ok() && !uav_state.armed)
    {
        uav_setup.cmd = sunray_msgs::UAVSetup::ARM;
        for (int i = 0; i < uav_num; i++)
        {
            uav_setup_pub[i].publish(uav_setup);
        }
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

    // 起飞无人机
    while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
        for (int i = 0; i < uav_num; i++)
        {
            uav_command_pub[i].publish(uav_cmd);
        }
        Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();
    }
    Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");

    Logger::print_color(int(LogColor::blue), ">>>>>>> move to the specified height");
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
    uav_cmd.desired_vel[0] = 0.0;
    uav_cmd.desired_vel[1] = 0.0;
    uav_cmd.desired_pos[2] = 1.0;
    uav_cmd.desired_yaw = 0.0;
    for (int i = 0; i < uav_num; i++)
    {
        uav_command_pub[i].publish(uav_cmd);
    }
    ros::Duration(2).sleep();
}