/*
本程序功能：无人机位置环控制精度评测节点
    1.通过参数配置，设定控制精度评测模式；
        模式一：无人机飞行固定航点，评测每个点的悬停精度
        模式二：无人机追踪特定轨迹，评测轨迹整体追踪精度
    2.程序一旦执行，将自动控制无人机解锁、起飞并执行相关操作
    3.误差计算：订阅无人机控制期望值和当前位置、速度来计算轨迹追踪误差
    4.轨迹生成：通过TRAJ_GENERATOR类来生成无人机轨迹（含无人机位置、速度、加速度期望值）
    5.固定航点：通过参数配置
    
*/

#include "ros_msg_utils.h"
#include "control_evaluation.h"
#include "traj_generator.h"

using namespace sunray_logger;
using namespace std;

int uav_id;
std::string uav_name;
string node_name;

sunray_msgs::UAVState uav_state;
sunray_msgs::UAVSetup uav_setup;
sunray_msgs::UAVControlCMD uav_cmd;
ros::Subscriber px4_state_sub;

ros::Publisher control_cmd_pub;
ros::Publisher uav_setup_pub;

ControlEvaluation ctrl_eva;

//用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
TRAJ_GENERATOR traj_generator;

bool start_evaluation{false};

// 无人机状态回调
void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;

    if(!start_evaluation)
    {
        return;
    }

    // 记录控制信息
    Eigen::Vector3d pos_sp, vel_sp, att_sp;
    Eigen::Vector3d pos_now, vel_now, att_now;

    for(int i=0; i<3; i++)
    {
        pos_sp[i] = uav_cmd.desired_pos[i];
        vel_sp[i] = uav_cmd.desired_vel[i];
        if(i==2)
        {
            att_sp[i] = uav_cmd.desired_yaw;
        }else
        {
            att_sp[i] = 0.0;
        }
        pos_now[i] = uav_state.position[i];
        vel_now[i] = uav_state.velocity[i];
        att_now[i] = uav_state.attitude[i];
    }

    ctrl_eva.add_traj_info(pos_sp, pos_now, vel_sp, vel_now, att_sp, att_now);
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "control_evaluation_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
    node_name = ros::this_node::getName();

    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称

    // 轨迹生成 - 初始化
    traj_generator.init(nh);
    
    uav_name = "/" + uav_name + std::to_string(uav_id);

    // 订阅无人机状态
    px4_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, uav_state_callback);

    // 发布控制指令
    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);

    // 发布无人机设定指令
    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);


    Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- Control Evaluation --------------------");
    Logger::print_color(int(LogColor::blue), ">>>>>>> Evaluation Start");

    // 初始化检查：等待PX4连接
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
        uav_setup_pub.publish(uav_setup);
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
        uav_setup_pub.publish(uav_setup);
        Logger::print_color(int(LogColor::green), node_name, ": Arm UAV now.");
        ros::Duration(1.0).sleep();
        ros::spinOnce();      
    }
    Logger::print_color(int(LogColor::green), node_name, ": Arm UAV successfully!");

    // 起飞无人机
    while (ros::ok() && abs(uav_state.position[2] - uav_state.home_pos[2] - uav_state.takeoff_height) > 0.2)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Takeoff;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();      
    }
    Logger::print_color(int(LogColor::green), node_name, ": Takeoff UAV successfully!");

    // 以上: 无人机已成功起飞，进入自由任务模式

    Logger::print_color(int(LogColor::blue), ">>>>>>> move to trajectory start point");
    uav_cmd = traj_generator.Circle_trajectory_generation(0.0);
    uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYaw;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(10.0).sleep();

    Logger::print_color(int(LogColor::blue), ">>>>>>> start trajectory and evaluation");

    start_evaluation = true;
    double time_now = 0;
    double traj_time = 50;
    double time_last_printf = 0;
    while (time_now < traj_time && ros::ok())
    {
        // 生成轨迹并发布
        uav_cmd = traj_generator.Circle_trajectory_generation(time_now);
        control_cmd_pub.publish(uav_cmd);
        time_now = time_now + 0.01;

        if(time_now - time_last_printf > 1.0)
        {
            cout << GREEN << "Trajectory tracking: " << time_now << " / " << traj_time << " [ s ]" << TAIL << endl;

            ctrl_eva.show_debug();
            time_last_printf = time_now;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    start_evaluation = false;
    
    ctrl_eva.show_result();

    // 降落无人机
    while (ros::ok() && uav_state.control_mode != sunray_msgs::UAVSetup::LAND_CONTROL && uav_state.landed_state != 1)
    {
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::Land;
        control_cmd_pub.publish(uav_cmd);
        Logger::print_color(int(LogColor::green), node_name, ": Land UAV now.");
        ros::Duration(4.0).sleep();
        ros::spinOnce();      
    }
    // 等待降落
    while (ros::ok() && uav_state.landed_state != 1)
    {
        Logger::print_color(int(LogColor::green), node_name, ": Landing");
        ros::Duration(1.0).sleep();
        ros::spinOnce();      
    }
    // 成功降落
    Logger::print_color(int(LogColor::green), node_name, ": Land UAV successfully!");

    // Demo 结束
    Logger::print_color(int(LogColor::green), node_name, ": Demo finished, quit!");

    // 程序结束
    ros::shutdown();
}
