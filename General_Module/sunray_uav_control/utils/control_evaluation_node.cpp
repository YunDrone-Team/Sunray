#include "ros_msg_utils.h"
#include "control_evaluation.h"
#include "traj_generator.h"

using namespace sunray_logger;
using namespace std;

int uav_id;
std::string uav_name;

sunray_msgs::PX4State px4_state;
sunray_msgs::UAVSetup setup;
sunray_msgs::UAVControlCMD uav_cmd;
ros::Subscriber px4_state_sub;

ros::Publisher control_cmd_pub;
ros::Publisher uav_setup_pub;

ControlEvaluation ctrl_eva;

//用于控制器测试的类，功能例如：生成圆形轨迹，8字轨迹等
TRAJ_GENERATOR traj_generator;

// 无人机状态回调
void px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg)
{
    px4_state = *msg;
}

void traj_evaluation(double trajectory_total_time, int traj_mode)
{
    double time_trajectory = 0.0;

    while (time_trajectory < trajectory_total_time)
    {
        uav_cmd = traj_generator.Circle_trajectory_generation(time_trajectory);
        uav_cmd.header.stamp = ros::Time::now();
        uav_cmd.cmd = sunray_msgs::UAVControlCMD::CTRL_Traj;
        control_cmd_pub.publish(uav_cmd);

        time_trajectory = time_trajectory + 0.01;
        cout << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << endl;

        time_trajectory = time_trajectory + 0.01;
        ros::Duration(0.01).sleep();
    }
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

    nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
    nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称

    traj_generator.init(nh);
    

    uav_name = "/" + uav_name + std::to_string(uav_id);

    px4_state_sub = nh.subscribe<sunray_msgs::PX4State>(uav_name + "/sunray/px4_state", 10, px4_state_callback);

    control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);

    uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);




    Logger::print_color(int(LogColor::blue), LOG_BOLD, "-------------------- Control Evaluation --------------------");
    Logger::print_color(int(LogColor::blue), ">>>>>>> Evaluation Start");

    // 初始化检查：等待PX4连接
    int trials = 0;
    while (ros::ok() && !px4_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            Logger::print_color(int(LogColor::red), "Wait for uav connect");
    }

    Logger::print_color(int(LogColor::blue), ">>>>>>> uav connected!");

    Logger::print_color(int(LogColor::blue), ">>>>>>> arm uav in 5 sec!");

    ros::Duration(5.0).sleep();

    Logger::print_color(int(LogColor::blue), ">>>>>>> arm uav!");

    // 解锁无人机
    setup.cmd = sunray_msgs::UAVSetup::ARM;
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();


    // 切换到指令控制模式
    Logger::print_color(int(LogColor::blue), ">>>>>>> switch to CMD_CONTROL");
    setup.cmd = 4;
    setup.control_mode = "CMD_CONTROL";
    uav_setup_pub.publish(setup);
    ros::Duration(1.0).sleep();

    // 起飞
    Logger::print_color(int(LogColor::blue), ">>>>>>> switch to Takeoff");
    uav_cmd.cmd = 100;
    control_cmd_pub.publish(uav_cmd);
    ros::Duration(5.0).sleep();

    Logger::print_color(int(LogColor::blue), ">>>>>>> Pls select evaluation mode: 1 for point evaluation, 2 for traj evaulation");


    // point_evaluation();
    double traj_time = 100;
    traj_evaluation(traj_time, 1);


    ros::shutdown();


    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}




void point_evaluation()
{

}