#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>
#include "printf_format.h"
#include "thread"
#include "std_msgs/Int16.h"

using namespace sunray_logger;
using namespace std;

void mySigintHandler(int sig)
{
    std::cout << "[] exit..." << std::endl;

    ros::shutdown();
    exit(EXIT_SUCCESS); // 或者使用 exit(0)
}

class auto_waypoint
{
public:
    auto_waypoint(ros::NodeHandle &nh)
    {
        // 【参数】无人机编号
        nh.param<int>("uav_id", uav_id, 1);
        // 【参数】无人机名称
        nh.param<string>("uav_name", uav_name, "uav");
        // 【参数】任务类型 0 手动 1 自动
        nh.param<int>("task_type", task_type, 1);
        nh.param<float>("move_speed", move_speed, 0.2);
        nh.param<int>("start_sate", start_sate, 0);
        nh.param<string>("odom_topic", odom_topic, "/odom");

        uav_name = uav_name + std::to_string(uav_id);
        string topic_prefix = "/" + uav_name;

        // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
        control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
        // 【发布】无人机设置指令（本节点 -> sunray_control_node）
        uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

        px4_state_sub = nh.subscribe<sunray_msgs::PX4State>(topic_prefix + "/sunray/px4_state", 10, &auto_waypoint::px4_state_callback, this);
        uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, &auto_waypoint::uav_state_callback, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, &auto_waypoint::odom_callback, this);
        // 订阅一个int类型的消息
        avoid_sub = nh.subscribe<std_msgs::Int16>("/avoid/state", 10, &auto_waypoint::avoid_callback, this);

        state = start_sate;
        next_step = 0;
        if (task_type == 1)
        {
            next_step = 1;
        }

        avoid_time = ros::Time::now();
        avoid_type = 0; // 0: front, 1: left, 2: right

        // 开启键盘监听线程
        keyboard_thread = std::thread(&auto_waypoint::keyboard_input, this);
        // keyboard_thread.join();
        keyboard_thread.detach();
    }
    ~auto_waypoint()
    {
    }

    void keyboard_input()
    {
        char input;
        while (ros::ok())
        {
            Logger::warning("press key: [Space] to next step, [q/Q] to exit, [h/H] to hover, [l/L] to land");
            input = cin.get();
            if (task_type == 0)
            {
                // 下一步
                if (input == ' ')
                {
                    next_step = 1;                }
            }
            // 降落退出
            if (input == 'q' || input == 'Q')
            {
                state = 5;
            }
            // 悬停
            if (input == 'h' || input == 'H')
            {
                switchMode(3);
            }
            // 降落
            if (input == 'l' || input == 'L')
            {
                switchMode(4);
            }
            input = '1';
        }
    }

    void px4_state_callback(const sunray_msgs::PX4State::ConstPtr &msg)
    {
        px4_state = *msg;
    }

    void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
    {
        uav_state = *msg;
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        vehicle_odom = *msg;
    }

    void avoid_callback(const std_msgs::Int16::ConstPtr &msg)
    {
        avoid_type = msg->data;
        avoid_time = ros::Time::now();
    }

    void switchMode(int i)
    {
        switch (i)
        {
        case 0:
            ros::Duration(0.5).sleep();
            // 解锁
            cout << "arm" << endl;
            setup.cmd = 1;
            uav_setup_pub.publish(setup);
            ros::Duration(1.5).sleep();
            break;
        case 1:
            // 切换到指令控制模式
            cout << "switch CMD_CONTROL" << endl;
            setup.cmd = 4;
            setup.control_mode = "CMD_CONTROL";
            uav_setup_pub.publish(setup);
            ros::Duration(1.0).sleep();
            break;
        case 2:
            // 起飞
            cout << "takeoff" << endl;
            uav_cmd.cmd = 100;
            control_cmd_pub.publish(uav_cmd);
            ros::Duration(10.0).sleep();
            break;
        case 3:
            // 悬停
            cout << "hover" << endl;
            uav_cmd.cmd = 105;
            control_cmd_pub.publish(uav_cmd);
            ros::Duration(2).sleep();
            break;
        case 4:
            // 降落
            cout << "land" << endl;
            uav_cmd.cmd = 101;
            control_cmd_pub.publish(uav_cmd);
            ros::Duration(0.5).sleep();
            state = 5;
            break;
        }
    }

    void plan()
    {
        if (ros::Time::now() - avoid_time > ros::Duration(0.3))
        {
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::Hover;
            uav_cmd.header.stamp = ros::Time::now();
            control_cmd_pub.publish(uav_cmd);
            hover_flag = false;
            return;
        }

        if (avoid_type == 0)
        {
            hover_flag = false;
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyVelZPosYawBody;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.desired_vel[0] = move_speed;
            uav_cmd.desired_vel[1] = 0;
            uav_cmd.desired_vel[2] = 0;
            uav_cmd.desired_pos[2] = 0.8 - uav_state.position[2];
            control_cmd_pub.publish(uav_cmd);
        }
        else if (avoid_type == 1)
        {
            if(!hover_flag)
            {
                hover_pos[0] = uav_state.position[0];
                hover_pos[1] = uav_state.position[1];
                hover_pos[2] = uav_state.position[2];
            }
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawrate;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.desired_pos[0] = hover_pos[0];
            uav_cmd.desired_pos[1] = hover_pos[1];
            uav_cmd.desired_pos[2] = hover_pos[2];
            uav_cmd.desired_yaw_rate = M_PI/6;
            control_cmd_pub.publish(uav_cmd);
        }
        else if (avoid_type == 2)
        {
            if(!hover_flag)
            {
                hover_pos[0] = uav_state.position[0];
                hover_pos[1] = uav_state.position[1];
                hover_pos[2] = uav_state.position[2];
            }
            uav_cmd.cmd = sunray_msgs::UAVControlCMD::XyzPosYawrate;
            uav_cmd.header.stamp = ros::Time::now();
            uav_cmd.desired_pos[0] = hover_pos[0];
            uav_cmd.desired_pos[1] = hover_pos[1];
            uav_cmd.desired_pos[2] = hover_pos[2];
            uav_cmd.desired_yaw_rate = -M_PI/6;
            control_cmd_pub.publish(uav_cmd);
        }
    }

    void print_state()
    {
        Logger::print_color(int(LogColor::blue), LOG_BOLD, "---------------------------------");
        Logger::info("state: ", state);
        Logger::info("avoid_type: ", avoid_type);
    }

    void main_loop()
    {
        switch (state)
        {
        case 0:
            if (px4_state.armed)
            {
                state = 1;
                break;
            }
            if (next_step)
            {
                Logger::info("switch to arm");
                switchMode(0);
                if (task_type == 0)
                {
                    next_step = 0;
                }
            }
            break;
        case 1:
            if (uav_state.control_mode == 2)
            {
                state = 2;
                break;
            }
            if (next_step)
            {
                Logger::info("switch to CMD_CONTROL");
                switchMode(1);
                if (task_type == 0)
                {
                    next_step = 0;
                }
            }
            break;
        case 2:
            if (next_step)
            {
                Logger::info("switch to takeoff");
                switchMode(2);
                if (task_type == 0)
                {
                    next_step = 0;
                }
                state = 3;
            }
            break;
        case 3:
            if (next_step)
            {
                Logger::info("start plan!");
                state = 4;
            }
            if (task_type == 0)
            {
                next_step = 0;
            }
            break;
        case 4:
            plan();
            break;
        case 5:
            if (next_step)
            {
                Logger::info("switch to land");
                switchMode(4);
            }
            state = 6;
            break;
        case 6:
            Logger::info("completed! exiting");
            ros::shutdown();
            break;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher uav_setup_pub;
    ros::Publisher control_cmd_pub;
    ros::Subscriber px4_state_sub;
    ros::Subscriber uav_state_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber avoid_sub;
    sunray_msgs::UAVControlCMD uav_cmd;
    sunray_msgs::UAVSetup setup;
    sunray_msgs::PX4State px4_state;
    sunray_msgs::UAVState uav_state;
    nav_msgs::Odometry vehicle_odom;
    geometry_msgs::PoseStamped goal_point;
    thread keyboard_thread;

    int uav_id;
    string uav_name;
    std::string odom_topic;
    int next_step;
    int task_type;
    int start_sate;
    bool hover_flag;
    float move_speed;
    float hover_pos[3];
    int avoid_type;
    ros::Time avoid_time;
    // 状态机状态量 0： 就绪 1：解锁 2：起飞中 3：悬停 ：4 开始发布规划点 5：降落 6：退出
    int state;

    vector<tuple<float, float, float>> plan_points;
};

int main(int argc, char **argv)
{

    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "auto_waypoint");
    ros::NodeHandle nh("~");

    ros::Rate rate(20.0);
    auto_waypoint aw(nh);
    ros::Time now = ros::Time::now();
    while (ros::ok())
    {
        if (ros::Time::now() - now > ros::Duration(1.0))
        {
            aw.print_state();
            now = ros::Time::now();
        }
        aw.main_loop();
        ros::spinOnce();
        rate.sleep();
    }

    signal(SIGINT, mySigintHandler);
}
