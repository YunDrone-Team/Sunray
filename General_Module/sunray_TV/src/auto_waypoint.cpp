#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"
#include <csignal>
#include "printf_format.h"
#include "thread"

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
        // 【参数】移动类型 0 位置控制 1 规划控制
        nh.param<int>("move_type", move_type, 1);

        uav_name = uav_name + std::to_string(uav_id);
        string topic_prefix = "/" + uav_name;

        // 【发布】无人机控制指令 （本节点 -> sunray_control_node）
        control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
        // 【发布】无人机设置指令（本节点 -> sunray_control_node）
        uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_" + std::to_string(uav_id), 10);

        px4_state_sub = nh.subscribe<sunray_msgs::PX4State>(topic_prefix + "/sunray/px4_state", 10, &auto_waypoint::px4_state_callback, this);
        uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state", 10, &auto_waypoint::uav_state_callback, this);

        plan_points.push_back(tuple<float, float, float>(5, 3, 1));
        plan_points.push_back(tuple<float, float, float>(-3, -5, 1));
        plan_points.push_back(tuple<float, float, float>(-10, 0, 1));
        plan_points.push_back(tuple<float, float, float>(1, 0, 1));

        state = 0;
        next_step = 0;
        next_plan = 1;
        plan_step = 0;
        if (task_type == 1)
        {
            next_step = 1;
        }

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
                    next_step = 1;
                    if (state == 4)
                    {
                        next_plan = 1;
                    }
                }
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
        if (plan_step == plan_points.size())
        {
            state = 5;
            return;
        }
        if (next_plan)
        {
            Logger::info("The current point is: ", plan_step, "point: [", get<0>(plan_points[plan_step]), get<1>(plan_points[plan_step]), get<2>(plan_points[plan_step]), "]");
            if (move_type == 1)
            {
                goal_point.header.stamp = ros::Time::now();
                goal_point.header.frame_id = "/world";
                goal_point.pose.position.x = get<0>(plan_points[plan_step]);
                goal_point.pose.position.y = get<1>(plan_points[plan_step]);
                goal_point.pose.position.z = get<2>(plan_points[plan_step]);
                goal_pub.publish(goal_point);
            }
            else
            {
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 1;
                uav_cmd.desired_pos[0] = std::get<0>(plan_points[plan_step]);
                uav_cmd.desired_pos[1] = std::get<1>(plan_points[plan_step]);
                uav_cmd.desired_pos[2] = std::get<2>(plan_points[plan_step]);
                control_cmd_pub.publish(uav_cmd);
            }
            next_plan = 0;
        }
        if (abs(uav_state.position[0] - get<0>(plan_points[plan_step])) < 0.20 &&
            abs(uav_state.position[1] - get<1>(plan_points[plan_step])) < 0.20 &&
            abs(uav_state.position[2] - get<2>(plan_points[plan_step])) < 0.20)
        {
            plan_step++;
            Logger::info("The current point has been reached: ", plan_step);
            if (task_type == 1)
            {
                next_plan = 1;
            }
        }
    }

    void print_state()
    {
        Logger::info("------------------------------------");
        Logger::info("state: ", state);
        Logger::info("plan_step: ", plan_step);
    }

    void main_loop()
    {
        switch (state)
        {
        case 0:
            if (next_step)
            {
                Logger::info("switch to arm");
                switchMode(0);
                if (task_type == 0)
                {
                    next_step = 0;
                }
            }
            if (px4_state.armed)
            {
                state = 1;
            }
            break;
        case 1:
            if (next_step)
            {
                Logger::info("switch to CMD_CONTROL");
                switchMode(1);
                if (task_type == 0)
                {
                    next_step = 0;
                }
            }
            if (uav_state.control_mode == 2)
            {
                state = 2;
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
    ros::Publisher goal_pub;
    ros::Subscriber px4_state_sub;
    ros::Subscriber uav_state_sub;
    sunray_msgs::UAVControlCMD uav_cmd;
    sunray_msgs::UAVSetup setup;
    sunray_msgs::PX4State px4_state;
    sunray_msgs::UAVState uav_state;
    geometry_msgs::PoseStamped goal_point;
    thread keyboard_thread;

    int uav_id;
    string uav_name;
    int next_step;
    int next_plan;
    int plan_step;
    int task_type;
    int move_type;

    // 状态机状态量 0： 就绪 1：解锁 2：起飞中 3：悬停 ：4 开始发布规划点 5：降落 6：退出
    int state;
    // 规划状态量 0：未开始 1：规划中 2：结束
    int plan_state;

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
