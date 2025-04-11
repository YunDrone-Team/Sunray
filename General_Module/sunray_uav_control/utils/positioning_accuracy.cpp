#include "ros_msg_utils.h"
#include "printf_format.h"
#include <numeric>

using namespace sunray_logger;
using namespace std;

struct TimeValue
{
    std::vector<double> pos_x;
    std::vector<double> pos_y;
    std::vector<double> pos_z;
    std::vector<double> vel_x;
    std::vector<double> vel_y;
    std::vector<double> vel_z;
};

// error estimation
class ErrorEstimation
{
private:
    int uav_id;
    int state;
    float wait_time;
    float record_time;
    bool record_flag;
    std::string uav_name;
    std::string uav_prefix;
    std::string topic_prefix;
    sunray_msgs::UAVControlCMD uav_cmd;
    sunray_msgs::UAVSetup setup;
    sunray_msgs::UAVState uav_state;
    double pos_err_mean[3];
    double vel_mean[3];
    double yaw_err_mean;
    double pos_err_max[3];
    double vel_max[3];
    double yaw_err_max;

    ros::Subscriber uav_state_sub;

    ros::Publisher control_cmd_pub;
    ros::Publisher uav_setup_pub;

    ros::Time last_time;
    ros::Time start_time;

    ros::Timer print_timer;
    ros::Timer update_timer;

    std::vector<TimeValue> timeValues;
    std::vector<std::tuple<double, double, double>> points;

public:
    ErrorEstimation(ros::NodeHandle &nh)
    {
        nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
        nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称

        uav_prefix = uav_name + std::to_string(uav_id);
        topic_prefix = "/" + uav_prefix;

        uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(topic_prefix + "/sunray/uav_state",
                                                            10, &ErrorEstimation::uav_state_callback, this);
        control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(topic_prefix + "/sunray/uav_control_cmd", 1);
        uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(topic_prefix + "/sunray/setup", 1);

        print_timer = nh.createTimer(ros::Duration(0.1), &ErrorEstimation::print_err, this);

        state = 0;
        wait_time = 0.5;
        record_time = 20; // 20s
        record_flag = false;
        last_time = ros::Time::now();
        timeValues.resize(5);
        points.push_back(std::make_tuple(0, 0, 1));
        points.push_back(std::make_tuple(1, 1, 1));
        points.push_back(std::make_tuple(1, -1, 1));
        points.push_back(std::make_tuple(-1, -1, 1));
        points.push_back(std::make_tuple(1, -1, 1));
    }

    void update_callback()
    {
        if (state >= 3 && state <= 7)
        {
            // std::cout << "err_x" << uav_state.position[0] - std::get<0>(points[state - 3]) << std::endl;
            // std::cout << "err_y" << uav_state.position[1] - std::get<1>(points[state - 3]) << std::endl;
            // std::cout << "err_z" << uav_state.position[2] - std::get<2>(points[state - 3]) << std::endl;
            if (!record_flag &&
                abs(uav_state.position[0] - std::get<0>(points[state - 3])) < 0.1 &&
                abs(uav_state.position[1] - std::get<1>(points[state - 3])) < 0.1 &&
                abs(uav_state.position[2] - std::get<2>(points[state - 3])) < 0.1)
            {
                if (abs(uav_state.velocity[0]) < 0.1 &&
                    abs(uav_state.velocity[1]) < 0.1 &&
                    abs(uav_state.velocity[2]) < 0.1)
                {
                    record_flag = true;
                    start_time = ros::Time::now();
                    std::cout << "start record point: " << state - 3 << std::endl;
                }
            }
            if (record_flag)
            {
                timeValues[state - 3].pos_x.push_back(uav_state.position[0] - std::get<0>(points[state - 3]));
                timeValues[state - 3].pos_y.push_back(uav_state.position[1] - std::get<1>(points[state - 3]));
                timeValues[state - 3].pos_z.push_back(uav_state.position[2] - std::get<2>(points[state - 3]));
                timeValues[state - 3].vel_x.push_back(uav_state.velocity[0]);
                timeValues[state - 3].vel_y.push_back(uav_state.velocity[1]);
                timeValues[state - 3].vel_z.push_back(uav_state.velocity[2]);
            }
        }
    }

    void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
    {
        uav_state = *msg;
        update_callback();
    }

    void calculate_err()
    {

        for (int i = 0; i < 5; i++)
        {
            pos_err_max[0] = pos_err_max[1] = pos_err_max[2] = 0;
            vel_max[0] = vel_max[1] = vel_max[2] = 0;
            pos_err_mean[0] = pos_err_mean[1] = pos_err_mean[2] = 0;
            vel_mean[0] = vel_mean[1] = vel_mean[2] = 0;
            
            pos_err_mean[0] += std::accumulate(timeValues[i].pos_x.begin(), timeValues[i].pos_x.end(), 0.0) / timeValues[i].pos_x.size();
            pos_err_mean[1] += std::accumulate(timeValues[i].pos_y.begin(), timeValues[i].pos_y.end(), 0.0) / timeValues[i].pos_y.size();
            pos_err_mean[2] += std::accumulate(timeValues[i].pos_z.begin(), timeValues[i].pos_z.end(), 0.0) / timeValues[i].pos_z.size();

            vel_mean[0] += std::accumulate(timeValues[i].vel_x.begin(), timeValues[i].vel_x.end(), 0.0) / timeValues[i].vel_x.size();
            vel_mean[1] += std::accumulate(timeValues[i].vel_y.begin(), timeValues[i].vel_y.end(), 0.0) / timeValues[i].vel_y.size();
            vel_mean[2] += std::accumulate(timeValues[i].vel_z.begin(), timeValues[i].vel_z.end(), 0.0) / timeValues[i].vel_z.size();

            pos_err_max[0] = max(pos_err_max[0], *max_element(timeValues[i].pos_x.begin(), timeValues[i].pos_x.end()));
            pos_err_max[1] = max(pos_err_max[1], *max_element(timeValues[i].pos_y.begin(), timeValues[i].pos_y.end()));
            pos_err_max[2] = max(pos_err_max[2], *max_element(timeValues[i].pos_z.begin(), timeValues[i].pos_z.end()));

            vel_max[0] = max(vel_max[0], *max_element(timeValues[i].vel_x.begin(), timeValues[i].vel_x.end()));
            vel_max[1] = max(vel_max[1], *max_element(timeValues[i].vel_y.begin(), timeValues[i].vel_y.end()));
            vel_max[2] = max(vel_max[2], *max_element(timeValues[i].vel_z.begin(), timeValues[i].vel_z.end()));
            Logger::print_color(int(LogColor::blue), LOG_BOLD, "--------------------------------");
            Logger::print_color(int(LogColor::blue), "【POS_X_MAX", "POS_Y_MAX", "POS_Z_MAX】");
            if (pos_err_max[0] > 0.1 || pos_err_max[1] > 0.1 || pos_err_max[2] > 0.1)
                Logger::print_color(int(LogColor::yellow), pos_err_max[0], pos_err_max[1], pos_err_max[2]);
            else
                Logger::print_color(int(LogColor::green), pos_err_max[0], pos_err_max[1], pos_err_max[2]);
            Logger::print_color(int(LogColor::blue), "【POS_X_MEAN", "POS_Y_MEAN", "POS_Z_MEAN】");
            if (pos_err_mean[0] > 0.1 || pos_err_mean[1] > 0.1 || pos_err_mean[2] > 0.1)
                Logger::print_color(int(LogColor::yellow), pos_err_mean[0], pos_err_mean[1], pos_err_mean[2]);
            else
                Logger::print_color(int(LogColor::green), pos_err_mean[0], pos_err_mean[1], pos_err_mean[2]);
            Logger::print_color(int(LogColor::blue), "【VEL_X_MAX", "VEL_Y_MAX", "VEL_Z_MAX】");
            if (vel_max[0] > 0.1 || vel_max[1] > 0.1 || vel_max[2] > 0.1)
                Logger::print_color(int(LogColor::yellow), vel_max[0], vel_max[1], vel_max[2]);
            else
                Logger::print_color(int(LogColor::green), vel_max[0], vel_max[1], vel_max[2]);
            Logger::print_color(int(LogColor::blue), "【VEL_X_MEAN", "VEL_Y_MEAN", "VEL_Z_MEAN】");
            if (vel_mean[0] > 0.1 || vel_mean[1] > 0.1 || vel_mean[2] > 0.1)
                Logger::print_color(int(LogColor::yellow), vel_mean[0], vel_mean[1], vel_mean[2]);
            else
                Logger::print_color(int(LogColor::green), vel_mean[0], vel_mean[1], vel_mean[2]);
        }

        // 关闭程序
        ros::shutdown();
    }

    void print_err(const ros::TimerEvent &event)
    {
    }

    void main_loop()
    {
        /*
        0 解锁
        1 切换模式
        2 起飞
        3 移动到第一个点（0 0 1）
        4 移动到第二个点（1 1 1）
        5 移动到第三个点（1 -1 1）
        6 移动到第四个点（-1 -1 1）
        7 移动到第五个点（1 -1 1）
        8 返回降落
        */

        switch (state)
        {
        case 0:
            if (uav_state.armed)
            {
                state = 1;
                last_time = ros::Time::now();
                wait_time = 0.5;
                break;
            }
            if ((ros::Time::now() - last_time).toSec() > wait_time)
            {
                // 解锁
                cout << "arm" << endl;
                setup.cmd = 1;
                uav_setup_pub.publish(setup);
                wait_time += 1;
            }
            break;
        case 1:
            if (uav_state.control_mode == 2)
            {
                state = 2;
                last_time = ros::Time::now();
                wait_time = 0.5;
                break;
            }
            if ((ros::Time::now() - last_time).toSec() > wait_time)
            {
                // 切换到指令控制模式
                cout << "switch CMD_CONTROL" << endl;
                setup.cmd = 4;
                setup.control_mode = "CMD_CONTROL";
                uav_setup_pub.publish(setup);
                wait_time += 1;
            }
            break;
        case 2:
            if ((ros::Time::now() - last_time).toSec() > 5)
            {
                state = 3;
                last_time = ros::Time::now();
                wait_time = 0.5;
                break;
            }
            if (wait_time == 0.5)
            {
                // 起飞
                cout << "takeoff" << endl;
                uav_cmd.cmd = 100;
                control_cmd_pub.publish(uav_cmd);
                wait_time += 1;
            }
            break;
        case 3:
            if (wait_time == 0.5) // 表示第一次进入该状态
            {
                std::cout << "go to point 1" << std::endl;
                last_time = ros::Time::now();
                wait_time += 1;
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 4;
                uav_cmd.desired_pos[0] = std::get<0>(points[0]);
                uav_cmd.desired_pos[1] = std::get<1>(points[0]);
                uav_cmd.desired_pos[2] = std::get<2>(points[0]);
                uav_cmd.desired_yaw = 0.0;
                control_cmd_pub.publish(uav_cmd);
            }

            if (record_flag)
            {
                // std::cout << (ros::Time::now() - start_time).toSec() << std::endl;
                if ((ros::Time::now() - start_time).toSec() > record_time)
                {
                    state = 4;
                    last_time = ros::Time::now();
                    wait_time = 0.5;
                    record_flag = false;
                    break;
                }
            }
            break;
        case 4:
            if (wait_time == 0.5)
            {
                std::cout << "go to point 2" << std::endl;
                last_time = ros::Time::now();
                wait_time += 1;
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 4;
                uav_cmd.desired_pos[0] = std::get<0>(points[1]);
                uav_cmd.desired_pos[1] = std::get<1>(points[1]);
                uav_cmd.desired_pos[2] = std::get<2>(points[1]);
                uav_cmd.desired_yaw = 0.0;
                control_cmd_pub.publish(uav_cmd);
            }
            if (record_flag && (ros::Time::now() - start_time).toSec() > record_time)
            {
                state = 5;
                last_time = ros::Time::now();
                wait_time = 0.5;
                record_flag = false;
                break;
            }
            break;
        case 5:
            if (wait_time == 0.5)
            {
                std::cout << " go to point 3" << std::endl;
                last_time = ros::Time::now();
                wait_time += 1;
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 4;
                uav_cmd.desired_pos[0] = std::get<0>(points[2]);
                uav_cmd.desired_pos[1] = std::get<1>(points[2]);
                uav_cmd.desired_pos[2] = std::get<2>(points[2]);
                uav_cmd.desired_yaw = 0.0;
                control_cmd_pub.publish(uav_cmd);
            }
            if (record_flag && (ros::Time::now() - start_time).toSec() > record_time)
            {
                state = 6;
                last_time = ros::Time::now();
                wait_time = 0.5;
                record_flag = false;
                break;
            }
            break;
        case 6:
            if (wait_time == 0.5)
            {
                std::cout << "go to point 4" << std::endl;
                last_time = ros::Time::now();
                wait_time += 1;
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 4;
                uav_cmd.desired_pos[0] = std::get<0>(points[3]);
                uav_cmd.desired_pos[1] = std::get<1>(points[3]);
                uav_cmd.desired_pos[2] = std::get<2>(points[3]);
                uav_cmd.desired_yaw = 0.0;
                control_cmd_pub.publish(uav_cmd);
            }
            if (record_flag && (ros::Time::now() - start_time).toSec() > record_time)
            {
                state = 7;
                last_time = ros::Time::now();
                wait_time = 0.5;
                record_flag = false;
                break;
            }
            break;
        case 7:
            if (wait_time == 0.5)
            {
                std::cout << "go to point 5" << std::endl;
                last_time = ros::Time::now();
                wait_time += 1;
                uav_cmd.header.stamp = ros::Time::now();
                uav_cmd.cmd = 4;
                uav_cmd.desired_pos[0] = std::get<0>(points[4]);
                uav_cmd.desired_pos[1] = std::get<1>(points[4]);
                uav_cmd.desired_pos[2] = std::get<2>(points[4]);
                uav_cmd.desired_yaw = 0.0;
                control_cmd_pub.publish(uav_cmd);
            }
            if (record_flag && (ros::Time::now() - start_time).toSec() > record_time)
            {
                state = 8;
                last_time = ros::Time::now();
                wait_time = 0.5;
                record_flag = false;
                break;
            }
            break;
        case 8:
            if (wait_time > 5)
            {
                state = 9;
                last_time = ros::Time::now();
                wait_time = 0.5;
                record_flag = false;
                break;
            }
            if (wait_time == 0.5)
            {
                cout << "return to home" << endl;
                uav_cmd.cmd = 104;
                control_cmd_pub.publish(uav_cmd);
                wait_time += 1;
            }
            wait_time += 1;
            break;
        case 9:
            calculate_err();
        default:
            break;
        }
    }

    ~ErrorEstimation()
    {
    }
};

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "ErrorEstimation");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    ErrorEstimation err(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        err.main_loop();
        rate.sleep();
    }
}