#ifndef Control_Utils_H
#define Control_Utils_H

#include <ros/ros.h>
#include "sunray_logger.h"
#include <Eigen/Eigen>
#include <sunray_msgs/UAVControlCMD.h>
#include <sunray_msgs/UAVState.h>
#include <sunray_msgs/UAVSetup.h>

using namespace std;
using namespace sunray_logger;

namespace sunray
{
    class Control_Utils
    {
    public:
        // 构造函数
        Control_Utils() = delete;
        // 析构函数
        ~Control_Utils() = delete;

        sunray_msgs::UAVState uav_state;
        sunray_msgs::UAVSetup uav_setup;
        sunray_msgs::UAVControlCMD uav_cmd;
        ros::Subscriber uav_state_sub;

        ros::Publisher control_cmd_pub;
        ros::Publisher uav_setup_pub;
        string node_name{"control_utils"};

        // 初始化默认变量
        void sunray_init(ros::NodeHandle &nh, int uav_id)
        {
            string uav_name = "/uav" + std::to_string(uav_id);

            // 【订阅】无人机状态
            uav_state_sub = nh.subscribe<sunray_msgs::UAVState>(uav_name + "/sunray/uav_state", 10, &Control_Utils::uav_state_callback,this);

            // 【发布】控制指令
            control_cmd_pub = nh.advertise<sunray_msgs::UAVControlCMD>(uav_name + "/sunray/uav_control_cmd", 1);

            // 【发布】无人机设定指令
            uav_setup_pub = nh.advertise<sunray_msgs::UAVSetup>(uav_name + "/sunray/setup", 1);
        }

        void uav_state_callback(const sunray_msgs::UAVState::ConstPtr &msg)
        {
            uav_state = *msg;
        }

        void auto_takeoff()
        {
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
        }


    };
}
#endif