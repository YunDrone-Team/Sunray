#include "PX4ParamManager.h"
#include "ros_msg_utils.h"
#include <iostream>
#include <string>
#include <signal.h>

using namespace std;

volatile sig_atomic_t shutdownRequested = 0;

void signalHandler(int signal)
{
    shutdownRequested = 1;
    Logger::print_color(int(LogColor::yellow), "\n收到退出信号，正在退出...");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    Logger::init_default();
    ros::init(argc, argv, "px4_param_test_class");
    ros::NodeHandle nh("~");
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    PX4ParamManager paramManager(nh, "/uav1/mavros");
    int input = 0;
    string paramName;
    int64_t integerValue;
    double realValue;
    while(ros::ok() && !shutdownRequested)
    {
        ros::spinOnce();
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Mavros Param Test (Class Version)<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "请选择:"
            << YELLOW << " 0 " << GREEN << "退出程序,"
            << YELLOW << " 1 " << GREEN << "查询PX4参数,"
            << YELLOW << " 2 " << GREEN << "设置PX4参数（整数型参数）,"
            << YELLOW << " 3 " << GREEN << "设置PX4参数（浮点型参数）." << TAIL << endl;
        cin >> input;
        if (cin.fail() || shutdownRequested)
        {
            break;
        }
        switch (input)
        {
            case 0:
            {
                Logger::print_color(int(LogColor::green), "退出程序");
                ros::shutdown();
                return 0;
            }
            case 1:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> paramName;
                paramManager.getParam(paramName);
                break;
            }
            case 2:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> paramName;
                cout << BLUE << "请输入参数值（注意一定要是整数）" << TAIL << endl;
                cin >> integerValue;
                paramManager.setParam(paramName, integerValue);
                break;
            }
            case 3:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> paramName;
                cout << BLUE << "请输入参数值（注意一定要是浮点数）" << TAIL << endl;
                cin >> realValue;
                paramManager.setParam(paramName, realValue);
                break;
            }
            default:
            {
                Logger::print_color(int(LogColor::yellow), "无效的选项，请重新选择");
                break;
            }
        }
        ros::Duration(0.5).sleep();
    }
    Logger::print_color(int(LogColor::green), "程序已退出");
    ros::shutdown();
    return 0;
}

