#include "ros_msg_utils.h"
#include <cstdint>
// 通过Mavros的服务可以查询PX4参数和设置PX4参数
// 有什么用？ 
// 1、sunray_uav_control节点启动的时候，读取一些关键参数并打印出来（也可以传递到地面站，检查用）
// 2、sunray_uav_control在执行一些特定行为的时候，可以临时更改PX4参数（动态参数，配合功能）
// 3、通过Sunray地面站可以配置PX4参数，不需要再打开QGC软件了

// 【服务】PX4参数获取服务
ros::ServiceClient px4_param_get_client;
// 【服务】PX4参数设置服务
ros::ServiceClient px4_param_set_client;

// 打印函数
void print_get_px4_param(std::string param_id, const mavros_msgs::ParamValue param_value) 
{
    if (param_value.integer != 0) 
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", param_id,":", param_value.integer);
    } else if (param_value.real != 0.0) 
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", param_id,":", param_value.real);
    } else 
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", param_id,":", 0);
    }
}

// 打印函数
void print_set_px4_param(std::string param_id, const mavros_msgs::ParamValue param_value_old, const mavros_msgs::ParamValue param_value) 
{
    if (param_value_old.integer != 0) 
    {
        Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", param_id,":", param_value_old.integer, "->", param_value.integer);
    } else if (param_value_old.real != 0.0) 
    {
        Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", param_id,":", param_value_old.real, "->", param_value.real);
    } else 
    {
        if (param_value.integer != 0) 
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", param_id,":", 0, "->", param_value.integer);
        } else if (param_value.real != 0.0) 
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", param_id,":", 0, "->", param_value.real);
        }else
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", param_id,":", 0, "->", 0);
        }
    }
}

// 获取并记录飞控当前的参数值
// param_id是参数名字，如“EKF2_HGT_REF”
bool get_px4_param(std::string param_id)
{
    mavros_msgs::ParamGet px4_param_get;
    px4_param_get.request.param_id = param_id;
    px4_param_get.response.success = false;

    if (px4_param_get_client.call(px4_param_get))
    {
        if (!px4_param_get.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
            return false;
        }
        mavros_msgs::ParamValue param_value;
        param_value = px4_param_get.response.value;
        print_get_px4_param(param_id, param_value);
        return true;
    }
    else 
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
        return false;
    }
}

// 设置飞控参数值(参数是int64_t的情况)
bool set_px4_param(std::string param_id, int64_t value)
{
    // 设置前先读取原值
    mavros_msgs::ParamValue param_value_old;
    mavros_msgs::ParamGet px4_param_get;
    px4_param_get.request.param_id = param_id;
    px4_param_get.response.success = false;

    if (px4_param_get_client.call(px4_param_get))
    {
        if (!px4_param_get.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
            return false;
        }
        param_value_old = px4_param_get.response.value;
    }
    else 
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
        return false;
    }

    // 设置参数
    mavros_msgs::ParamSet px4_param_set;
    px4_param_set.request.param_id = param_id;
    px4_param_set.request.value.integer = value;
    px4_param_set.request.value.real = 0;
    px4_param_set.response.success = false;

    if (px4_param_set_client.call(px4_param_set))
    {
        if (!px4_param_set.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，服务响应失败", param_id);
            return false;
        }
        
        Logger::print_color(int(LogColor::green), "设置参数成功：", param_id);
        mavros_msgs::ParamValue param_value;
        param_value = px4_param_set.response.value;     // 设置后返回的参数值
        print_set_px4_param(param_id, param_value_old, param_value);
        return true;
    }
    else 
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，服务无法被调用", param_id);
        return false;
    }
}

// 设置飞控参数值(参数是double的情况)
bool set_px4_param(std::string param_id, double value)
{
    // 设置前先读取原值
    mavros_msgs::ParamValue param_value_old;
    mavros_msgs::ParamGet px4_param_get;
    px4_param_get.request.param_id = param_id;
    px4_param_get.response.success = false;

    if (px4_param_get_client.call(px4_param_get))
    {
        if (!px4_param_get.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
            return false;
        }
        param_value_old = px4_param_get.response.value;
    }
    else 
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", param_id);
        return false;
    }

    // 设置参数
    mavros_msgs::ParamSet px4_param_set;
    px4_param_set.request.param_id = param_id;
    px4_param_set.request.value.integer = 0;
    px4_param_set.request.value.real = value;
    px4_param_set.response.success = false;

    if (px4_param_set_client.call(px4_param_set))
    {
        if (!px4_param_set.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，服务响应失败", param_id);
            return false;
        }
        
        Logger::print_color(int(LogColor::green), "设置参数成功：", param_id);
        mavros_msgs::ParamValue param_value;
        param_value = px4_param_set.response.value;     // 设置后返回的参数值
        print_set_px4_param(param_id, param_value_old, param_value);
        return true;
    }
    else 
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，服务无法被调用", param_id);
        return false;
    }
}

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();

    ros::init(argc, argv, "px4_param_test");
    ros::NodeHandle nh("~");

    // 【服务】PX4参数获取服务
    px4_param_get_client = nh.serviceClient<mavros_msgs::ParamGet>("/uav1/mavros/param/get");
    // 【服务】PX4参数设置服务
    px4_param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("/uav1/mavros/param/set");
    
    int input = 0;
    string param_name;
    int64_t integer_;
    double real_;

    while(ros::ok())
    {
        ros::spinOnce();
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>Mavros Param Test<<<<<<<<<<<<<<<<<<<<<<<<< " << TAIL << endl;
        cout << GREEN << "请选择:"
            << YELLOW << " 1 " << GREEN << "查询PX4参数,"
            << YELLOW << " 2 " << GREEN << "设置PX4参数（整数型参数）,"
            << YELLOW << " 3 " << GREEN << "设置PX4参数（浮点型参数）." << TAIL << endl;
        cin >> input;
        switch (input)
        {
            case 1:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> param_name;
                get_px4_param(param_name);
                break;
            }
            case 2:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> param_name;
                cout << BLUE << "请输入参数值（注意一定要是整数）" << TAIL << endl;
                cin >> integer_;
                set_px4_param(param_name, (int64_t)integer_);
                break;
            }
            case 3:
            {
                cout << BLUE << "请输入PX4参数名字，如EKF2_HGT_REF" << TAIL << endl;
                cin >> param_name;
                cout << BLUE << "请输入参数值（注意一定要是浮点数）" << TAIL << endl;
                cin >> real_;
                set_px4_param(param_name, (double)real_);
                break;
            }
        }

        ros::Duration(0.5).sleep();
    }

    // 程序结束
    ros::shutdown();
}
