#include "PX4ParamManager.h"
#include "ros_msg_utils.h"

PX4ParamManager::PX4ParamManager(ros::NodeHandle& nh, const std::string& mavrosNamespace)
{
    std::string getServiceName = mavrosNamespace + "/param/get";
    std::string setServiceName = mavrosNamespace + "/param/set";
    paramGetClient = nh.serviceClient<mavros_msgs::ParamGet>(getServiceName);
    paramSetClient = nh.serviceClient<mavros_msgs::ParamSet>(setServiceName);
    Logger::print_color(int(LogColor::green), "PX4ParamManager initialized with namespace:", mavrosNamespace);
}

PX4ParamManager::~PX4ParamManager()
{
}

void PX4ParamManager::printGetParam(const std::string& paramId, const mavros_msgs::ParamValue& paramValue)
{
    if (paramValue.integer != 0)
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", paramId, ":", paramValue.integer);
    }
    else if (paramValue.real != 0.0)
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", paramId, ":", paramValue.real);
    }
    else
    {
        Logger::print_color(int(LogColor::green), "成功读取PX4参数 -", paramId, ":", 0);
    }
}

void PX4ParamManager::printSetParam(const std::string& paramId,
                                    const mavros_msgs::ParamValue& paramValueOld,
                                    const mavros_msgs::ParamValue& paramValueNew)
{
    if (paramValueOld.integer != 0)
    {
        Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", paramId, ":", 
                          paramValueOld.integer, "->", paramValueNew.integer);
    }
    else if (paramValueOld.real != 0.0)
    {
        Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", paramId, ":", 
                          paramValueOld.real, "->", paramValueNew.real);
    }
    else
    {
        if (paramValueNew.integer != 0)
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", paramId, ":", 
                              0, "->", paramValueNew.integer);
        }
        else if (paramValueNew.real != 0.0)
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", paramId, ":", 
                              0, "->", paramValueNew.real);
        }
        else
        {
            Logger::print_color(int(LogColor::green), "成功设置PX4参数 -", paramId, ":", 
                              0, "->", 0);
        }
    }
}

bool PX4ParamManager::readParam(const std::string& paramId, mavros_msgs::ParamValue& paramValue)
{
    mavros_msgs::ParamGet px4ParamGet;
    px4ParamGet.request.param_id = paramId;
    px4ParamGet.response.success = false;
    if (!paramGetClient.waitForExistence(ros::Duration(0.5)))
    {
        Logger::print_color(int(LogColor::red), "参数读取服务不可用（超时）", paramId);
        return false;
    }
    if (paramGetClient.call(px4ParamGet))
    {
        if (!px4ParamGet.response.success)
        {
            Logger::print_color(int(LogColor::red), "无法读取指定参数", paramId);
            return false;
        }
        paramValue = px4ParamGet.response.value;
        return true;
    }
    else
    {
        Logger::print_color(int(LogColor::red), "参数读取服务调用失败", paramId);
        return false;
    }
}

bool PX4ParamManager::getParam(const std::string& paramId)
{
    mavros_msgs::ParamValue paramValue;
    if (!readParam(paramId, paramValue))
    {
        return false;
    }
    printGetParam(paramId, paramValue);
    return true;
}

bool PX4ParamManager::getParamValue(const std::string& paramId, mavros_msgs::ParamValue& paramValue)
{
    return readParam(paramId, paramValue);
}

bool PX4ParamManager::setParam(const std::string& paramId, int64_t value)
{
    mavros_msgs::ParamValue paramValueOld;
    if (!readParam(paramId, paramValueOld))
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", paramId);
        return false;
    }
    mavros_msgs::ParamSet px4ParamSet;
    px4ParamSet.request.param_id = paramId;
    px4ParamSet.request.value.integer = value;
    px4ParamSet.request.value.real = 0;
    px4ParamSet.response.success = false;
    if (!paramSetClient.waitForExistence(ros::Duration(0.5)))
    {
        Logger::print_color(int(LogColor::red), "参数设置服务不可用（超时）", paramId);
        return false;
    }
    if (paramSetClient.call(px4ParamSet))
    {
        if (!px4ParamSet.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，服务响应失败", paramId);
            return false;
        }
        Logger::print_color(int(LogColor::green), "设置参数成功：", paramId);
        mavros_msgs::ParamValue paramValue = px4ParamSet.response.value;
        printSetParam(paramId, paramValueOld, paramValue);
        return true;
    }
    else
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，服务无法被调用", paramId);
        return false;
    }
}

bool PX4ParamManager::setParam(const std::string& paramId, double value)
{
    if (!paramSetClient.waitForExistence(ros::Duration(0.5)))
    {
        Logger::print_color(int(LogColor::red), "参数设置服务不可用（超时）", paramId);
        return false;
    }
    mavros_msgs::ParamValue paramValueOld;
    if (!readParam(paramId, paramValueOld))
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，无法读取指定参数", paramId);
        return false;
    }
    mavros_msgs::ParamSet px4ParamSet;
    px4ParamSet.request.param_id = paramId;
    px4ParamSet.request.value.integer = 0;
    px4ParamSet.request.value.real = value;
    px4ParamSet.response.success = false;
    
    if (paramSetClient.call(px4ParamSet))
    {
        if (!px4ParamSet.response.success)
        {
            Logger::print_color(int(LogColor::red), "设置参数失败，服务响应失败", paramId);
            return false;
        }
        Logger::print_color(int(LogColor::green), "设置参数成功：", paramId);
        mavros_msgs::ParamValue paramValue = px4ParamSet.response.value;
        printSetParam(paramId, paramValueOld, paramValue);
        return true;
    }
    else
    {
        Logger::print_color(int(LogColor::red), "设置参数失败，服务无法被调用", paramId);
        return false;
    }
}

