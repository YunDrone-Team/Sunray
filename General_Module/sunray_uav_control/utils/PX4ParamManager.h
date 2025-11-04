#ifndef PX4_PARAM_MANAGER_H
#define PX4_PARAM_MANAGER_H

#include <ros/ros.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <string>
#include <cstdint>

/**
 * @brief PX4参数管理器类，通过Mavros服务管理PX4参数
 * 
 * 本类提供通过Mavros服务读取和设置PX4参数的功能。
 * 
 * 主要功能：
 * 1. 查询并显示PX4参数
 * 2. 运行时动态设置PX4参数
 * 3. 支持整数型和浮点型参数
 */
class PX4ParamManager
{
public:
    /**
     * @brief 构造函数，初始化参数管理器
     * 
     * @param nh ROS节点句柄
     * @param mavrosNamespace Mavros命名空间（如 "/uav1/mavros"）
     */
    PX4ParamManager(ros::NodeHandle& nh, const std::string& mavrosNamespace = "/uav1/mavros");

    /**
     * @brief 析构函数
     */
    ~PX4ParamManager();

    /**
     * @brief 查询并显示PX4参数值
     * 
     * @param paramId 参数名称（如 "EKF2_HGT_REF"）
     * @return 成功返回true，失败返回false
     */
    bool getParam(const std::string& paramId);

    /**
     * @brief 设置PX4参数（整数型）
     * 
     * @param paramId 参数名称
     * @param value 要设置的整数值
     * @return 成功返回true，失败返回false
     */
    bool setParam(const std::string& paramId, int64_t value);

    /**
     * @brief 设置PX4参数（浮点型）
     * 
     * @param paramId 参数名称
     * @param value 要设置的浮点值
     * @return 成功返回true，失败返回false
     */
    bool setParam(const std::string& paramId, double value);

    /**
     * @brief 获取参数值但不打印（用于程序内部逻辑）
     * 
     * @param paramId 参数名称
     * @param paramValue 输出参数值
     * @return 成功返回true，失败返回false
     */
    bool getParamValue(const std::string& paramId, mavros_msgs::ParamValue& paramValue);

private:
    /**
     * @brief 打印查询到的参数值
     * 
     * @param paramId 参数名称
     * @param paramValue 参数值
     */
    void printGetParam(const std::string& paramId, const mavros_msgs::ParamValue& paramValue);

    /**
     * @brief 打印参数设置的变化
     * 
     * @param paramId 参数名称
     * @param paramValueOld 旧的参数值
     * @param paramValueNew 新的参数值
     */
    void printSetParam(const std::string& paramId, 
                      const mavros_msgs::ParamValue& paramValueOld, 
                      const mavros_msgs::ParamValue& paramValueNew);

    /**
     * @brief 从PX4读取当前参数值
     * 
     * @param paramId 参数名称
     * @param paramValue 输出参数值
     * @return 成功返回true，失败返回false
     */
    bool readParam(const std::string& paramId, mavros_msgs::ParamValue& paramValue);

    ros::ServiceClient paramGetClient;  // PX4参数获取服务客户端
    ros::ServiceClient paramSetClient;  // PX4参数设置服务客户端
};

#endif // PX4_PARAM_MANAGER_H

