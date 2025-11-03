#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/StreamRate.h> // 添加StreamRate服务头文件

// 话题状态结构体
struct TopicStatus {
    std::string name;
    bool received;
    ros::Time last_received;
    double frequency;
    int count;
    
    TopicStatus(const std::string& topic_name) : 
        name(topic_name), received(false), frequency(0.0), count(0) {}
};

// 全局变量存储话题状态
std::vector<TopicStatus> topic_statuses;

// 回调函数模板
template<typename T>
void genericCallback(const typename T::ConstPtr& msg, const std::string& topic_name) {
    for (auto& status : topic_statuses) {
        if (status.name == topic_name) {
            if (!status.received) {
                status.received = true;
                ROS_INFO("First message received on topic: %s", topic_name.c_str());
            }
            status.last_received = ros::Time::now();
            status.count++;
            break;
        }
    }
}

// 特定回调函数（可根据需要添加特殊处理）
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    genericCallback<mavros_msgs::State>(msg, "/mavros/state");
    // 可以在这里添加状态信息的特殊处理
}

void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
    genericCallback<mavros_msgs::ExtendedState>(msg, "/mavros/extended_state");
}

void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    genericCallback<sensor_msgs::BatteryState>(msg, "/mavros/battery");
}

void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    genericCallback<geometry_msgs::PoseStamped>(msg, "/mavros/local_position/pose");
}

void localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    genericCallback<geometry_msgs::TwistStamped>(msg, "/mavros/local_position/velocity_local");
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    genericCallback<sensor_msgs::Imu>(msg, "/mavros/imu/data_raw");
}

void satellitesCallback(const std_msgs::UInt32::ConstPtr& msg) {
    genericCallback<std_msgs::UInt32>(msg, "/mavros/global_position/raw/satellites");
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    genericCallback<sensor_msgs::NavSatFix>(msg, "/mavros/global_position/global");
}

void gpsRawCallback(const mavros_msgs::GPSRAW::ConstPtr& msg) {
    genericCallback<mavros_msgs::GPSRAW>(msg, "/mavros/gpsstatus/gps1/raw");
}

void setpointLocalCallback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
    genericCallback<mavros_msgs::PositionTarget>(msg, "/mavros/setpoint_raw/target_local");
}

void setpointAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    genericCallback<mavros_msgs::AttitudeTarget>(msg, "/mavros/setpoint_raw/target_attitude");
}

// 请求数据流服务函数
bool requestDataStream(ros::ServiceClient& client, int message_id, int frequency_hz) {
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = message_id;
    srv.request.message_rate = frequency_hz;
    srv.request.on_off = true; // 启用消息发送
    
    if (client.call(srv)) {
        ROS_INFO("成功设置消息ID %d 的流频率为 %d Hz", message_id, frequency_hz);
        return true;
    } else {
        ROS_ERROR("设置消息ID %d 的流频率失败", message_id);
        return false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); // 设置区域为C.UTF-8以避免中文乱码问题
    ros::init(argc, argv, "fmt_topic_test");
    ros::NodeHandle nh;
    
    // 初始化话题状态
    topic_statuses = {
        TopicStatus("/mavros/state"),
        TopicStatus("/mavros/extended_state"),
        TopicStatus("/mavros/battery"),
        TopicStatus("/mavros/local_position/pose"),
        TopicStatus("/mavros/local_position/velocity_local"),
        TopicStatus("/mavros/imu/data_raw"),
        TopicStatus("/mavros/global_position/raw/satellites"),
        TopicStatus("/mavros/global_position/global"),
        TopicStatus("/mavros/gpsstatus/gps1/raw"),
        TopicStatus("/mavros/setpoint_raw/target_local"),
        TopicStatus("/mavros/setpoint_raw/target_attitude")
    };
    
    // 创建数据流服务客户端
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    
    // 等待MAVROS服务可用
    ROS_INFO("等待MAVROS服务启动...");
    ros::Duration(2.0).sleep(); // 等待2秒
    
    // 请求FMT飞控发送所需数据流
    ROS_INFO("请求FMT飞控发送数据流...");
    
    // 请求高精度IMU数据 (HIGHRES_IMU) 频率 100Hz
    requestDataStream(stream_rate_client, 105, 100);
    
    // 请求本地位置和速度数据 (LOCAL_POSITION_NED) 频率 50Hz
    requestDataStream(stream_rate_client, 32, 50);
    
    // 请求全局位置数据 (GLOBAL_POSITION_INT) 频率 10Hz
    requestDataStream(stream_rate_client, 33, 10);
    
    // 请求GPS原始数据 (GPS_RAW_INT) 频率 5Hz
    requestDataStream(stream_rate_client, 24, 5);
    
    ROS_INFO("数据流请求完成，开始监听话题...");
    
    // 创建订阅者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, stateCallback);
    
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>(
        "/mavros/extended_state", 10, extendedStateCallback);
    
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>(
        "/mavros/battery", 10, batteryCallback);
    
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 10, localPoseCallback);
    
    ros::Subscriber local_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(
        "/mavros/local_position/velocity_local", 10, localVelocityCallback);
    
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/mavros/imu/data_raw", 10, imuCallback);
    
    ros::Subscriber satellites_sub = nh.subscribe<std_msgs::UInt32>(
        "/mavros/global_position/raw/satellites", 10, satellitesCallback);
    
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 10, globalPositionCallback);
    
    ros::Subscriber gps_raw_sub = nh.subscribe<mavros_msgs::GPSRAW>(
        "/mavros/gpsstatus/gps1/raw", 10, gpsRawCallback);
    
    ros::Subscriber setpoint_local_sub = nh.subscribe<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/target_local", 10, setpointLocalCallback);
    
    ros::Subscriber setpoint_attitude_sub = nh.subscribe<mavros_msgs::AttitudeTarget>(
        "/mavros/setpoint_raw/target_attitude", 10, setpointAttitudeCallback);
    
    ROS_INFO("FMT话题测试程序已启动，正在监听话题...");
    ROS_INFO("等待10秒收集数据...");
    
    // 等待一段时间收集数据
    ros::Time start_time = ros::Time::now();
    ros::Duration test_duration(20.0); // 测试10秒
    
    ros::Rate rate(1); // 1Hz更新显示
    
    while (ros::ok() && (ros::Time::now() - start_time) < test_duration) {
        ros::spinOnce();
        
        // 更新频率计算
        for (auto& status : topic_statuses) {
            if (status.received) {
                ros::Duration elapsed = ros::Time::now() - start_time;
                status.frequency = status.count / elapsed.toSec();
            }
        }
        
        // 显示当前状态
        ROS_INFO("=== FMT话题接收状态 ===");
        for (const auto& status : topic_statuses) {
            if (status.received) {
                ROS_INFO("%s: ✓ (%.2f Hz, %d msgs)", 
                         status.name.c_str(), status.frequency, status.count);
            } else {
                ROS_INFO("%s: ✗ (未收到消息)", status.name.c_str());
            }
        }
        ROS_INFO("======================");
        
        rate.sleep();
    }
    
    // 最终报告
    ROS_INFO("=== 测试完成 - 最终结果 ===");
    int received_count = 0;
    for (const auto& status : topic_statuses) {
        if (status.received) {
            ROS_INFO("%s: ✓ (%.2f Hz)", status.name.c_str(), status.frequency);
            received_count++;
        } else {
            ROS_INFO("%s: ✗ (未收到消息)", status.name.c_str());
        }
    }
    
    ROS_INFO("收到消息的话题: %d/%d", received_count, topic_statuses.size());
    
    if (received_count == topic_statuses.size()) {
        ROS_INFO("✅ 所有话题都正常接收消息！");
    } else {
        ROS_INFO("⚠️  部分话题未收到消息，请检查FMT配置");
    }
    
    return 0;
}
