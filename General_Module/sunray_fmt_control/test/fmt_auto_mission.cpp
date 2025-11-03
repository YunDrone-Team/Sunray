#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/StreamRate.h>

// 飞行参数
#define FLIGHT_ALTITUDE 1.5f  // 飞行高度1.5米
#define RATE 20               // 循环频率20Hz
#define HOVER_TIME 10.0f       // 悬停时间10秒
#define TAKEOFF_TIMEOUT 20.0f // 起飞超时30秒
#define UNLOCK_DELAY 1        // 延迟起飞时间

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool has_pose_data = false;  // 标记是否接  收到位置数据

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) { 
    current_pose = *msg; 
    double cur_enu_x = current_pose.pose.position.x;
    double cur_enu_y = current_pose.pose.position.y;
    double cur_enu_z = current_pose.pose.position.z;
    ROS_INFO_THROTTLE(1.0,"X=%.3f y=%.3f z=%.3f",cur_enu_x,cur_enu_y,cur_enu_z); 
}

// 设置数据流请求的函数
bool setStreamRate(ros::NodeHandle& nh, uint8_t stream_id, uint16_t message_rate, bool on_off) {
    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    
    // 等待服务可用
    if (!stream_rate_client.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Stream rate service not available");
        return false;
    }
    
    mavros_msgs::StreamRate srv;
    srv.request.stream_id = stream_id;
    srv.request.message_rate = message_rate;
    srv.request.on_off = on_off;
    
    if (stream_rate_client.call(srv)) {
        ROS_INFO("Stream rate set successfully for stream ID %d at %d Hz", stream_id, message_rate);
        return true;
    } else {
        ROS_ERROR("Failed to set stream rate for stream ID %d", stream_id);
        return false;
    }
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, ""); // 设置本地化环境，确保中文输出正常
    ros::init(argc, argv, "fmt_auto_mission");
    ros::NodeHandle nh;
    
    ROS_INFO("简单起飞降落演示程序");

    // 设置数据流请求 - 请求本地位置数据
    ROS_INFO("配置数据流请求...");
    if (!setStreamRate(nh, 32, 50, true)) {  // MAVLINK_MSG_ID_LOCAL_POSITION_NED = 32
        ROS_WARN("未能成功配置本地位置数据流，可能会影响飞行控制");
    }

    // 也可以请求其他可能需要的数据流
    // setStreamRate(nh, 105, 100, true);  // MAVLINK_MSG_ID_HIGHRES_IMU
    // setStreamRate(nh, 30, 50, true);    // MAVLINK_MSG_ID_ATTITUDE

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    ros::Rate rate(RATE);

    // 等待与飞控连接
    ROS_INFO("等待与飞控（FCU）连接...");
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("已连接到飞控！");

    // // 等待位置信息可用
    // ROS_INFO("等待位置信息...");
    // ros::Time pose_wait_start = ros::Time::now();
    // while(ros::ok() && (ros::Time::now() - pose_wait_start).toSec() < 10.0 && !has_pose_data) {
    //     ROS_INFO_THROTTLE(1.0, "等待接收位置数据...");
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
    // if (!has_pose_data) {
    //     ROS_ERROR("未能接收到位置数据，程序退出");
    //     return 1;
    // }
    
    // ROS_INFO("已接收到位置数据，当前高度: %.3f米", current_pose.pose.position.z);

    // // 记录初始位置
    // float initial_x = current_pose.pose.position.x;
    // float initial_y = current_pose.pose.position.y;
    // float initial_z = current_pose.pose.position.z;
    // ROS_INFO("记录初始位置: (%.3f, %.3f, %.3f)", initial_x, initial_y, initial_z);

    // 准备设定点消息
    mavros_msgs::PositionTarget setpoint_msg;
    setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_msg.type_mask = 
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint_msg.position.x = 0;
    setpoint_msg.position.y = 0;
    setpoint_msg.position.z = 0; // 初始高度为0
    setpoint_msg.yaw = 0;

    // 预发布设定点，确保OFFBOARD模式可切换
    ROS_INFO("预发布设定点...");
    for(int i = 100; ros::ok() && i > 0; --i) {
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("设定点预发布完成");

    // 切换到OFFBOARD模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    ros::Time last_request = ros::Time::now();
    bool offboard_enabled = false;
    
    while(ros::ok() && !offboard_enabled) {
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        // 只要当前已经是OFFBOARD就直接通过
        if(current_state.mode == "OFFBOARD") {
            ROS_INFO("已处于OFFBOARD模式");
            offboard_enabled = true;
        } else if(ros::Time::now() - last_request > ros::Duration(2.0)) {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("已切换到OFFBOARD模式");
                offboard_enabled = true;
            } else {
                ROS_WARN("OFFBOARD模式切换失败，重试中...");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 解锁无人机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    bool armed = false;
    
    while(ros::ok() && !armed) {
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        
        if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("无人机已解锁");
                armed = true;
            } else {
                ROS_WARN("解锁失败，重试中...");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 解锁后延迟2秒 - 添加的延迟
    ROS_INFO("解锁完成，等待%d秒后起飞...", UNLOCK_DELAY);
    ros::Time unlock_time = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - unlock_time).toSec() < UNLOCK_DELAY) {
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // 起飞到指定高度
    ROS_INFO("起飞到%.1f米高度...", FLIGHT_ALTITUDE);
    ros::Time takeoff_start = ros::Time::now();

    double ground_z = current_pose.pose.position.z;
    double desired_alt = ground_z + FLIGHT_ALTITUDE;
    // 比例控制参数 (根据无人机特性调整)
    double Kp = 1.0;               // 比例系数
    double max_altitude_rate = 0.6; // 最大爬升速率限制 (m/s)
    double min_altitude_rate = 0.1; // 最小爬升速率 (避免停滞)

    while (ros::ok()) {
        
        // 目标高度（正向向上），但发送到 PX4 的 NED z 应为 -altitude
        
        double cur_enu_z = current_pose.pose.position.z; 
        double error = desired_alt-cur_enu_z;

        // 比例控制计算: 输出与误差成正比
        double altitude_rate = Kp * error;
        // 限制爬升速率在安全范围内 (防止输出过大)
        altitude_rate = std::max(std::min(altitude_rate, max_altitude_rate), min_altitude_rate);

        // 计算当前目标高度 (基于速率积分)
        double elapsed_time = (ros::Time::now() - takeoff_start).toSec();
        double current_target_alt = std::min(
            cur_enu_z + altitude_rate * elapsed_time,  
            desired_alt
        );
        setpoint_msg.position.z = current_target_alt; 
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);

        ros::spinOnce();
        // ENU z = 高度（m）
        ROS_INFO_THROTTLE(1.0, "-----当前相对高度 (ENU): %.3f m, 相对目标: %.3f m-----", cur_enu_z, desired_alt);
        //检查是否达到高度
        if (cur_enu_z >= desired_alt * 0.95) {
            ROS_INFO("已接近目标高度");
            break;
        }
        //检查是否超时
        if ((ros::Time::now() - takeoff_start).toSec() > TAKEOFF_TIMEOUT) {
            ROS_WARN("起飞超时，跳出起飞循环");
            break;
        }
        rate.sleep();
    }
    
    // 保持目标高度
    
    ROS_INFO("到达目标高度，保持悬停%.1f秒...", HOVER_TIME);
    
    ros::Time hover_start = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - hover_start).toSec() < HOVER_TIME) {
        setpoint_msg.position.z = desired_alt;
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // 更平滑的 setpoint 降落
    ROS_INFO("开始平滑降落...");
    float descend_rate = 0.20f; // 更慢的降落速度（米/秒）
    float descend_step = 0.10f; // 每步最大高度变化（米）
    float target_z = desired_alt;
    ros::Time descend_start = ros::Time::now();

    while(ros::ok() && target_z > 0.1f) {
        // 计算下一步目标高度
        float elapsed = (ros::Time::now() - descend_start).toSec();
        float expect_z = desired_alt - elapsed * descend_rate;
        // 限制每步最大高度变化
        if (target_z - expect_z > descend_step) {
            target_z -= descend_step;
        } else {
            target_z = expect_z;
        }

        if(target_z < 0.0f) target_z = 0.0f;


        setpoint_msg.position.z = target_z;
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(setpoint_msg);
        ROS_INFO_THROTTLE(1.0, "当前高度: %.2f米", target_z);
        ros::spinOnce();
        rate.sleep();
    }

    // 上锁
    arm_cmd.request.value = false;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("无人机已上锁");
    } else {
        ROS_WARN("上锁失败");
    }

    ROS_INFO("任务完成，程序退出");
    return 0;
}
