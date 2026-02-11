#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <csignal>
#include <condition_variable>
#include "imu_process.hpp"
#include "ekf_filter.hpp"

typedef std::pair<double, Eigen::Matrix4d> OdomType;
typedef std::pair<double, ImuData> ImuType;

std::deque<ImuType> imu_buffer;
std::deque<OdomType> odom_buffer;
std::mutex mtx_buffer;

bool exit_flag = false;
std::condition_variable sig_buffer;


void SigHandle(int sig) {

    exit_flag = true;
    ROS_WARN("Catch sig %d", sig);
    sig_buffer.notify_all();
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    
    std::lock_guard<std::mutex> lock(mtx_buffer);
    double time = msg->header.stamp.toSec();

    ImuData imu_data;
    imu_data.cur_acc = Eigen::Vector3d(msg->linear_acceleration.x,
                                       msg->linear_acceleration.y,
                                       msg->linear_acceleration.z);
    imu_data.cur_gyr = Eigen::Vector3d(msg->angular_velocity.x,
                                       msg->angular_velocity.y,        
                                       msg->angular_velocity.z);
    imu_data.timestamp = time;
    imu_buffer.push_back(ImuType(time, imu_data));
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    std::lock_guard<std::mutex> lock(mtx_buffer);
    double time = msg->header.stamp.toSec();
    Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
    odom.block<3,3>(0,0) = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                              msg->pose.pose.orientation.x,
                                              msg->pose.pose.orientation.y,
                                              msg->pose.pose.orientation.z).toRotationMatrix(); 

    odom.block<3,1>(0,3) = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    odom_buffer.push_back(OdomType(time, odom));
}
   
void PublishOdometry(ros::Publisher& odom_pub, const EkfState& state) {

    nav_msgs::Odometry odom_msg;
    
    odom_msg.header.stamp = ros::Time(state.timestamp);
    
    // Set position
    odom_msg.pose.pose.position.x = state.pos.x();
    odom_msg.pose.pose.position.y = state.pos.y();
    odom_msg.pose.pose.position.z = state.pos.z();
    
    // Set orientation
    odom_msg.pose.pose.orientation.w = state.rot.w();
    odom_msg.pose.pose.orientation.x = state.rot.x();
    odom_msg.pose.pose.orientation.y = state.rot.y();
    odom_msg.pose.pose.orientation.z = state.rot.z();
    
    // Set velocity
    odom_msg.twist.twist.linear.x = state.vel.x();
    odom_msg.twist.twist.linear.y = state.vel.y();
    odom_msg.twist.twist.linear.z = state.vel.z();
    
    odom_pub.publish(odom_msg);
}
    
int main(int argc, char** argv) {

    ros::init(argc, argv, "ekf_odometry_node");

    ros::NodeHandle nh("~");
        
    std::string imu_topic, odom_topic;

    nh.param<std::string>("imu_topic", imu_topic, "/livox/imu");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");

    // Setup subscriber
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, ImuCallback);
    ros::Subscriber odom_sub = nh.subscribe(odom_topic, 10, OdomCallback);
        
    // Setup publisher
    ros::Publisher ekf_odom_pub = nh.advertise<nav_msgs::Odometry>("/sunray/ekf_odometry", 10);
        
    ROS_INFO("EKF Odometry Node Starting...");
    ROS_INFO("Subscribing to IMU topic: %s", imu_topic.c_str());
    ROS_INFO("Subscribing to Odometry topic: %s", odom_topic.c_str());
    ROS_INFO("Publishing to: /sunray/ekf_odometry");

    std::shared_ptr<EkfFilter> ekf_filter = std::make_shared<EkfFilter>();
    std::shared_ptr<ImuProcess> imu_process = std::make_shared<ImuProcess>();

    // TODO:初始化
    ros::Rate rate(200);    
    
    bool imu_init_flag = false, system_initialized = false;

    while (ros::ok()) {

        if (exit_flag)
            break;

        ros::spinOnce();

        // 初始化IMU重力，陀螺仪偏置
        if (!imu_init_flag) {
            
            while (!imu_buffer.empty()) {

                auto cur_imu = imu_buffer.front();
                imu_buffer.pop_front();

                if (imu_process->ImuInit(cur_imu.second)) {

                    ekf_filter->SetInitGyrBias(imu_process->GetInitGyrBias());
                    imu_init_flag = true;
                    ROS_INFO("Imu Initialized!");
                    break;
                }
            }

            continue;
        }

        // 初始化EKF
        if (!system_initialized) {

             while (!imu_buffer.empty() && !odom_buffer.empty()) {

                auto cur_imu = imu_buffer.front();
                auto cur_odom = odom_buffer.front();
                
                double dt = cur_imu.first - cur_odom.first;

                if (dt < 0) {
                    
                    imu_buffer.pop_front();

                } else {
                
                    if (dt > 0.1) {

                        odom_buffer.pop_front();

                    } else {

                        ekf_filter->SetInitState(cur_odom.second);  //设置初始值
                        ImuData imu_data;
                        imu_process->ProcssIMU(cur_imu.second, imu_data);
                        imu_buffer.pop_front();
                        system_initialized = true;
                        ROS_INFO("EKF Initialized!");
                        break;
                    }
                }
            }

            continue;
        }

        // EKF预测-更新循环
        while (!imu_buffer.empty() && !odom_buffer.empty()) {

            auto cur_imu = imu_buffer.front();
            auto cur_odom = odom_buffer.front();

            if (cur_imu.first < cur_odom.first) {

                ImuData imu_data;
                imu_process->ProcssIMU(cur_imu.second, imu_data);
                imu_buffer.pop_front();
                ekf_filter->Predict(ekf_state, imu_data);
                PublishOdometry(ekf_odom_pub, ekf_filter->GetState());

            } else {
            
                ekf_filter->Update(ekf_state, cur_odom.second);
                odom_buffer.pop_front();
            }
        }

        rate.sleep();
    }
  
    return 0;
}



      
