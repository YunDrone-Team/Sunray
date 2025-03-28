// #include <ros/ros.h>
// #include <Eigen/Eigen>
// #include <sstream>
// #include <iostream>
// #include <cmath>
// #include <boost/thread.hpp> 

// #include "ros_msg_utils.h"

// // 圆形轨迹
// class CircularTrajectory {
//     double radius, center_x, center_y, altitude, angular_velocity;

// public:
//     CircularTrajectory(){}
//     void ci_init(double r, double x, double y, double z, double w){
//         radius = r;
//         center_x = x;
//         center_y = y;
//         altitude = z; 
//         angular_velocity = w; 
//     }
//     Eigen::Vector3d calculate_pose(double time) {
//         Eigen::Vector3d pose(0,0,0);
//         double theta = angular_velocity * time;
//         pose[0] = center_x + radius * cos(theta);
//         pose[1] = center_y + radius * sin(theta);
//         pose[2] = altitude;
//         return pose;
//     }

//     Eigen::Vector3d calculate_velocity(double time) {
//         Eigen::Vector3d vel(0,0,0);
//         double theta = angular_velocity * time;
//         vel[0] = -radius * angular_velocity * sin(theta);
//         vel[1] = radius * angular_velocity * cos(theta);
//         vel[2] = 0;
//         return vel;
//     }
// };

// class TrajectoryPublisher {
// private:
    

// public:
//     ros::Publisher setup_pub_;
//     ros::Publisher cmd_pub_;
//     ros::Timer pub_trajectory_;
//     sunray_msgs::UAVSetup setup_;
//     sunray_msgs::UAVControlCMD cmd_;
//     CircularTrajectory cicle_;
//     ros::Time start_time_;
//     int mode_ = 0;
//     Eigen::Vector3d input;
//     Eigen::Vector3d pos_;
//     Eigen::Vector3d vel_;

//     boost::thread run_thread_;

//     TrajectoryPublisher(ros::NodeHandle &nh_) {
        
//         pub_trajectory_ = nh_.createTimer(ros::Duration(0.1), &TrajectoryPublisher::pub_trajectory_cb, this);
//         setup_pub_ = nh_.advertise<sunray_msgs::UAVSetup>("/uav1/sunray/setup",1);
//         cmd_pub_ = nh_.advertise<sunray_msgs::UAVControlCMD>("/uav1/sunray/uav_control_cmd",1);
//         cicle_.ci_init(1, 0, 0, 1, 0.5);
//     }

//     void pub_trajectory_cb(const ros::TimerEvent& e) {
//         // std::cout<<ros::Time::now().toSec()<<std::endl;
//         switch (mode_) {
//             case 1: // move pose
//                 cmd_.cmd = 4;
//                 cmd_.desired_pos[0] = pos_[0];
//                 cmd_.desired_pos[1] = pos_[1];
//                 cmd_.desired_pos[2] = pos_[2];
//                 cmd_.enable_yawRate = false;
//                 cmd_.desired_yaw = 0;
//                 cmd_pub_.publish(cmd_);
//                 break;
//             case 2: // circle pose
//             {
//                 double elapsed = (ros::Time::now() - start_time_).toSec();
//                 Eigen::Vector3d pose = cicle_.calculate_pose(elapsed);
//                 // std::cout<<"pos:"<< pose <<std::endl;
//                 cmd_.cmd = 4;
//                 cmd_.desired_pos[0] = pose[0];
//                 cmd_.desired_pos[1] = pose[1];
//                 cmd_.desired_pos[2] = pose[2];
//                 cmd_.enable_yawRate = false;
//                 cmd_.desired_yaw = 0;
//                 cmd_pub_.publish(cmd_);
//                 break;
//             }
//             case 3: // circle vel
//             {
//                 double elapsed = (ros::Time::now() - start_time_).toSec();
//                 Eigen::Vector3d vel = cicle_.calculate_velocity(elapsed);
//                 cmd_.cmd = 5;
//                 cmd_.desired_vel[0] = vel[0];
//                 cmd_.desired_vel[1] = vel[1];
//                 cmd_.desired_vel[2] = 0;
//                 cmd_.desired_pos[2] = 1;
//                 cmd_.enable_yawRate = false;
//                 cmd_.desired_yaw = 0;
//                 cmd_pub_.publish(cmd_);
//                 break;
//             }
//             default:
//                 break;
//         }
//     }

//     void run() {
//         while (ros::ok()) {
//             int tmp;
//             std::cout << "Please select the operation mode 1: arming 2: set cmd mode 3: takeoff 4: hover 5: move 6: land" << std::endl;
//             std::cin >> tmp;
//             switch (tmp) {
//                 case 1:
//                     int arming;
//                     std::cout << "Please select Operation 1 arm 0 disarm" << std::endl;
//                     std::cin >> arming;
//                     if (arming!= 1 && arming!= 0) {
//                         std::cout << "input error" << std::endl;
//                     } else {
//                         setup_.cmd = 0;
//                         setup_.arming = arming;
//                         setup_pub_.publish(setup_);
//                     }
//                     mode_ = 0;
//                     break;
//                 case 2:
//                     setup_.cmd = 3;
//                     setup_.control_mode = "CMD_CONTROL";
//                     setup_pub_.publish(setup_);
    
//                     break;
//                 case 3:
//                     cmd_.cmd = 1;
//                     cmd_pub_.publish(cmd_);
//                     mode_ = 0;
//                     break;
//                 case 4:
//                     cmd_.cmd = 2;
//                     cmd_pub_.publish(cmd_);
//                     mode_ = 0;
//                     break;
//                 case 5:
//                     int op;
//                     std::cout << "Please select Operation 1: move pose 2: circle pose 3: circle vel" << std::endl;
//                     std::cin >> op;
//                     if (op == 1) {
//                         std::cout << "input x (m)" << std::endl;
//                         std::cin >> input[0];
//                         std::cout << "input y (m)" << std::endl;
//                         std::cin >> input[1];
//                         std::cout << "input z (m)" << std::endl;
//                         std::cin >> input[2];
//                         if (abs(input[0]) < 5 && abs(input[1]) < 5 && abs(input[2]) < 2) {
//                             mode_ = 1;
//                             pos_ = input;
//                         }
//                     } else if (op == 2) {
//                         start_time_ = ros::Time::now();
//                         mode_ = 2;
//                     } else if (op == 3) {
//                         start_time_ = ros::Time::now();
//                         mode_ = 3;
//                     }
//                     break;
//                 case 6:
//                     cmd_.cmd = 3;
//                     cmd_pub_.publish(cmd_);
//                     mode_ = 0;
//                     break;
//                 default:
//                     break;
//             }
//         }
//     }

//     void start() {
//         run_thread_ = boost::thread(&TrajectoryPublisher::run, this); // Start the run thread
//     }
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "terminal_control_node");
//     ros::NodeHandle nh("~");
//     TrajectoryPublisher tp(nh);
//     tp.start(); // Start the run thread
//     ros::spin(); // Keep the main thread alive
//     return 0;
// }