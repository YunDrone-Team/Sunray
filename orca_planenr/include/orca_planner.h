// #ifndef ORCA_PLANNER_H
// #define ORCA_PLANNER_H

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <tf2/utils.h>

#include "RVO/RVO.h"

using namespace std;

// namespace orca_planner
// {

class OrcaPlanner
{
public:
    ros::NodeHandle nh;

    OrcaPlanner();

    ~OrcaPlanner();

    void initialize(ros::NodeHandle& nh);

    bool setPlan(const std::vector<RVO::Vector2> &orig_global_plan);

    bool computeVelocityCommands(std::vector<geometry_msgs::Twist> &cmd_vel);

    bool isGoalReached();

private:
    
    bool initialized_, odom_flag_, goal_reached_;

    int agent_number_, agent_id_;                                      // id begin from 1
    double d_t_;                                                       // control time step
    double neighbor_dist_, time_horizon_, time_horizon_obst_, radius_; // orca parameters
    int max_neighbors_;
    // if the distance is less than the tolerance value, it is considered to have reached the target position
    double goal_dist_tol_;
    // if the angle deviation is greater than this threshold, perform rotation first
    double rotate_tol_;

    double max_v_, min_v_, max_v_inc_; // linear velocity
    double max_w_, min_w_, max_w_inc_; // angular velocity

    std::string uav_name{""};
    int uav_num;

    RVO::RVOSimulator *sim_;
    std::vector<RVO::Vector2> goal_;
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<nav_msgs::Odometry> other_odoms_;

    void odometryCallback(const nav_msgs::OdometryConstPtr &msg, int agent_id);

    bool initState();

    void updateState();

    double regularizeAngle(double angle);

    double linearRegularization(nav_msgs::Odometry &base_odometry, double v_d);

    double angularRegularization(nav_msgs::Odometry &base_odometry, double w_d);
};
// }; // namespace orca_planner

// #endif