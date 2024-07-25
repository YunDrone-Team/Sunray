#include <ros/ros.h>
#include "orca_planner.h"

// namespace orca_planner
// {

OrcaPlanner::OrcaPlanner(): initialized_(false), odom_flag_(false), goal_reached_(false)
{
}


OrcaPlanner::~OrcaPlanner()
{
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void OrcaPlanner::initialize(ros::NodeHandle& nh)
{
    if (!initialized_)
    {
        initialized_ = true;

        // base
        nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
        nh.param("rotate_tolerance", rotate_tol_, 0.5);
        nh.param<std::string>("uav_name", uav_name, "uav");
        nh.param("uav_num", uav_num, 1);
        // multi-robot info
        nh.param("agent_number", agent_number_, -1);
        nh.param("agent_id", agent_id_, -1);

        // linear velocity
        nh.param("max_v", max_v_, 0.5);
        nh.param("min_v", min_v_, 0.0);
        nh.param("max_v_inc", max_v_inc_, 0.5);

        // angular velocity
        nh.param("max_w", max_w_, 1.57);
        nh.param("min_w", min_w_, 0.0);
        nh.param("max_w_inc", max_w_inc_, 1.57);

        // orca parameters
        nh.param("neighbor_dist", neighbor_dist_, 3.0);
        nh.param("time_horizon", time_horizon_, 5.0);
        nh.param("time_horizon_obst", time_horizon_obst_, 3.0);
        nh.param("radius", radius_, 0.2);
        nh.param("max_neighbors", max_neighbors_, 10);

        other_odoms_.resize(agent_number_);
        for (int i = 0; i < agent_number_; ++i)
        {
            ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
                "/" + uav_name + std::to_string(i + 1) + "/mavros/local_position/odom", 1, boost::bind(&OrcaPlanner::odometryCallback, this, _1, i + 1));
            odom_subs_.push_back(odom_sub);
            ROS_INFO("agent %d, subscribe to agent %d.", agent_id_, i + 1);
        }

        int spin_cnt = 5 * agent_number_;
        ros::Rate rate(10);
        ROS_WARN("[ORCA] Waiting for odoms...");
        while (spin_cnt-- > 0)
        {
            ros::spinOnce();
            rate.sleep();
        }

        double controller_freqency;
        nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
        d_t_ = 1 / controller_freqency;

        sim_ = new RVO::RVOSimulator();
        while(!initState());
        
        ROS_INFO("ORCA planner initialized!");
    }
    else
    {
        ROS_WARN("ORCA planner has already been initialized.");
    }
}

bool OrcaPlanner::setPlan(const std::vector<RVO::Vector2> &orig_global_plan)
{
    goal_ = orig_global_plan;
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    ROS_INFO("Got new plan");

    goal_reached_ = false;

    return true;
}

bool OrcaPlanner::computeVelocityCommands(std::vector<geometry_msgs::Twist> &cmd_vel)
{   bool all_reached = true;
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    for (int i = 0; i < agent_number_; ++i)
    {
        nav_msgs::Odometry agent_odom = other_odoms_[i];
        RVO::Vector2 curr_pose(agent_odom.pose.pose.position.x, agent_odom.pose.pose.position.y);
        if (RVO::abs(goal_[i] - curr_pose) < goal_dist_tol_)
        {
            cmd_vel[i].linear.x = 0.0;
            cmd_vel[i].linear.y = 0.0;
        }
        else
        {
            updateState();
            RVO::Vector2 new_speed = sim_->getAgentNewSpeed(i);

            // transform from linear.x and linear.y to linear.x and angular.z
            // double theta = tf2::getYaw(agent_odom.pose.pose.orientation);

            // double v_d = RVO::abs(new_speed);
            // double theta_d = std::atan2(new_speed.y(), new_speed.x());
            // double e_theta = regularizeAngle(theta_d - theta);
            // double w_d = e_theta / d_t_;

            cmd_vel[i].linear.x = new_speed.x();
            cmd_vel[i].linear.y = new_speed.y();
            all_reached = false;
        }
    }
    
    if(all_reached){
        goal_reached_ = true;
    }
    return true;
}

bool OrcaPlanner::isGoalReached()
{
    if (!initialized_)
    {
        ROS_ERROR("ORCA planner has not been initialized");
        return false;
    }

    if (goal_reached_)
    {
        ROS_INFO("GOAL Reached!");
        return true;
    }
    return false;
}

bool OrcaPlanner::initState()
{
    if (!odom_flag_)
    {
        ROS_ERROR("Odom not received!");
        return false;
    }

    sim_->setTimeStep(d_t_);
    sim_->setAgentDefaults(neighbor_dist_, max_neighbors_, time_horizon_, time_horizon_obst_, radius_, max_v_);

    for (int i = 0; i < agent_number_; ++i)
    {
        sim_->addAgent(RVO::Vector2(other_odoms_[i].pose.pose.position.x, other_odoms_[i].pose.pose.position.y));
        sim_->setAgentVelocity(i, RVO::Vector2(other_odoms_[i].twist.twist.linear.x, other_odoms_[i].twist.twist.linear.y));
    }
    ROS_INFO("init State OK!");
    return true;
}

void OrcaPlanner::odometryCallback(const nav_msgs::OdometryConstPtr &msg, int agent_id)
{
    if (!odom_flag_)
        odom_flag_ = true;

    other_odoms_[agent_id - 1] = *msg;
}

void OrcaPlanner::updateState()
{
    for (int i = 0; i < agent_number_; ++i)
    {
        sim_->setAgentPosition(i, RVO::Vector2(other_odoms_[i].pose.pose.position.x, other_odoms_[i].pose.pose.position.y));
        sim_->setAgentVelocity(i, RVO::Vector2(other_odoms_[i].twist.twist.linear.x, other_odoms_[i].twist.twist.linear.y));

        sim_->setAgentPrefVelocity(i, RVO::normalize(goal_[i] - sim_->getAgentPosition(i)) * max_v_);
    }

    
}

double OrcaPlanner::regularizeAngle(double angle)
{
    return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

double OrcaPlanner::linearRegularization(nav_msgs::Odometry &base_odometry, double v_d)
{
    double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
    double v_inc = v_d - v;

    if (std::fabs(v_inc) > max_v_inc_)
        v_inc = std::copysign(max_v_inc_, v_inc);

    double v_cmd = v + v_inc;
    if (std::fabs(v_cmd) > max_v_)
        v_cmd = std::copysign(max_v_, v_cmd);
    else if (std::fabs(v_cmd) < min_v_)
        v_cmd = std::copysign(min_v_, v_cmd);

    return v_cmd;
}

double OrcaPlanner::angularRegularization(nav_msgs::Odometry &base_odometry, double w_d)
{
    if (std::fabs(w_d) > max_w_)
        w_d = std::copysign(max_w_, w_d);

    double w = base_odometry.twist.twist.angular.z;
    double w_inc = w_d - w;

    if (std::fabs(w_inc) > max_w_inc_)
        w_inc = std::copysign(max_w_inc_, w_inc);

    double w_cmd = w + w_inc;
    if (std::fabs(w_cmd) > max_w_)
        w_cmd = std::copysign(max_w_, w_cmd);
    else if (std::fabs(w_cmd) < min_w_)
        w_cmd = std::copysign(min_w_, w_cmd);

    return w_cmd;
}

// } // namespace orca_planner