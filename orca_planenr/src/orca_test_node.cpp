#include <ros/ros.h>
#include <sunray_msgs/OrcaCmd.h>
#include "orca_planner.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "orca_planner_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    OrcaPlanner orca_planner;
    orca_planner.initialize(nh);

    std::vector<RVO::Vector2> global_plan;
    RVO::Vector2 pose;
    pose = RVO::Vector2(10,0);
    global_plan.push_back(pose);
    global_plan.push_back(pose);
    global_plan.push_back(pose);

    orca_planner.setPlan(global_plan);

    ros::Publisher orca_cmd_pub = nh.advertise<sunray_msgs::OrcaCmd>("orca_cmd", 10); // Create a publisher for the custom message

    while (!orca_planner.isGoalReached())
    {
        std::vector<geometry_msgs::Twist> cmd_vel;
        orca_planner.computeVelocityCommands(cmd_vel);

        sunray_msgs::OrcaCmd orca_cmd; // Create an instance of the custom message
        orca_cmd.agents = 3; // Set the number of agents
        orca_cmd.twists.resize(3); // Resize the twists vector to hold 3 elements

        for(int i=0; i<3; i++){
            orca_cmd.twists[i].linear.x = cmd_vel[i].linear.x;
            orca_cmd.twists[i].linear.y = cmd_vel[i].linear.y;
            ROS_INFO("Velocity commands %d: linear.x = %f, linear.y = %f", i, cmd_vel[i].linear.x, cmd_vel[i].linear.y);
        }

        orca_cmd_pub.publish(orca_cmd); // Publish the custom message

        rate.sleep();
    }
    ROS_INFO("Goal reached!");

    return 0;
}