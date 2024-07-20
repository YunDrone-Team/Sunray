#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include "RVO.h"

using namespace std;
class changingFormation
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odomSub_;               // Subscribe to odometry
    ros::Subscriber mocapSub_;              // Subscribe to mocap
    ros::Subscriber gazeboSub_;             // Subscribe to gazebo
    ros::Publisher posCmdPub_;              // position command publisher
    ros::Publisher velCmdPub_;              // position command publisher

    ros::Timer cmdTimer_;                   // command timer
    ros::Time last_pose_stamp{0};

    // parameters
    bool posControl_ = false;
    bool velControl_ = false;

    // controller data
    bool odomReceived_ = false;
    nav_msgs::Odometry odom_;
    Eigen::Vector3d pos_from_external;
    ros::Time cmdThrustTime_;

    RVO::RVOSimulator *simulator;
    std::vector<RVO::Vector2> goals;

    std::string topic_name;               // 节点名称
    std::string uav_name{""};            // 无人机名称
    int uav_id;                     // 无人机编号

public:
    changingFormation(const ros::NodeHandle &nh);

    // callback functions
    void odomCB(const nav_msgs::OdometryConstPtr &odom);
    void mocapCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void gazeboCB(const nav_msgs::Odometry::ConstPtr &msg);

    void setupScenario(RVO::RVOSimulator *simulator, std::vector<RVO::Vector2> &goals);
    void setPreferredVelocities(RVO::RVOSimulator *simulator, const std::vector<RVO::Vector2> &goals);
    bool reachedGoal(RVO::RVOSimulator *simulator, const std::vector<RVO::Vector2> &goals);
    void run(RVO::RVOSimulator *simulator_, const std::vector<RVO::Vector2> &goals);

    void publishCommandPos(const Eigen::Vector3d &posRef);
    void publishCommandVel(const Eigen::Vector3d &velRef);
    void positionControl(const geometry_msgs::Vector3 &targetPosition, Eigen::Vector3d &cmd);
};