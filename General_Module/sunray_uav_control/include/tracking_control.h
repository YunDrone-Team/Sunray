#include <ros/ros.h>
#include <Eigen/Dense>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <sunray_msgs/Target.h>
#include "utils.h"

class trackingController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber odomSub_;             // Subscribe to odometry
    ros::Subscriber imuSub_;              // IMU data subscriber
    ros::Subscriber targetSub_;           // subscriber for the tracking target states
    ros::Publisher cmdPub_;               // command publisher
    ros::Publisher accCmdPub_;            // acceleration command publisher
    ros::Publisher posCmdPub_;            // position command publisher
    ros::Publisher velCmdPub_;            // position command publisher
    ros::Timer cmdTimer_;                 // command timer

    // parameters
    bool bodyRateControl_ = false;
    bool attitudeControl_ = false;
    bool posControl_ = false;
    bool velControl_ = false;
    bool accControl_ = true;


    // controller data
    bool odomReceived_ = false;
    bool imuReceived_ = false;
    bool firstTargetReceived_ = false;
    bool targetReceived_ = false;
    bool firstTime_ = true;
    nav_msgs::Odometry odom_;
    sensor_msgs::Imu imuData_;
    sunray_msgs::Target target_;
    ros::Time prevTime_;
    double deltaTime_;

    // kalman filter
    bool kfFirstTime_ = true;
    ros::Time kfStartTime_;
    double stateVar_ = 0.01;
    double processNoiseVar_ = 0.01;
    double measureNoiseVar_ = 0.02;
    std::deque<double> prevEstimateThrusts_;

    std::string topic_name;               // 节点名称
    std::string uav_name{""};            // 无人机名称
    int uav_id;                     // 无人机编号

public:
    trackingController(const ros::NodeHandle &nh);
    void initParam();
    void registerPub();
    void registerCallback();

    // callback functions
    void odomCB(const nav_msgs::OdometryConstPtr &odom);
    void imuCB(const sensor_msgs::ImuConstPtr &imu);
    void targetCB(const sunray_msgs::TargetConstPtr &target);
    void cmdCB(const ros::TimerEvent &);

    void publishCommandPos(const Eigen::Vector3d &posRef);
    void publishCommandVel(const Eigen::Vector3d &velRef);
    void positionControl(const geometry_msgs::Vector3 &targetPosition, Eigen::Vector3d &cmd);
};