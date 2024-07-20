#include "trajectory_builder.h"

using namespace std;

trajectoryBuilder::trajectoryBuilder(){};

void trajectoryBuilder::init(const ros::NodeHandle &nh)
{
    nh_ = nh;
    // Subscriber

    // Publisher
    this->statePub_ = this->nh_.advertise<sunray_msgs::Target>("/sunray/trajectory/target_state", 1000);

    // Tareget publish thread
    this->targetPubWorker_ = std::thread(&trajectoryBuilder::publishTarget, this);
    this->targetPubWorker_.detach();

    // state update callback (velocity and acceleration)
    this->stateUpdateTimer_ = this->nh_.createTimer(ros::Duration(0.033), &trajectoryBuilder::stateUpdateCB, this);
}

void trajectoryBuilder::publishTarget()
{
    ros::Rate r(200);
    while (ros::ok())
    {

        if (this->poseControl_)
        {
            this->statePub_.publish(this->stateTgt_);
        }
        // ros::spinOnce();
        r.sleep();
    }
}


void trajectoryBuilder::stateUpdateCB(const ros::TimerEvent &)
{
    Eigen::Vector3d currVelBody(this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
    Eigen::Vector4d orientationQuat(this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
    Eigen::Matrix3d orientationRot = quat2RotMatrix(orientationQuat);
    this->currVel_ = orientationRot * currVelBody;
    ros::Time currTime = ros::Time::now();
    if (this->stateUpdateFirstTime_)
    {
        this->currAcc_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        this->prevStateTime_ = currTime;
        this->stateUpdateFirstTime_ = false;
    }
    else
    {
        double dt = (currTime - this->prevStateTime_).toSec();
        this->currAcc_ = (this->currVel_ - this->prevVel_) / dt;
        this->prevVel_ = this->currVel_;
        this->prevStateTime_ = currTime;
    }
}

void trajectoryBuilder::odomCB(const nav_msgs::Odometry::ConstPtr& odom){
    this->odom_ = *odom;
    this->currPos_(0) = this->odom_.pose.pose.position.x;
    this->currPos_(1) = this->odom_.pose.pose.position.y;
    this->currPos_(2) = this->odom_.pose.pose.position.z;
    if (not this->odomReceived_){
        this->odomReceived_ = true;
    }
}

void trajectoryBuilder::circle()
{
    // circle tracking parameters
    if (not this->nh_.getParam("autonomous_flight/circle_radius", this->radius_))
    {
        this->radius_ = 1.0;
        cout << "[AutoFlight]: No radius param found. Use default: 2.0 m." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Circle Radius: " << this->radius_ << "m." << endl;
    }

    if (not this->nh_.getParam("autonomous_flight/time_to_max_radius", this->timeStep_))
    {
        this->timeStep_ = 30;
        cout << "[AutoFlight]: No time to max radius param found. Use default: 30s." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Time to Maximum Circle Radius: " << this->timeStep_ << "s." << endl;
    }

    if (not this->nh_.getParam("autonomous_flight/yaw_control", this->yawControl_))
    {
        this->yawControl_ = false;
        cout << "[AutoFlight]: No yaw control param found. Use default: false." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Yaw Control: " << this->yawControl_ << endl;
    }

    if (not this->nh_.getParam("autonomous_flight/velocity", this->velocity_))
    {
        this->velocity_ = 0.5;
        cout << "[AutoFlight]: No angular velocity param found. Use default: 0.5 m/s." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Velocity: " << this->velocity_ << "m/s." << endl;
    }

    if (not this->nh_.getParam("autonomous_flight/takeoff_height", this->takeoffHgt_)){
        this->takeoffHgt_ = 1.0;
        cout << "[AutoFlight]: No takeoff height param found. Use default: 1.0 m." << endl;
    }
    else{
        cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
    }

    double x = 0;
    double y = 0;
    double z = 0;
    double vx = 0;
    double vy = 0;
    double vz = 0;
    double ax = 0;
    double ay = 0;
    double az = 0;
    double yaw = 0;
    double theta = 0;
    double radius = 0;
    double velocity = 0;
    int rate = 100;
    ros::Rate r(rate);
    double theta_start;
    double theta_end;
    double step = this->timeStep_ * rate;
    int circle = 1;
    int terminate = 0;
    ros::Time startTime = ros::Time::now();
    ros::Time endTime1, endTime2, endTime3;
    while (ros::ok() && terminate == 0)
    {
        x = radius * cos(theta);
        y = radius * sin(theta);
        vx = -velocity * sin(theta);
        vy = velocity * cos(theta);
        ax = -velocity * velocity / radius * cos(theta);
        ay = -velocity * velocity / radius * sin(theta);
        if (this->yawControl_ == true)
        {
            yaw = theta + PI_const / 2;
        }
        else if (this->yawControl_ == false)
        {
            yaw = rpy_from_quaternion(this->odom_.pose.pose.orientation);
        }
        z = this->takeoffHgt_;
        vz = 0;
        az = 0;

        sunray_msgs::Target target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.velocity.x = vx;
        target.velocity.y = vy;
        target.velocity.z = vz;
        target.acceleration.x = ax;
        target.acceleration.y = ay;
        target.acceleration.z = az;
        target.yaw = yaw;

        this->updateTargetWithState(target);

        if (circle == 1)
        {
            radius += this->radius_ / step;
            velocity += this->velocity_ / step;

            if (std::abs(radius - radius_) <= 0.01)
            {
                theta_start = theta;
                endTime1 = ros::Time::now();
                circle += 1;
            }
        }
        else if (circle == 2)
        {
            radius = this->radius_;
            velocity = this->velocity_;
            theta_end = 3 * 2 * PI_const;
            if (std::abs((theta - theta_start) - theta_end) <= 0.1)
            {
                endTime2 = ros::Time::now();
                circle += 1;
            }
        }
        else if (circle == 3)
        {
            radius -= this->radius_ / step;
            velocity -= this->velocity_ / step;
            if (std::abs(radius - 0.0) <= 0.01)
            {
                endTime3 = ros::Time::now();
                circle += 1;
            }
        }
        else
        {
            terminate = 1;
            break;
        }
        ros::Time currentTime = ros::Time::now();
        double t = (currentTime - startTime).toSec();
        theta = velocity / radius * t;
        cout << "[AutoFlight]: Drawing Circle...Radius: " << radius << "\t\r" << std::flush;
        ;
        ros::spinOnce();
        r.sleep();
    }

    if (this->yawControl_ == true)
    {
        while (ros::ok() and std::abs(this->odom_.pose.pose.orientation.z - 0.0) >= 0.01)
        {
            theta += (PI_const * 2) / 180;
            yaw = theta + PI_const / 2;

            sunray_msgs::Target target;
            target.position.x = x;
            target.position.y = y;
            target.position.z = z;
            target.velocity.x = vx;
            target.velocity.y = vy;
            target.velocity.z = vz;
            target.acceleration.x = ax;
            target.acceleration.y = ay;
            target.acceleration.z = az;
            target.yaw = yaw;
            ROS_INFO("Decreasing Radius...");

            updateTargetWithState(target);
            ros::spinOnce();
            r.sleep();
        }
    }
}

void trajectoryBuilder::run()
{

    // flight test with circle
    double r; // radius
    double v; // circle velocity

    // track circle radius parameters
    if (not this->nh_.getParam("autonomous_flight/radius", r))
    {
        r = 1.0;
        cout << "[AutoFlight]: No circle radius param found. Use default: 1.0 m." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Circle radius: " << r << "m." << endl;
    }

    // track circle velocity parameters
    if (not this->nh_.getParam("autonomous_flight/circle_velocity", v))
    {
        v = 1.0;
        cout << "[AutoFlight]: No circle velocity param found. Use default: 1.0 m/s." << endl;
    }
    else
    {
        cout << "[AutoFlight]: Circle velocity: " << v << "m/s." << endl;
    }

    double z = this->odom_.pose.pose.position.z;
    geometry_msgs::PoseStamped startPs;
    startPs.pose.position.x = r;
    startPs.pose.position.y = 0.0;
    startPs.pose.position.z = z;
    this->updateTarget(startPs);

    cout << "[AutoFlight]: Go to target point..." << endl;
    ros::Rate rate(30);
    while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1)
    {
        ros::spinOnce();
        rate.sleep();
    }
    cout << "[AutoFlight]: Reach target point." << endl;

    ros::Time startTime = ros::Time::now();
    while (ros::ok())
    {
        ros::Time currTime = ros::Time::now();
        double t = (currTime - startTime).toSec();
        double rad = v * t / r;
        double x = r * cos(rad);
        double y = r * sin(rad);
        double vx = -v * sin(rad);
        double vy = v * cos(rad);
        double vz = 0.0;
        double aNorm = v * v / r;
        Eigen::Vector3d accVec(x, y, 0);
        accVec = -aNorm * accVec / accVec.norm();
        double ax = accVec(0);
        double ay = accVec(1);
        double az = 0.0;

        // state target message
        sunray_msgs::Target target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.velocity.x = vx;
        target.velocity.y = vy;
        target.velocity.z = vz;
        target.acceleration.x = ax;
        target.acceleration.y = ay;
        target.acceleration.z = az;
        this->updateTargetWithState(target);
        ros::spinOnce();
        rate.sleep();
    }
}

void trajectoryBuilder::stop()
{
    geometry_msgs::PoseStamped ps;
    ps.pose = this->odom_.pose.pose;
    this->updateTarget(ps);
}


void trajectoryBuilder::updateTarget(const geometry_msgs::PoseStamped &ps)
{
    this->poseTgt_ = ps;
    this->poseTgt_.header.frame_id = "map";
    this->poseControl_ = true;
}

void trajectoryBuilder::updateTargetWithState(const sunray_msgs::Target &target)
{
    this->stateTgt_ = target;
    this->poseControl_ = true;
}