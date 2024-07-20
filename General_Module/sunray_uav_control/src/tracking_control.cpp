/*
	FILE: trackingController.cpp
	-------------------------------
	function implementation of px4 tracking controller
*/

#include "tracking_control.h"

using namespace std;
trackingController::trackingController(const ros::NodeHandle &nh) : nh_(nh)
{
	this->initParam();
	this->registerPub();
	this->registerCallback();
}

void trackingController::initParam()
{
	this->nh_.param<int>("uav_id", uav_id, 1);
    this->nh_.param<string>("uav_name", uav_name, "uav");
	topic_name = "/" + uav_name + std::to_string(uav_id);
	// body rate control
	if (not this->nh_.getParam("controller/body_rate_control", this->bodyRateControl_))
	{
		this->bodyRateControl_ = true;
		cout << "[trackingController]: No body rate control param. Use default: acceleration control." << endl;
	}
	else
	{
		cout << "[trackingController]: Body rate control is set to: " << this->bodyRateControl_ << endl;
	}

	// attitude control
	if (not this->nh_.getParam("controller/attitude_control", this->attitudeControl_))
	{
		this->attitudeControl_ = true;
		cout << "[trackingController]: No attitude control param. Use default: acceleration control." << endl;
	}
	else
	{
		cout << "[trackingController]: Attitude control is set to: " << this->attitudeControl_ << endl;
	}

	// acceleration control
	if (not this->nh_.getParam("controller/acceleration_control", this->accControl_))
	{
		this->accControl_ = true;
		cout << "[trackingController]: No acceleration control param. Use default: acceleration control." << endl;
	}
	else
	{
		cout << "[trackingController]: Acceleration control is set to: " << this->accControl_ << endl;
	}

	// position control
	if (not this->nh_.getParam("controller/position_control", this->posControl_))
	{
		this->posControl_ = true;
		cout << "[trackingController]: No acceleration control param. Use default: position control." << endl;
	}
	else
	{
		cout << "[trackingController]: Position control is set to: " << this->posControl_ << endl;
	}

	// P for Position
	std::vector<double> pPosTemp;
	if (not this->nh_.getParam("controller/position_p", pPosTemp))
	{
		this->pPos_(0) = 1.0;
		this->pPos_(1) = 1.0;
		this->pPos_(2) = 1.0;
		cout << "[trackingController]: No position p param. Use default: [1.0, 1.0, 1.0]." << endl;
	}
	else
	{
		this->pPos_(0) = pPosTemp[0];
		this->pPos_(1) = pPosTemp[1];
		this->pPos_(2) = pPosTemp[2];
		cout << "[trackingController]: Position p is set to: " << "[" << this->pPos_(0) << ", " << this->pPos_(1) << ", " << this->pPos_(2) << "]." << endl;
	}

	// I for Position
	std::vector<double> iPosTemp;
	if (not this->nh_.getParam("controller/position_i", iPosTemp))
	{
		this->iPos_(0) = 0.0;
		this->iPos_(1) = 0.0;
		this->iPos_(2) = 0.0;
		cout << "[trackingController]: No position i param. Use default: [0.0, 0.0, 0.0]." << endl;
	}
	else
	{
		this->iPos_(0) = iPosTemp[0];
		this->iPos_(1) = iPosTemp[1];
		this->iPos_(2) = iPosTemp[2];
		cout << "[trackingController]: Position i is set to: " << "[" << this->iPos_(0) << ", " << this->iPos_(1) << ", " << this->iPos_(2) << "]." << endl;
	}

	// D for Position
	std::vector<double> dPosTemp;
	if (not this->nh_.getParam("controller/position_d", dPosTemp))
	{
		this->dPos_(0) = 0.0;
		this->dPos_(1) = 0.0;
		this->dPos_(2) = 0.0;
		cout << "[trackingController]: No position d param. Use default: [0.0, 0.0, 0.0]." << endl;
	}
	else
	{
		this->dPos_(0) = dPosTemp[0];
		this->dPos_(1) = dPosTemp[1];
		this->dPos_(2) = dPosTemp[2];
		cout << "[trackingController]: Position d is set to: " << "[" << this->dPos_(0) << ", " << this->dPos_(1) << ", " << this->dPos_(2) << "]." << endl;
	}

	// P for Velocity
	std::vector<double> pVelTemp;
	if (not this->nh_.getParam("controller/velocity_p", pVelTemp))
	{
		this->pVel_(0) = 1.0;
		this->pVel_(1) = 1.0;
		this->pVel_(2) = 1.0;
		cout << "[trackingController]: No velocity p param. Use default: [1.0, 1.0, 1.0]." << endl;
	}
	else
	{
		this->pVel_(0) = pVelTemp[0];
		this->pVel_(1) = pVelTemp[1];
		this->pVel_(2) = pVelTemp[2];
		cout << "[trackingController]: Velocity p is set to:" << "[" << this->pVel_(0) << ", " << this->pVel_(1) << ", " << this->pVel_(2) << "]." << endl;
	}

	// I for Velocity
	std::vector<double> iVelTemp;
	if (not this->nh_.getParam("controller/velocity_i", iVelTemp))
	{
		this->iVel_(0) = 0.0;
		this->iVel_(1) = 0.0;
		this->iVel_(2) = 0.0;
		cout << "[trackingController]: No velocity p param. Use default: [0.0, 0.0, 0.0]." << endl;
	}
	else
	{
		this->iVel_(0) = iVelTemp[0];
		this->iVel_(1) = iVelTemp[1];
		this->iVel_(2) = iVelTemp[2];
		cout << "[trackingController]: Velocity i is set to:" << "[" << this->iVel_(0) << ", " << this->iVel_(1) << ", " << this->iVel_(2) << "]." << endl;
	}

	// D for Velocity
	std::vector<double> dVelTemp;
	if (not this->nh_.getParam("controller/velocity_d", dVelTemp))
	{
		this->dVel_(0) = 0.0;
		this->dVel_(1) = 0.0;
		this->dVel_(2) = 0.0;
		cout << "[trackingController]: No velocity p param. Use default: [0.0, 0.0, 0.0]." << endl;
	}
	else
	{
		this->dVel_(0) = dVelTemp[0];
		this->dVel_(1) = dVelTemp[1];
		this->dVel_(2) = dVelTemp[2];
		cout << "[trackingController]: Velocity d is set to:" << "[" << this->dVel_(0) << ", " << this->dVel_(1) << ", " << this->dVel_(2) << "]." << endl;
	}
}

void trackingController::registerPub()
{	
	// command publisher
	this->cmdPub_ = this->nh_.advertise<mavros_msgs::AttitudeTarget>(topic_name + "/mavros/setpoint_raw/attitude", 100);

	// acc comman publisher
	this->accCmdPub_ = this->nh_.advertise<mavros_msgs::PositionTarget>(topic_name + "/mavros/setpoint_raw/local", 100);

	// pos comman publisher
	this->posCmdPub_ = this->nh_.advertise<mavros_msgs::PositionTarget>(topic_name + "/mavros/setpoint_raw/local", 100);

	// vel comman publisher
	this->velCmdPub_ = this->nh_.advertise<mavros_msgs::PositionTarget>(topic_name + "/mavros/setpoint_raw/local", 100);

	// current pose visualization publisher
	this->poseVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/robot_pose", 1);

	// trajectory history visualization publisher
	this->histTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/trajectory_history", 1);

	// target pose visualization publisher
	this->targetVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/target_pose", 1);

	// target trajectory history publisher
	this->targetHistTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/target_trajectory_history", 1);

	// velocity and acceleration visualization publisher
	this->velAndAccVisPub_ = this->nh_.advertise<visualization_msgs::Marker>("/tracking_controller/vel_and_acc_info", 1);
}

void trackingController::registerCallback()
{
	// odom subscriber
	this->odomSub_ = this->nh_.subscribe(topic_name + "/mavros/local_position/odom", 1, &trackingController::odomCB, this);

	// imu subscriber
	this->imuSub_ = this->nh_.subscribe(topic_name + "/mavros/imu/data", 1, &trackingController::imuCB, this);

	// target setpoint subscriber
	this->targetSub_ = this->nh_.subscribe("/sunray/trajectory/target_state", 1, &trackingController::targetCB, this);

	// controller publisher timer
	this->cmdTimer_ = this->nh_.createTimer(ros::Duration(0.01), &trackingController::cmdCB, this);

	if (not this->accControl_)
	{
		// auto thrust esimator timer
		this->thrustEstimatorTimer_ = this->nh_.createTimer(ros::Duration(0.01), &trackingController::thrustEstimateCB, this);
	}
	// visualization timer
	this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &trackingController::visCB, this);
}

void trackingController::odomCB(const nav_msgs::OdometryConstPtr &odom)
{
	this->odom_ = *odom;
	this->odomReceived_ = true;
}

void trackingController::imuCB(const sensor_msgs::ImuConstPtr &imu)
{
	this->imuData_ = *imu;
	this->imuReceived_ = true;
}

void trackingController::targetCB(const sunray_msgs::TargetConstPtr &target)
{
	this->target_ = *target;
	this->firstTargetReceived_ = true;
	this->targetReceived_ = true;
}

void trackingController::cmdCB(const ros::TimerEvent &)
{
	if (not this->odomReceived_ or not this->targetReceived_)
	{
		return;
	}
	Eigen::Vector4d cmd;

	if (this->posControl_)
	{
		// Control position
		Eigen::Vector3d positionCmd;
		positionControl(this->target_.position, positionCmd);

		// Publish commanded acceleration
		publishCommandPos(positionCmd);
	}

	this->targetReceived_ = false;
}

void trackingController::positionControl(const geometry_msgs::Vector3 &targetPosition, Eigen::Vector3d &cmd)
{
	cmd(0) = targetPosition.x;
	cmd(1) = targetPosition.y;
	cmd(2) = targetPosition.z;
}


void trackingController::publishCommandPos(const Eigen::Vector3d &posRef)
{
	mavros_msgs::PositionTarget cmdMsg;
	cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
	cmdMsg.header.stamp = ros::Time::now();
	cmdMsg.header.frame_id = "map";
	cmdMsg.position.x = posRef(0);
	cmdMsg.position.y = posRef(1);
	cmdMsg.position.z = posRef(2);
	cmdMsg.yaw = this->target_.yaw;
	cmdMsg.type_mask = cmdMsg.IGNORE_AFX + cmdMsg.IGNORE_AFY + cmdMsg.IGNORE_AFZ + cmdMsg.IGNORE_VX + cmdMsg.IGNORE_VY + cmdMsg.IGNORE_VZ + cmdMsg.IGNORE_YAW_RATE;
	this->posCmdPub_.publish(cmdMsg);
}

void trackingController::publishCommandVel(const Eigen::Vector3d &velRef)
{
	mavros_msgs::PositionTarget cmdMsg;
	cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
	cmdMsg.header.stamp = ros::Time::now();
	cmdMsg.header.frame_id = "map";
	cmdMsg.velocity.x = velRef(0);
	cmdMsg.velocity.y = velRef(1);
	cmdMsg.velocity.z = velRef(2);
	cmdMsg.yaw = this->target_.yaw;
	cmdMsg.type_mask = cmdMsg.IGNORE_AFX + cmdMsg.IGNORE_AFY + cmdMsg.IGNORE_AFZ + cmdMsg.IGNORE_PX + cmdMsg.IGNORE_PY + cmdMsg.IGNORE_PZ + cmdMsg.IGNORE_YAW_RATE;
	this->velCmdPub_.publish(cmdMsg);
}
