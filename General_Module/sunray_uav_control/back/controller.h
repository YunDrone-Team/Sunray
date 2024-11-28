#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <queue>

#include "ROScallback.h"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t() {};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0) {};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	// Eigen::Vector3d des_v_real;
};

class LinearControl
{
public:
	LinearControl(Parameter_t &);

	quadrotor_msgs::Px4ctrlDebug
	calculateControl(const Desired_State_t &des,
					 const Odom_Data_t &odom,
					 const Imu_Data_t &imu,
					 Controller_Output_t &u);

	quadrotor_msgs::Px4ctrlDebug
	update_alg1(const Desired_State_t &des,
				const Odom_Data_t &odom,
				const Imu_Data_t &imu,
				Controller_Output_t &u);

	bool estimateThrustModel(const Eigen::Vector3d &est_v);
	void resetThrustMapping(void);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	static constexpr double kAlmostZeroThrustThreshold_ = 0.01;

	// Thrust-accel mapping params
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;

	void normalizeWithGrad(const Eigen::Vector3d &x,
						   const Eigen::Vector3d &xd,
						   Eigen::Vector3d &xNor,
						   Eigen::Vector3d &xNord) const;
	Eigen::Vector3d computePIDErrorAcc(
		const Odom_Data_t &odom,
		const Desired_State_t &des,
		const Parameter_t &param);

	Eigen::Vector3d computeLimitedTotalAcc(
		const Eigen::Vector3d &ref_acc) const;

	void computeFlatInput(const Eigen::Vector3d &thr_acc,
						  const Eigen::Vector3d &jer,
						  const double &yaw,
						  const double &yawd,
						  const Eigen::Quaterniond &att_est,
						  Eigen::Quaterniond &att,
						  Eigen::Vector3d &omg) const;

	Eigen::Vector3d computeFeedBackControlBodyrates(
		const Eigen::Quaterniond &des_q,
		const Eigen::Quaterniond &est_q);

	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
	double fromQuaternion2yaw(Eigen::Quaterniond q);
};

#endif