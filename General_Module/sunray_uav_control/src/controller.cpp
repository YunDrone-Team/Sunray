#include "controller.h"

using namespace std;

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{

    resetThrustMapping();
}

quadrotor_msgs::Px4ctrlDebug
LinearControl::update_alg1(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u)
{

    // Check the given velocity is valid.
    if (des.v(2) < -3.0)
        ROS_WARN("[px4ctrl] Desired z-Velocity = %6.3fm/s, < -3.0m/s, which is dangerous since the drone will be unstable!", des.v(2));

    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations = computePIDErrorAcc(odom, des, param_);
    Eigen::Vector3d total_acc = pid_error_accelerations + des.a + Eigen::Vector3d(0, 0, param_.gra);
    Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(total_acc);

    debug_msg_.fb_a_x = pid_error_accelerations(0); // debug
    debug_msg_.fb_a_y = pid_error_accelerations(1);
    debug_msg_.fb_a_z = pid_error_accelerations(2);
    debug_msg_.des_a_x = total_des_acc(0);
    debug_msg_.des_a_y = total_des_acc(1);
    debug_msg_.des_a_z = total_des_acc(2);

    u.thrust = computeDesiredCollectiveThrustSignal(total_des_acc);

    Eigen::Quaterniond desired_attitude;
    computeFlatInput(total_des_acc, des.j, des.yaw, des.yaw_rate, odom.q, desired_attitude, u.bodyrates);
    const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q);

    debug_msg_.des_q_w = desired_attitude.w(); // debug
    debug_msg_.des_q_x = desired_attitude.x();
    debug_msg_.des_q_y = desired_attitude.y();
    debug_msg_.des_q_z = desired_attitude.z();

    u.q = imu.q * odom.q.inverse() * desired_attitude; // Align with FCU frame
    u.bodyrates += feedback_bodyrates;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
    {

        timed_thrust_.pop();
    }
    return debug_msg_;
};

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
                                const Odom_Data_t &odom,
                                const Imu_Data_t &imu,
                                Controller_Output_t &u)
{

    // compute disired acceleration
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    Eigen::Vector3d p_error = Kp.asDiagonal() * (des.p - odom.p);
    des_acc = des.a + Kv.asDiagonal() * ((des.v + p_error) - odom.v);
    des_acc += Eigen::Vector3d(0, 0, param_.gra);

    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
    // yaw = fromQuaternion2yaw(des.q);
    yaw_imu = fromQuaternion2yaw(imu.q);
    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    u.q = imu.q * odom.q.inverse() * q;

    /* WRITE YOUR CODE HERE */
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);

    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrust_.size() > 100)
    {

        timed_thrust_.pop();
    }
    return debug_msg_;
}

Eigen::Vector3d LinearControl::computePIDErrorAcc(
    const Odom_Data_t &odom,
    const Desired_State_t &des,
    const Parameter_t &param)
{
    // Compute the desired accelerations due to control errors in world frame
    // with a PID controller
    Eigen::Vector3d acc_error;
    Eigen::Vector3d Kp, Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    // x acceleration
    double x_pos_error = std::isnan(des.p(0)) ? 0.0 : std::max(std::min(des.p(0) - odom.p(0), 1.0), -1.0);
    double x_vel_error = std::max(std::min((des.v(0) + Kp(0) * x_pos_error) - odom.v(0), 1.0), -1.0);
    acc_error(0) = Kv(0) * x_vel_error;

    // y acceleration
    double y_pos_error = std::isnan(des.p(1)) ? 0.0 : std::max(std::min(des.p(1) - odom.p(1), 1.0), -1.0);
    double y_vel_error = std::max(std::min((des.v(1) + Kp(1) * y_pos_error) - odom.v(1), 1.0), -1.0);
    acc_error(1) = Kv(1) * y_vel_error;

    // z acceleration
    double z_pos_error = std::isnan(des.p(2)) ? 0.0 : std::max(std::min(des.p(2) - odom.p(2), 1.0), -1.0);
    double z_vel_error = std::max(std::min((des.v(2) + Kp(2) * z_pos_error) - odom.v(2), 1.0), -1.0);
    acc_error(2) = Kv(2) * z_vel_error;

    debug_msg_.des_v_x = (des.v(0) + Kp(0) * x_pos_error); // debug
    debug_msg_.des_v_y = (des.v(1) + Kp(1) * y_pos_error);
    debug_msg_.des_v_z = (des.v(2) + Kp(2) * z_pos_error);

    return acc_error;
}
/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{

    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a)
{

    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1)
    {

        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)
        { // 45ms

            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035)
        { // 35ms

            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K = gamma * P_ * thr;
        thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_ = (1 - K * thr) * P_ / rho2_;
        if (param_.thr_map.print_val)
            printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        debug_msg_.hover_percentage = thr2acc_;
        return true;
    }
    return false;
}

Eigen::Vector3d LinearControl::computeLimitedTotalAcc(
    const Eigen::Vector3d &ref_acc) const
{

    Eigen::Vector3d total_acc = ref_acc;

    // Limit angle
    if (param_.max_angle > 0)
    {

        double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
        Eigen::Vector3d z_B = total_acc.normalized();
        if (z_acc < kMinNormalizedCollectiveThrust_)
        {

            z_acc = kMinNormalizedCollectiveThrust_; // Not allow too small z-force when angle limit is enabled.
        }
        Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
        double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
        if (rot_ang > param_.max_angle)
        { // Exceed the angle limit

            Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(param_.max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
            total_acc = z_acc / std::cos(param_.max_angle) * limited_z_B;
        }
    }

    return total_acc;
}

// grav is the gravitional acceleration
// the coordinate should have upward z-axis
void LinearControl::computeFlatInput(const Eigen::Vector3d &thr_acc,
                                     const Eigen::Vector3d &jer,
                                     const double &yaw,
                                     const double &yawd,
                                     const Eigen::Quaterniond &att_est,
                                     Eigen::Quaterniond &att,
                                     Eigen::Vector3d &omg) const
{

    static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);

    if (thr_acc.norm() < kMinNormalizedCollectiveThrust_)
    {

        att = att_est;
        omg.setConstant(0.0);
        ROS_WARN("Conor case, thrust is too small, thr_acc.norm()=%f", thr_acc.norm());
        return;
    }
    else
    {

        Eigen::Vector3d zb, zbd;
        normalizeWithGrad(thr_acc, jer, zb, zbd);
        double syaw = sin(yaw);
        double cyaw = cos(yaw);
        Eigen::Vector3d xc(cyaw, syaw, 0.0);
        Eigen::Vector3d xcd(-syaw * yawd, cyaw * yawd, 0.0);
        Eigen::Vector3d yc = zb.cross(xc);
        if (yc.norm() < kAlmostZeroValueThreshold_)
        {

            ROS_WARN("Conor case, pitch is close to 90 deg");
            att = att_est;
            omg = omg_old;
        }
        else
        {

            Eigen::Vector3d ycd = zbd.cross(xc) + zb.cross(xcd);
            Eigen::Vector3d yb, ybd;
            normalizeWithGrad(yc, ycd, yb, ybd);
            Eigen::Vector3d xb = yb.cross(zb);
            Eigen::Vector3d xbd = ybd.cross(zb) + yb.cross(zbd);
            omg(0) = (zb.dot(ybd) - yb.dot(zbd)) / 2.0;
            omg(1) = (xb.dot(zbd) - zb.dot(xbd)) / 2.0;
            omg(2) = (yb.dot(xbd) - xb.dot(ybd)) / 2.0;
            Eigen::Matrix3d rotM;
            rotM << xb, yb, zb;
            att = Eigen::Quaterniond(rotM);
            omg_old = omg;
        }
    }
    return;
}

Eigen::Vector3d LinearControl::computeFeedBackControlBodyrates(
    const Eigen::Quaterniond &des_q,
    const Eigen::Quaterniond &est_q)
{

    // Compute the error quaternion
    const Eigen::Quaterniond q_e = est_q.inverse() * des_q;

    Eigen::AngleAxisd rotation_vector(q_e); // debug
    Eigen::Vector3d axis = rotation_vector.axis();
    debug_msg_.err_axisang_x = axis(0);
    debug_msg_.err_axisang_y = axis(1);
    debug_msg_.err_axisang_z = axis(2);
    debug_msg_.err_axisang_ang = rotation_vector.angle();

    // Compute desired body rates from control error
    Eigen::Vector3d bodyrates;
    Eigen::Vector3d KAng;
    KAng << param_.gain.KAngR, param_.gain.KAngP, param_.gain.KAngY;

    if (q_e.w() >= 0)
    {
        bodyrates.x() = 2.0 * KAng(0) * q_e.x();
        bodyrates.y() = 2.0 * KAng(1) * q_e.y();
        bodyrates.z() = 2.0 * KAng(2) * q_e.z();
    }
    else
    {
        bodyrates.x() = -2.0 * KAng(0) * q_e.x();
        bodyrates.y() = -2.0 * KAng(1) * q_e.y();
        bodyrates.z() = -2.0 * KAng(2) * q_e.z();
    }

    // debug
    debug_msg_.fb_rate_x = bodyrates.x();
    debug_msg_.fb_rate_y = bodyrates.y();
    debug_msg_.fb_rate_z = bodyrates.z();

    return bodyrates;
}

void LinearControl::resetThrustMapping(void)
{

    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_ = 1e6;
}

void LinearControl::normalizeWithGrad(const Eigen::Vector3d &x,
                                      const Eigen::Vector3d &xd,
                                      Eigen::Vector3d &xNor,
                                      Eigen::Vector3d &xNord) const
{

    const double xSqrNorm = x.squaredNorm();
    const double xNorm = sqrt(xSqrNorm);
    xNor = x / xNorm;
    xNord = (xd - x * (x.dot(xd) / xSqrNorm)) / xNorm;
    return;
}

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{

    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}