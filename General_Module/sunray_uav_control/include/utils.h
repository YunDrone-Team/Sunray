#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Eigen>

const double PI_const = 3.1415926;
inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
{
    if (yaw > PI_const)
    {
        yaw = yaw - 2 * PI_const;
    }
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
    return quaternion;
}

inline double rpy_from_quaternion(const geometry_msgs::Quaternion &quat)
{
    // return is [0, 2pi]
    tf2::Quaternion tf_quat;
    tf2::convert(quat, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

inline void rpy_from_quaternion(const geometry_msgs::Quaternion &quat, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion tf_quat;
    tf2::convert(quat, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
}

inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q)
{
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R)
{
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0)
    {
        double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    }
    else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
    {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    }
    else if (R(1, 1) > R(2, 2))
    {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    }
    else
    {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}

inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p)
{
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}