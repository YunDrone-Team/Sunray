#include "ros_msg_utils.h"
#include "printf_format.h"

using namespace sunray_logger;

// 无人机状态集合
struct PX4State
{
    Eigen::Vector3d pos{0.0, 0.0, 0.0};        // 无人机当前位置
    Eigen::Vector3d vel{0.0, 0.0, 0.0};        // 无人机当前速度
    Eigen::Vector3d att{0.0, 0.0, 0.0};        // 无人机当前姿态 角度
    Eigen::Vector4d att_q{0.0, 0.0, 0.0, 0.0}; // 无人机当前姿态 四元数xyzw
    Eigen::Vector3d target_pos{0.0, 0.0, 0.0}; // 无人机目标位置
    Eigen::Vector3d target_vel{0.0, 0.0, 0.0}; // 无人机目标速度
    Eigen::Vector3d target_att{0.0, 0.0, 0.0}; // 无人机目标姿态 角度
    float target_yaw = 0;                      // 无人机目标姿态 四元数
};

struct TimeValue
{
    ros::Time time;
    PX4State px4_state;
};

// error estimation
class ErrorEstimation
{
private:
    int uav_id;
    int type_mask;
    int timeout = 10; // 设置超时时间为10秒
    std::string uav_name;
    std::string uav_prefix;
    std::string topic_prefix;
    PX4State px4_state;
    PX4State vision_state;
    double pos_err_mean[3];
    double vel_err_mean[3];
    double yaw_err_mean;
    double pos_err_max[3];
    double vel_err_max[3];
    double yaw_err_max;

    ros::Subscriber px4_pos_sub;
    ros::Subscriber px4_vel_sub;
    ros::Subscriber px4_att_sub;
    ros::Subscriber vision_pose_sub;
    ros::Subscriber px4_pos_target_sub;

    ros::Timer print_timer;
    ros::Timer update_timer;

    std::vector<TimeValue> timeValues;

public:
    ErrorEstimation(ros::NodeHandle &nh)
    {
        nh.param<int>("uav_id", uav_id, 1);                 // 【参数】无人机编号
        nh.param<std::string>("uav_name", uav_name, "uav"); // 【参数】无人机名称

        uav_prefix = uav_name + std::to_string(uav_id);
        topic_prefix = "/" + uav_prefix;

        // px4_state_sub = nh.subscribe<mavros_msgs::State>(topic_prefix + "/mavros/state",
        //                                                  10, px4_state_callback, this);
        px4_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/local_position/pose",
                                                               10, &ErrorEstimation::px4_pos_callback, this);
        px4_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_prefix + "/mavros/local_position/velocity_local",
                                                                10, &ErrorEstimation::px4_vel_callback, this);
        px4_att_sub = nh.subscribe<sensor_msgs::Imu>(topic_prefix + "/mavros/imu/data", 1,
                                                     &ErrorEstimation::px4_att_callback, this);
        vision_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_prefix + "/mavros/vision_pose/pose", 10, &ErrorEstimation::vision_pose_callback, this);
        px4_pos_target_sub =
            nh.subscribe<mavros_msgs::PositionTarget>(topic_prefix + "/mavros/setpoint_raw/target_local",
                                                      1,
                                                      &ErrorEstimation::px4_pos_target_callback, this);
        print_timer = nh.createTimer(ros::Duration(0.1), &ErrorEstimation::print_err, this);
        update_timer = nh.createTimer(ros::Duration(0.1), &ErrorEstimation::update_data, this);
    }

    void update_data(const ros::TimerEvent &event)
    {
        timeValues.push_back(TimeValue{ros::Time::now(), px4_state});
        cleanVector();
        // 计算历史误差 均值 最大值
        if (timeValues.size() > 1)
        {
            int pos_cont = 0;
            int vel_cont = 0;
            double pos_mean[3] = {0.0, 0.0, 0.0};
            double vel_mean[3] = {0.0, 0.0, 0.0};
            double pos_yaw_mean = 0.0;
            double pos_yaw_max = 0.0;
            double pos_max[3] = {0.0, 0.0, 0.0};
            double vel_max[3] = {0.0, 0.0, 0.0};

            for (int i = 0; i < timeValues.size() - 1; i++)
            {
                if (isnan(timeValues[i].px4_state.pos[0]))
                {
                    pos_cont = 0;
                    pos_mean[0] = 0;
                    pos_mean[1] = 0;
                    pos_mean[2] = 0;
                }
                if (isnan(timeValues[i].px4_state.vel[0]))
                {
                    vel_cont = 0;
                    vel_mean[0] = 0;
                    vel_mean[1] = 0;
                    vel_mean[2] = 0;
                }
                pos_mean[0] += timeValues[i].px4_state.pos[0] - timeValues[i].px4_state.target_pos[0];
                pos_mean[1] += timeValues[i].px4_state.pos[1] - timeValues[i].px4_state.target_pos[1];
                pos_mean[2] += timeValues[i].px4_state.pos[2] - timeValues[i].px4_state.target_pos[2];
                pos_yaw_mean += timeValues[i].px4_state.att[2] - timeValues[i].px4_state.target_yaw;
                vel_mean[0] += timeValues[i].px4_state.vel[0] - timeValues[i].px4_state.target_vel[0];
                vel_mean[1] += timeValues[i].px4_state.vel[1] - timeValues[i].px4_state.target_vel[1];
                vel_mean[2] += timeValues[i].px4_state.vel[2] - timeValues[i].px4_state.target_vel[2];
                pos_cont++;
                vel_cont++;
                pos_max[0] = std::max(pos_max[0], fabs(timeValues[i].px4_state.pos[0] - timeValues[i].px4_state.target_pos[0]));
                pos_max[1] = std::max(pos_max[1], fabs(timeValues[i].px4_state.pos[1] - timeValues[i].px4_state.target_pos[1]));
                pos_max[2] = std::max(pos_max[2], fabs(timeValues[i].px4_state.pos[2] - timeValues[i].px4_state.target_pos[2]));
                pos_yaw_max = std::max(pos_yaw_max, fabs(timeValues[i].px4_state.att[2] - timeValues[i].px4_state.target_yaw));
                vel_max[0] = std::max(vel_max[0], fabs(timeValues[i].px4_state.vel[0] - timeValues[i].px4_state.target_vel[0]));
                vel_max[1] = std::max(vel_max[1], fabs(timeValues[i].px4_state.vel[1] - timeValues[i].px4_state.target_vel[1]));
                vel_max[2] = std::max(vel_max[2], fabs(timeValues[i].px4_state.vel[2] - timeValues[i].px4_state.target_vel[2]));
            }
            pos_err_mean[0] = pos_mean[0] / pos_cont;
            pos_err_mean[1] = pos_mean[1] / pos_cont;
            pos_err_mean[2] = pos_mean[2] / pos_cont;
            yaw_err_mean = pos_yaw_mean / timeValues.size();
            vel_err_mean[0] = vel_mean[0] / vel_cont;
            vel_err_mean[1] = vel_mean[1] / vel_cont;
            vel_err_mean[2] = vel_mean[2] / vel_cont;
            pos_err_max[0] = pos_max[0];
            pos_err_max[1] = pos_max[1];
            pos_err_max[2] = pos_max[2];
            yaw_err_max = pos_yaw_max;
            vel_err_max[0] = vel_max[0];
            vel_err_max[1] = vel_max[1];
            vel_err_max[2] = vel_max[2];
        }
    }

    // 检查并删除超过特定时间的元素
    void cleanVector()
    {
        auto currentTime = ros::Time::now();
        timeValues.erase(std::remove_if(timeValues.begin(), timeValues.end(), [currentTime](const TimeValue &value)
                                        { return (currentTime - value.time).toSec() > 1.0; }),
                         timeValues.end());
    }

    // 无人机pose回调
    void px4_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        px4_state.pos[0] = msg->pose.position.x;
        px4_state.pos[1] = msg->pose.position.y;
        px4_state.pos[2] = msg->pose.position.z;
    }

    // 无人机vel回调
    void px4_vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        px4_state.vel[0] = msg->twist.linear.x;
        px4_state.vel[1] = msg->twist.linear.y;
        px4_state.vel[2] = msg->twist.linear.z;
    }

    // 无人机姿态回调
    void px4_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        px4_state.att_q[0] = msg->orientation.x;
        px4_state.att_q[1] = msg->orientation.y;
        px4_state.att_q[2] = msg->orientation.z;
        px4_state.att_q[3] = msg->orientation.w;

        // 转为rpy
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        px4_state.att[0] = roll;
        px4_state.att[1] = pitch;
        px4_state.att[2] = yaw;
    }

    // vision_pose回调
    void vision_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        vision_state.pos[0] = msg->pose.position.x;
        vision_state.pos[1] = msg->pose.position.y;
        vision_state.pos[2] = msg->pose.position.z;
        vision_state.att_q[0] = msg->pose.orientation.x;
        vision_state.att_q[1] = msg->pose.orientation.y;
        vision_state.att_q[2] = msg->pose.orientation.z;
        vision_state.att_q[3] = msg->pose.orientation.w;

        // 转为rpy
        tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        vision_state.att[0] = roll;
        vision_state.att[1] = pitch;
        vision_state.att[2] = yaw;
    }

    // 无人机目标位置回调
    void px4_pos_target_callback(const mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        type_mask = msg->type_mask;
        px4_state.target_pos[0] = msg->position.x;
        px4_state.target_pos[1] = msg->position.y;
        px4_state.target_pos[2] = msg->position.z;
        px4_state.target_vel[0] = msg->velocity.x;
        px4_state.target_vel[1] = msg->velocity.y;
        px4_state.target_vel[2] = msg->velocity.z;
        px4_state.target_yaw = msg->yaw;
    }

    void print_err(const ros::TimerEvent &event)
    {
        // 打印
        // 清空终端
        system("clear");
        Logger::print_color(int(LogColor::blue), LOG_BOLD, ">>>>>>>>>>>>>>", uav_prefix, "<<<<<<<<<<<<<<<");
        // vision
        Logger::print_color(int(LogColor::blue), "CUR POS(vision_pose)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z YAW]:",
                            vision_state.pos[0],
                            vision_state.pos[1],
                            vision_state.pos[2],
                            vision_state.att[2] / M_PI * 180);
        // px4
        Logger::print_color(int(LogColor::blue), "PX4 POS(ekf)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z YAW]:",
                            px4_state.pos[0],
                            px4_state.pos[1],
                            px4_state.pos[2],
                            px4_state.att[2] / M_PI * 180);
        // target
        Logger::print_color(int(LogColor::blue), "TAR POS(target)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z YAW]:",
                            px4_state.target_pos[0],
                            px4_state.target_pos[1],
                            px4_state.target_pos[2],
                            px4_state.target_yaw / M_PI * 180);
        // px4 error
        Logger::print_color(int(LogColor::yellow), "PX4 POS ERR(vision_pose - ekf)");
        Logger::print_color(int(LogColor::green), "POS[X Y Z YAW]:",
                            vision_state.pos[0] - px4_state.pos[0],
                            vision_state.pos[1] - px4_state.pos[1],
                            vision_state.pos[2] - px4_state.pos[2],
                            (vision_state.att[2] - px4_state.att[2]) / M_PI * 180);
        // target error
        if (!isnan(px4_state.target_pos[0]))
        {
            Logger::print_color(int(LogColor::yellow), "TAR POS ERR(ekf - target)");
            Logger::print_color(int(LogColor::green), "POS[X Y Z YAW]:",
                                px4_state.pos[0] - px4_state.target_pos[0],
                                px4_state.pos[1] - px4_state.target_pos[1],
                                px4_state.pos[2] - px4_state.target_pos[2],
                                (px4_state.att[2] - px4_state.target_yaw) / M_PI * 180);
        }

        if (!isnan(px4_state.target_vel[0]))
        {
            Logger::print_color(int(LogColor::yellow), "TAR VEl ERR(ekf - target)");
            Logger::print_color(int(LogColor::green), "VEL[X Y Z YAW]:",
                                px4_state.vel[0] - px4_state.target_vel[0],
                                px4_state.vel[1] - px4_state.target_vel[1],
                                px4_state.vel[2] - px4_state.target_vel[2],
                                (px4_state.att[2] - px4_state.target_yaw) / M_PI * 180);
        }

        // history pos err
        if (!isnan(px4_state.target_pos[0]))
        {
            Logger::print_color(int(LogColor::yellow), "TAR POS ERR(history ekf - target)");
            Logger::print_color(int(LogColor::green), "MEAN POS[X Y Z YAW]:",
                                pos_err_mean[0],
                                pos_err_mean[1],
                                pos_err_mean[2],
                                yaw_err_mean / M_PI * 180);
            Logger::print_color(int(LogColor::green), "MAX  POS[X Y Z YAW]:",
                                pos_err_max[0],
                                pos_err_max[1],
                                pos_err_max[2],
                                yaw_err_max / M_PI * 180);
        }

        if (!isnan(px4_state.target_vel[0]))
        {
            Logger::print_color(int(LogColor::yellow), "TAR VEl ERR(history ekf - target)");
            Logger::print_color(int(LogColor::green), "MEAN VEL[X Y Z YAW]:",
                                vel_err_mean[0],
                                vel_err_mean[1],
                                vel_err_mean[2],
                                yaw_err_mean / M_PI * 180);
            Logger::print_color(int(LogColor::green), "MAX  VEL[X Y Z YAW]:",
                                vel_err_max[0],
                                vel_err_max[1],
                                vel_err_max[2],
                                yaw_err_max / M_PI * 180);
        }
    }

    ~ErrorEstimation()
    {
    }
};

int main(int argc, char **argv)
{
    // 设置日志
    Logger::init_default();
    Logger::setPrintLevel(false);
    Logger::setPrintTime(false);
    Logger::setPrintToFile(false);
    Logger::setFilename("~/Documents/Sunray_log.txt");

    ros::init(argc, argv, "ErrorEstimation");
    ros::NodeHandle nh("~");
    ros::Rate rate(50);
    ErrorEstimation err(nh);
    std::cout<<std::abs(-0.011)<<std::fabs(-0.011)<<std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}