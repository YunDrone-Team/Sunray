#ifndef SUNRAY_VIOBOT_H
#define SUNRAY_VIOBOT_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Temperature.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sunray_msgs/ViobotState.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <visualization_msgs/MarkerArray.h>

#include "system_ctrl/algo_ctrl.h"
#include "system_ctrl/algo_status.h"
#include "system_ctrl/viobot_ctrl.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include "loop_action/KeyFrameHandleActionGoal.h"
#include "loop_action/KeyFrameHandleActionFeedback.h"
#include "loop_action/KeyFrameHandleActionResult.h"

#include "printf_utils.h"
#include "math_utils.h"

using namespace std;

class VIOBOT
{
    public:
        // 构造函数
        VIOBOT(){};
        // 析构函数
        ~VIOBOT(){
            // this->shutdown_stereo2();
        };

        // 初始化函数
        void init(ros::NodeHandle& nh, bool if_pritf);
        // 启动stereo2算法
        bool start_stereo2();
        // 关闭stereo2算法
        void shutdown_stereo2();

        system_ctrl::algo_status algo_status;    // viobot算法状态
        nav_msgs::Odometry odom_rect;            // 里程计数据
        sunray_msgs::ViobotState viobot_state;   // VIOBOT状态集合
        Eigen::Quaterniond q_viobot;             // 四元数
        Eigen::Vector3d euler_viobot;            // 欧拉角
        Eigen::Vector3d viobot_pos;              // 位置
        Eigen::Vector3d viobot_vel;              // 速度
        geometry_msgs::Quaternion q_viobot_msg;  // 四元数
        double viobot_yaw;                       // 偏航角  

        sensor_msgs::PointCloud2 rdf_point;
        sensor_msgs::PointCloud2ConstPtr rdf_point_ptr;

    private:
        string node_name;
        bool flag_printf = false;

        system_ctrl::algo_ctrl stereo2_ctrl;     // 启动stereo2算法      
        bool stereo2_flag = false;

        // 【发布】vio state
        ros::Publisher viobot_state_pub;
        // 【发布】stereo2算法启动、关闭、重置操作 to /viobot节点
        ros::Publisher pub_stereo2_ctrl;
        // 【订阅】算法状态 from /viobot节点
        ros::Subscriber viobot_algo_status_sub;
        // 【订阅】imu信息 from /viobot节点
        ros::Subscriber viobot_imu_sub;
        // 【订阅】左目图像信息 from /viobot节点
        ros::Subscriber viobot_image_left_sub;
        // 【订阅】右目图像信息 from /viobot节点
        ros::Subscriber viobot_image_right_sub;
        // 【订阅】左目相机参数 from /viobot节点
        ros::Subscriber viobot_camera_left_info_sub;
        // 【订阅】右目相机参数 from /viobot节点
        ros::Subscriber viobot_camera_right_info_sub;
        // 【订阅】里程计数据 from /stereo2节点
        ros::Subscriber stereo2_odometry_rect_sub;
        // 【订阅】原始点云数据 from /stereo2节点
        ros::Subscriber stereo2_points_sub;
        // 【订阅】RDF点云数据 from /stereo2节点
        ros::Subscriber stereo2_points_rdf_sub;
        // 【订阅】带特征点的图片 from /stereo2节点
        ros::Subscriber stereo2_warped_img_sub;
        // 【定时器】定时打印
        ros::Timer debug_timer;

        void viobot_algo_status_cb(const system_ctrl::algo_status::ConstPtr &msg);
        void viobot_imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
        void viobot_image_right_cb(const sensor_msgs::Image::ConstPtr &msg);
        void viobot_image_left_cb(const sensor_msgs::Image::ConstPtr &msg);
        void viobot_camera_left_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void viobot_camera_right_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void stereo2_odometry_rect_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void stereo2_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void stereo2_points_rdf_cb(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void stereo2_warped_img_cb(const sensor_msgs::Image::ConstPtr &msg);
        void debug_timer_cb(const ros::TimerEvent &e);
        Eigen::Vector3d quaternion_to_euler_viobot(const Eigen::Quaterniond &q);

};

#endif