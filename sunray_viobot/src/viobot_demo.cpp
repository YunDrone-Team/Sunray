#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Temperature.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


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


void viobot_algo_status_cb(const system_ctrl::algo_status::ConstPtr &msg)
{
    system_ctrl::algo_status algo_status;
    cout << GREEN << "algo_status: " << msg->algo_status  << TAIL << endl;
}

void viobot_sys_status_cb(const system_ctrl::viobot_ctrl::ConstPtr &msg)
{
    ROS_INFO("sys_status:");
    if(msg->image_select == 1) std::cout << "image_select: " << "left" << std::endl;
    else if(msg->image_select == 2) std::cout << "image_select: " << "right" << std::endl;
    else if(msg->image_select == 3) std::cout << "image_select: " << "left and right" << std::endl;
    else std::cout << "image_select: " << "OFF" << std::endl;

    if(msg->imu_raw == false) std::cout << "imu_raw: " << "OFF" << std::endl;
    else std::cout << "imu_raw: " << "ON" << std::endl;

    if(msg->tof_depth == false) std::cout << "tof_depth: " << "OFF" << std::endl;
    else std::cout << "tof_depth: " << "ON" << std::endl;

    if(msg->tof_enable == false) std::cout << "tof_enable: " << "OFF" << std::endl;
    else std::cout << "tof_enable: " << "ON" << std::endl;

    if(msg->tof_amp == false) std::cout << "tof_amp: " << "OFF" << std::endl;
    else std::cout << "tof_amp: " << "ON" << std::endl;

    if(msg->light == false) std::cout << "light: " << "OFF" << std::endl;
    else std::cout << "light: " << "ON" << std::endl;
}


void viobot_vio_restart_cb(const std_msgs::Bool::ConstPtr &msg)
{
    cout << GREEN << "VIO_restart: " << msg->data  << TAIL << endl;
}

void viobot_camera_left_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    cout << GREEN << "camera_left_info: " << msg  << TAIL << endl;
}
void viobot_camera_right_info_cb(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    cout << GREEN << "camera_right_info: " << msg  << TAIL << endl;
}
void viobot_image_right_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    // cout << GREEN << "image_right: " << msg  << TAIL << endl;
}
void viobot_image_left_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    // cout << GREEN << "image_left: " << msg  << TAIL << endl;
}
void viobot_imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    // cout << GREEN << "imu: " << msg  << TAIL << endl;
}
void viobot_imu_temp_cb(const sensor_msgs::Temperature::ConstPtr &msg)
{
    cout << GREEN << "imu_temp: " << msg  << TAIL << endl;
}
void viobot_pcd_save_cb(const std_msgs::Char::ConstPtr &msg)
{
    // cout << GREEN << "PCD_Save: " << msg  << TAIL << endl;
}
void viobot_global_consistency_cb(const std_msgs::Char::ConstPtr &msg)
{
    // cout << GREEN << "global_consistency: " << msg  << TAIL << endl;
}
void viobot_keyframe_action_cancel_cb(const actionlib_msgs::GoalID::ConstPtr &msg)
{
    // cout << GREEN << "global_consistency cancel: " << msg  << TAIL << endl;
}
void viobot_keyframe_action_goal_cb(const loop_action::KeyFrameHandleActionGoal::ConstPtr &msg)
{
    // cout << GREEN << "keyframe_action goal: " << msg  << TAIL << endl;
}

void viobot_reset_vocabulary_cb(const std_msgs::Bool::ConstPtr &msg)
{
    cout << GREEN << "reset_vocabulary: " << msg->data  << TAIL << endl;
}
void viobot_stereo2_restart_cb(const std_msgs::Bool::ConstPtr &msg)
{
    cout << GREEN << "stereo2_restart: " << msg->data  << TAIL << endl;
}
void stereo2_base_path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    cout << GREEN << "base_path: " << msg  << TAIL << endl;
}
void stereo2_imu_propagate_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry viobot_raw = *msg;
    cout << GREEN << "stereo2_imu_propagate_cb: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << viobot_raw.pose.pose.position.x << " [ m ] " << viobot_raw.pose.pose.position.y << " [ m ] " << viobot_raw.pose.pose.position.z << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << viobot_raw.twist.twist.linear.x << " [m/s] " << viobot_raw.twist.twist.linear.y << " [m/s] " << viobot_raw.twist.twist.linear.z << " [m/s] " << TAIL << endl;
}
void stereo2_odometry_adjusted_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry viobot_raw = *msg;
    cout << GREEN << "stereo2_odometry_adjusted_cb: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << viobot_raw.pose.pose.position.x << " [ m ] " << viobot_raw.pose.pose.position.y << " [ m ] " << viobot_raw.pose.pose.position.z << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << viobot_raw.twist.twist.linear.x << " [m/s] " << viobot_raw.twist.twist.linear.y << " [m/s] " << viobot_raw.twist.twist.linear.z << " [m/s] " << TAIL << endl;
}
void stereo2_odometry_rect_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry viobot_raw = *msg;
    cout << GREEN << "stereo2_odometry_rect_cb: " << TAIL << endl;
    cout << GREEN << "Pos [X Y Z] : " << viobot_raw.pose.pose.position.x << " [ m ] " << viobot_raw.pose.pose.position.y << " [ m ] " << viobot_raw.pose.pose.position.z << " [ m ] " << TAIL << endl;
    cout << GREEN << "Vel [X Y Z] : " << viobot_raw.twist.twist.linear.x << " [m/s] " << viobot_raw.twist.twist.linear.y << " [m/s] " << viobot_raw.twist.twist.linear.z << " [m/s] " << TAIL << endl;
}
void stereo2_camera_pose_visual_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    cout << GREEN << "stereo2_camera_pose_visual_cb: " << TAIL << endl;
}

void stereo2_keyframe_action_feedback_cb(const loop_action::KeyFrameHandleActionFeedback::ConstPtr &msg)
{
    cout << GREEN << "stereo2_keyframe_action_feedback_cb: " << TAIL << endl;
}

void stereo2_keyframe_action_result_cb(const loop_action::KeyFrameHandleActionResult::ConstPtr &msg)
{
    cout << GREEN << "stereo2_keyframe_action_result_cb: " << TAIL << endl;
}

void stereo2_keyframe_action_status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    cout << GREEN << "stereo2_keyframe_action_status_cb: " << TAIL << endl;
}

void stereo2_match_image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << GREEN << "stereo2_match_image_cb: " << TAIL << endl;
}
void stereo2_path_1_cb(const nav_msgs::Path::ConstPtr &msg)
{
    cout << GREEN << "stereo2_path_1_cb: " << TAIL << endl;
}
void stereo2_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << GREEN << "stereo2_points_cb: " << TAIL << endl;
}
void stereo2_points_rdf_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << GREEN << "stereo2_points_rdf_cb: " << TAIL << endl;
}
void stereo2_points_adjust_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << GREEN << "stereo2_points_adjust_cb: " << TAIL << endl;
}


void stereo2_pose_graph_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    cout << GREEN << "stereo2_pose_graph_cb: " << TAIL << endl;
}
void stereo2_pose_graph_path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    cout << GREEN << "stereo2_pose_graph_path_cb: " << TAIL << endl;
}
void stereo2_camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    cout << GREEN << "stereo2_camera_pose_cb: " << TAIL << endl;
}
void stereo2_image_left_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << GREEN << "stereo2_image_left_cb: " << TAIL << endl;
}
void stereo2_keyframe_points_cb(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    cout << GREEN << "stereo2_keyframe_points_cb: " << TAIL << endl;
}
void stereo2_keyframe_pose_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    cout << GREEN << "stereo2_keyframe_pose_cb: " << TAIL << endl;
}

void stereo2_odometry_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    cout << GREEN << "stereo2_odometry_cb: " << TAIL << endl;
}

void stereo2_odometry_path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    cout << GREEN << "stereo2_odometry_path_cb: " << TAIL << endl;
}
void stereo2_parray_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    cout << GREEN << "stereo2_parray_cb: " << TAIL << endl;
}
void stereo2_zc_stereo2_points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cout << GREEN << "stereo2_zc_stereo2_points_cb: " << TAIL << endl;
}
void stereo2_warped_img_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    cout << GREEN << "stereo2_warped_img_cb: " << TAIL << endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "viobot_demo");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 订阅VIOBOT /viobot节点的数据
    // 
    ros::Subscriber viobot_vio_restart_sub = nh.subscribe("/viobot/VIO_restart", 1, viobot_vio_restart_cb);
    // 话题含义：系统状态，发布频率：xxHz
    ros::Subscriber viobot_sys_status_sub = nh.subscribe("/viobot/sys_status", 1, viobot_sys_status_cb);
    ros::Subscriber viobot_algo_status_sub = nh.subscribe("//viobotalgo_status", 1, viobot_algo_status_cb);

    ros::Subscriber viobot_imu_sub = nh.subscribe("/viobot/imu", 1, viobot_imu_cb);
    ros::Subscriber viobot_image_left_sub = nh.subscribe("/viobot/image_left", 1, viobot_image_left_cb);
    ros::Subscriber viobot_image_right_sub = nh.subscribe("/viobot/image_right", 1, viobot_image_right_cb);
    ros::Subscriber viobot_camera_left_info_sub = nh.subscribe("/viobot/camera_left_info", 1, viobot_camera_left_info_cb);
    ros::Subscriber viobot_camera_right_info_sub = nh.subscribe("v/camera_right_info", 1, viobot_camera_right_info_cb);
    ros::Subscriber viobot_imu_temp_sub = nh.subscribe("/viobot/imu_temp", 1, viobot_imu_temp_cb);
    ros::Subscriber viobot_pcd_save_sub = nh.subscribe("/viobot/pr_loop/PCD_Save", 1, viobot_pcd_save_cb);
    ros::Subscriber viobot_global_consistency_sub = nh.subscribe("/viobot/pr_loop/global_consistency", 1, viobot_global_consistency_cb);
    ros::Subscriber viobot_keyframe_action_cancel_sub = nh.subscribe("/viobot/pr_loop/keyframe_action/cancel", 1, viobot_keyframe_action_cancel_cb);
    ros::Subscriber viobot_keyframe_action_goal_sub = nh.subscribe("/viobot/pr_loop/keyframe_action/goal", 1, viobot_keyframe_action_goal_cb);
    ros::Subscriber viobot_reset_vocabulary_sub = nh.subscribe("/viobot/reset_vocabulary", 1, viobot_reset_vocabulary_cb);
    ros::Subscriber viobot_stereo2_restart_sub = nh.subscribe("/viobot/zc_stereo2/stereo2_restart", 1, viobot_stereo2_restart_cb);

    // 话题 -> /viobot节点
    ros::Publisher pub_mono_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/viobot/mono_ctrl", 2);
    ros::Publisher pub_stereo1_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/viobot/stereo1_ctrl", 2);
    ros::Publisher pub_stereo2_ctrl = nh.advertise<system_ctrl::algo_ctrl>("/viobot/stereo2_ctrl", 2);

    // 订阅VIOBOT /stereo2节点的数据
    ros::Subscriber stereo2_base_path_sub = nh.subscribe("/viobot/pr_loop/base_path", 1, stereo2_base_path_cb);
    ros::Subscriber stereo2_camera_pose_visual_sub = nh.subscribe("/viobot/pr_loop/camera_pose_visual", 1, stereo2_camera_pose_visual_cb);
    ros::Subscriber stereo2_imu_propagate_sub = nh.subscribe("/viobot/pr_loop/imu_propagate", 1, stereo2_imu_propagate_cb);
    ros::Subscriber stereo2_keyframe_action_feedback_sub = nh.subscribe("/viobot/pr_loop/keyframe_action/feedback", 1, stereo2_keyframe_action_feedback_cb);
    ros::Subscriber stereo2_keyframe_action_result_sub = nh.subscribe("/viobot/pr_loop/keyframe_action/result", 1, stereo2_keyframe_action_result_cb);
    ros::Subscriber stereo2_keyframe_action_status_sub = nh.subscribe("/viobot/pr_loop/keyframe_action/status", 1, stereo2_keyframe_action_status_cb);
    ros::Subscriber stereo2_match_image_sub = nh.subscribe("/viobot/pr_loop/match_image", 1, stereo2_match_image_cb);
    ros::Subscriber stereo2_odometry_adjusted_sub = nh.subscribe("/viobot/pr_loop/odometry_adjusted", 1, stereo2_odometry_adjusted_cb);
    ros::Subscriber stereo2_odometry_rect_sub = nh.subscribe("/viobot/pr_loop/odometry_rect", 1, stereo2_odometry_rect_cb);
    ros::Subscriber stereo2_path_1_sub = nh.subscribe("/viobot/pr_loop/path_1", 1, stereo2_path_1_cb);

    ros::Subscriber stereo2_points_sub = nh.subscribe("/viobot/pr_loop/points", 1, stereo2_points_cb);
    ros::Subscriber stereo2_points_rdf_sub = nh.subscribe("/viobot/pr_loop/points_rdf", 1, stereo2_points_rdf_cb);
    ros::Subscriber stereo2_points_adjust_sub = nh.subscribe("/viobot/pr_loop/points_adjust", 1, stereo2_points_adjust_cb);

    ros::Subscriber stereo2_pose_graph_sub = nh.subscribe("/viobot/pr_loop/pose_graph", 1, stereo2_pose_graph_cb);
    ros::Subscriber stereo2_pose_graph_path_sub = nh.subscribe("/viobot/pr_loop/pose_graph_path", 1, stereo2_pose_graph_path_cb);
    ros::Subscriber stereo2_camera_pose_sub = nh.subscribe("/viobot/zc_stereo2/camera_pose", 1, stereo2_camera_pose_cb);
    // 为什么没有右目
    ros::Subscriber stereo2_image_left_sub = nh.subscribe("/viobot/zc_stereo2/image_left", 1, stereo2_image_left_cb);
    ros::Subscriber stereo2_keyframe_points_sub = nh.subscribe("/viobot/zc_stereo2/keyframe_points", 1, stereo2_keyframe_points_cb);
    ros::Subscriber stereo2_keyframe_pose_sub = nh.subscribe("/viobot/zc_stereo2/keyframe_pose", 1, stereo2_keyframe_pose_cb);
    ros::Subscriber stereo2_odometry_sub = nh.subscribe("/viobot/zc_stereo2/odometry", 1, stereo2_odometry_cb);
    ros::Subscriber stereo2_odometry_path_sub = nh.subscribe("/viobot/zc_stereo2/odometry_path", 1, stereo2_odometry_path_cb);
    ros::Subscriber stereo2_parray_sub = nh.subscribe("/viobot/zc_stereo2/parray", 1, stereo2_parray_cb);
    ros::Subscriber stereo2_zc_stereo2_points_sub = nh.subscribe("/viobot/zc_stereo2/points", 1, stereo2_zc_stereo2_points_cb);
    ros::Subscriber stereo2_warped_img_sub = nh.subscribe("/viobot/zc_stereo2/warped_img", 1, stereo2_warped_img_cb);


    ros::Time time_now = ros::Time::now();
    ros::Time time_last = ros::Time::now();
    time_last.sec = time_last.sec - 10;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
