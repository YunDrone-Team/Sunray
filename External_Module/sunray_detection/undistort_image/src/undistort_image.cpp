#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher left_image_pub, right_image_pub;

cv::Mat image_left, image_right;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC, CAMERA0_INFO_TOPIC, CAMERA1_INFO_TOPIC;

double left_fx, left_fy, left_cx, left_cy;
double left_xi, left_alpha;
double right_fx, right_fy, right_cx, right_cy;
double right_xi, right_alpha;

cv::Mat map_x_L, map_y_L;
cv::Mat map_x_R, map_y_R;

bool camera_info_init;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> CameraSyncPolicy;

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera0_info_msg, const sensor_msgs::CameraInfoConstPtr &camera1_info_msg)
{
    // 退出订阅
    if (camera_info_init)
        return;

    if (camera0_info_msg->distortion_model == "ds")
    {
        int width = camera0_info_msg->width;
        int height = camera0_info_msg->height;

        cv::Mat map(height, width, CV_32FC1);
        cv::Mat map_stereorectify_cam0_x, map_stereorectify_cam0_y, map_stereorectify_cam1_x, map_stereorectify_cam1_y;
        cv::Mat map_undistort_cam0_x, map_undistort_cam0_y, map_undistort_cam1_x, map_undistort_cam1_y;

        // 设置参数
        left_fx = camera0_info_msg->K[0];
        left_fy = camera0_info_msg->K[4];
        left_cx = camera0_info_msg->K[2];
        left_cy = camera0_info_msg->K[5];
        left_xi = camera0_info_msg->D[0];
        left_alpha = camera0_info_msg->D[1];

        right_fx = camera1_info_msg->K[0];
        right_fy = camera1_info_msg->K[4];
        right_cx = camera1_info_msg->K[2];
        right_cy = camera1_info_msg->K[5];
        right_xi = camera1_info_msg->D[0];
        right_alpha = camera1_info_msg->D[1];

        // 计算双目校正映射
        cv::Mat Q;
        cv::Mat K_new_L, K_new_R, _R_L, _R_R;
        cv::Mat K_left = (cv::Mat_<double>(3, 3) << left_fx, 0, left_cx, 0, left_fy, left_cy, 0, 0, 1);
        cv::Mat D_left = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
        cv::Mat K_right = (cv::Mat_<double>(3, 3) << right_fx, 0, right_cx, 0, right_fy, right_cy, 0, 0, 1);
        cv::Mat D_right = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
        cv::Mat _extrinsic_R = (cv::Mat_<double>(3, 3) << camera1_info_msg->R[0], camera1_info_msg->R[1], camera1_info_msg->R[2],
                                camera1_info_msg->R[3], camera1_info_msg->R[4], camera1_info_msg->R[5],
                                camera1_info_msg->R[6], camera1_info_msg->R[7], camera1_info_msg->R[8]);
        cv::Mat _extrinsic_T = (cv::Mat_<double>(3, 1) << camera1_info_msg->P[3], camera1_info_msg->P[7], camera1_info_msg->P[11]);

        cv::stereoRectify(K_left, D_left, K_right, D_right, cv::Size(width, height), _extrinsic_R, _extrinsic_T, _R_L, _R_R, K_new_L, K_new_R, Q);
        cv::initUndistortRectifyMap(K_left, D_left, _R_L, K_new_L, cv::Size(width, height), CV_32FC1, map_stereorectify_cam0_x, map_stereorectify_cam0_y);
        cv::initUndistortRectifyMap(K_right, D_right, _R_R, K_new_R, cv::Size(width, height), CV_32FC1, map_stereorectify_cam1_x, map_stereorectify_cam1_y);

        // 计算去畸映射
        // 左目
        map_undistort_cam0_x = map.clone();
        map_undistort_cam0_y = map.clone();
        for (int v = 0; v < height; v++)
        {
            for (int u = 0; u < width; u++)
            {
                const double x = (double(u) - left_cx) / left_fx;
                const double y = (double(v) - left_cy) / left_fy;

                const double d1 = std::sqrt(x * x + y * y + 1.0);

                // 公式43-45: 检查有效投影区域
                const double w1 = (left_alpha <= 0.5) ? (left_alpha / (1 - left_alpha)) : ((1 - left_alpha) / left_alpha);
                const double w2 = (w1 + left_xi) / sqrt(2 * w1 * left_xi + left_xi * left_xi + 1);
                if (1.0 <= -w2 * d1)
                {
                    map_undistort_cam0_x.at<float>(v, u) = -1;
                    map_undistort_cam0_y.at<float>(v, u) = -1;
                    continue;
                }

                // 公式42: 计算d2
                const double term_z = left_xi * d1 + 1.0;
                const double d2 = sqrt(x * x + y * y + term_z * term_z);

                // 公式40: 计算畸变坐标
                double denominator = left_alpha * d2 + (1 - left_alpha) * term_z;
                if (fabs(denominator) < 1e-9)
                {
                    map_undistort_cam0_x.at<float>(v, u) = -1;
                    map_undistort_cam0_y.at<float>(v, u) = -1;
                    continue;
                }

                const double xd = x / denominator;
                const double yd = y / denominator;

                const double xDistorted = xd * left_fx + left_cx;
                const double yDistorted = yd * left_fy + left_cy;

                map_undistort_cam0_x.at<float>(v, u) = xDistorted;
                map_undistort_cam0_y.at<float>(v, u) = yDistorted;
            }
        }

        // 右目
        map_undistort_cam1_x = map.clone();
        map_undistort_cam1_y = map.clone();
        for (int v = 0; v < height; v++)
        {
            for (int u = 0; u < width; u++)
            {
                const double x = (double(u) - right_cx) / right_fx;
                const double y = (double(v) - right_cy) / right_fy;

                const double d1 = std::sqrt(x * x + y * y + 1.0);

                // 公式43-45: 检查有效投影区域
                const double w1 = (right_alpha <= 0.5) ? (right_alpha / (1 - right_alpha)) : ((1 - right_alpha) / right_alpha);
                const double w2 = (w1 + right_xi) / sqrt(2 * w1 * right_xi + right_xi * right_xi + 1);
                if (1.0 <= -w2 * d1)
                {
                    map_undistort_cam1_x.at<float>(v, u) = -1;
                    map_undistort_cam1_y.at<float>(v, u) = -1;
                    continue;
                }

                // 公式42: 计算d2
                const double term_z = right_xi * d1 + 1.0;
                const double d2 = sqrt(x * x + y * y + term_z * term_z);

                // 公式40: 计算畸变坐标
                double denominator = right_alpha * d2 + (1 - right_alpha) * term_z;
                if (fabs(denominator) < 1e-9)
                {
                    map_undistort_cam1_x.at<float>(v, u) = -1;
                    map_undistort_cam1_y.at<float>(v, u) = -1;
                    continue;
                }

                const double xd = x / denominator;
                const double yd = y / denominator;

                const double xDistorted = xd * right_fx + right_cx;
                const double yDistorted = yd * right_fy + right_cy;

                map_undistort_cam1_x.at<float>(v, u) = xDistorted;
                map_undistort_cam1_y.at<float>(v, u) = yDistorted;
            }
        }

        // 计算最终映射
        map_x_L = map.clone();
        map_y_L = map.clone();
        map_x_R = map.clone();
        map_y_R = map.clone();
        cv::remap(map_undistort_cam0_x, map_x_L, map_stereorectify_cam0_x, map_stereorectify_cam0_y, cv::INTER_CUBIC);
        cv::remap(map_undistort_cam0_y, map_y_L, map_stereorectify_cam0_x, map_stereorectify_cam0_y, cv::INTER_CUBIC);
        cv::remap(map_undistort_cam1_x, map_x_R, map_stereorectify_cam1_x, map_stereorectify_cam1_y, cv::INTER_CUBIC);
        cv::remap(map_undistort_cam1_y, map_y_R, map_stereorectify_cam1_x, map_stereorectify_cam1_y, cv::INTER_CUBIC);

        camera_info_init = true;
        std::cout << "camera_info_init" << std::endl;
    }
    else
    {
        std::cout << "distortion_model error" << std::endl;
    }
}

void img_callback(const sensor_msgs::ImageConstPtr &img0, const sensor_msgs::ImageConstPtr &img1)
{
    if (!camera_info_init)
    {
        ros::Duration(0.1).sleep();
        return;
    }

    image_left = getImageFromMsg(img0);
    image_right = getImageFromMsg(img1);

    cv::remap(image_left, image_left, map_x_L, map_y_L, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = img0->header.stamp;
    out_msg.header.frame_id = "camera";
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = image_left;
    left_image_pub.publish(out_msg.toImageMsg());

    cv::remap(image_right, image_right, map_x_R, map_y_R, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    out_msg.image = image_right;
    right_image_pub.publish(out_msg.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "undistort_image");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("image0_topic", IMAGE0_TOPIC, "/baton/image_left");
    pnh.param<std::string>("image1_topic", IMAGE1_TOPIC, "/baton/image_right");
    pnh.param<std::string>("camera0_info_topic", CAMERA0_INFO_TOPIC, "/baton/camera_left_info");
    pnh.param<std::string>("camera1_info_topic", CAMERA1_INFO_TOPIC, "/baton/camera_right_info");

    cv::setNumThreads(4);
    ROS_INFO("========== undistort start =============");

    left_image_pub = nh.advertise<sensor_msgs::Image>("/baton/undistort_image_left", 10);
    right_image_pub = nh.advertise<sensor_msgs::Image>("/baton/undistort_image_right", 10);

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> left_img_sub_ptr, right_img_sub_ptr;
    std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> img_sync_ptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> left_cam_info_sub_ptr, right_cam_info_sub_ptr;
    std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>> cam_sync_ptr;
    {
        // 订阅image话题
        left_img_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, IMAGE0_TOPIC, 10);
        right_img_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, IMAGE1_TOPIC, 10);
        img_sync_ptr = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *left_img_sub_ptr.get(), *right_img_sub_ptr.get());
        img_sync_ptr->registerCallback(boost::bind(&img_callback, _1, _2));
        // 订阅camera话题
        left_cam_info_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, CAMERA0_INFO_TOPIC, 2);
        right_cam_info_sub_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, CAMERA1_INFO_TOPIC, 2);
        cam_sync_ptr = std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(CameraSyncPolicy(10), *left_cam_info_sub_ptr.get(), *right_cam_info_sub_ptr.get());
        cam_sync_ptr->registerCallback(boost::bind(&camera_info_callback, _1, _2));
    }

    ros::spin();
    ros::shutdown();
    return 0;
}