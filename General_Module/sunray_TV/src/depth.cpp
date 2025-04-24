#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Bool.h>

ros::Publisher obstacle_pub;

// 点云话题回调函数
void depthCallback(const sensor_msgs::ImageConstPtr& msg) {

    // 将ROS图像消息转换为OpenCV图像
    cv::Mat depth_image = cv_bridge::toCvShare(msg, "mono16")->image;

    // 下采样到一半
    cv::Mat downsampled_image;
    cv::pyrDown(depth_image, downsampled_image);

    // 提取正中区域
    cv::Rect center_rect(85, 25, 150, 150);
    cv::Mat center_block = downsampled_image(center_rect);

    // 统计正中区域深度小于阈值的像素数量
    int obstacle_pixel_count = 0;

    for (int y = 0; y < center_block.rows; ++y) {

        for (int x = 0; x < center_block.cols; ++x) {

            uint16_t depth_value = center_block.at<uint16_t>(y, x) << 3;
            double depth_in_meters = depth_value * 1.0 / 8000.0;
          //  std::cout << "depth: " << depth_in_meters << std::endl;
            if (depth_in_meters > 0.1 && depth_in_meters <= 1.0) {

                obstacle_pixel_count++;
            }
        }
    }

    // 判断是否有障碍物
    int total_pixel_count = center_block.rows * center_block.cols;
    bool has_obstacle = (static_cast<double>(obstacle_pixel_count) / static_cast<double>(total_pixel_count)) > 0.5;

    // 发布障碍物判断结果
    std_msgs::Bool obstacle_msg;
    obstacle_msg.data = has_obstacle;
    obstacle_pub.publish(obstacle_msg);

    if (has_obstacle) {

        ROS_INFO("Has obstacle!");

    } else {

        ROS_INFO("No obstacle!");
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "viobot");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/berxel_camera/depth/depth_raw", 1, depthCallback);

    obstacle_pub = nh.advertise<std_msgs::Bool>("/has_obstacle", 1);

    ros::spin();

    return 0;
}