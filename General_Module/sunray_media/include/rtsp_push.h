#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <unistd.h>

class RTSP_Push
{
private:
    std::string rtsp_url;      // 推流地址
    std::string sub_topic;     // 订阅的图像话题主题
    std::string ffmpeg_cmd;    // ffmpeg推流命令
    cv::Mat img_frame;         // 推流的图像帧
    int frame_height;          // 推流的图像高度
    int frame_width;           // 推流的图像宽度
    int frame_fps;             // 推流的图像帧率
    FILE *pipe;                // ffmpeg推流管道
    ros::Subscriber image_sub; // 订阅图像话题
    ros::Timer push_timer;      // 定时器
public:
    RTSP_Push(ros::NodeHandle &nh);
    ~RTSP_Push();
    // 图像话题回调函数
    void image_callback(const sensor_msgs::ImageConstPtr &msg);
    // 定时推流函数
    void push_image(const ros::TimerEvent &event);
};
