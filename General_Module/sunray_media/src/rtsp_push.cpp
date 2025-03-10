#include "rtsp_push.h"

RTSP_Push::RTSP_Push(ros::NodeHandle &nh)
{
    nh.param("frame_width", frame_width, 640);
    nh.param("frame_height", frame_height, 480);
    nh.param("fps", frame_fps, 30);
    nh.param("rtsp_url", rtsp_url, std::string("rtsp://192.168.25.164:8554/live"));
    nh.param("topic_name", sub_topic, std::string("/usb_cam/image_raw"));

    // 构建ffmpeg命令
    std::stringstream ffmpeg_cmd;
    ffmpeg_cmd << "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 "
               << "-s " << frame_width << "x" << frame_height
               << " -r " << frame_fps << " -i - -c:v libx264 -preset ultrafast -f rtsp "
               << rtsp_url;
    // ffmpeg_cmd = "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 -s 640x480 -r 30 -i - -c:v libx264 -preset ultrafast -f flv rtmp://your_rtmp_server/live/stream";
    pipe = popen(ffmpeg_cmd.str().c_str(), "w");
    // 初始化图像帧 填充为全黑
    img_frame = cv::Mat::zeros(frame_height, frame_width, CV_8UC3);
    // 订阅图像话题
    image_sub = nh.subscribe(sub_topic, 1, &RTSP_Push::image_callback, this);

    push_timer = nh.createTimer(ros::Duration(1.0 / frame_fps), &RTSP_Push::push_image, this);
}

RTSP_Push::~RTSP_Push()
{
    pclose(pipe);
}

void RTSP_Push::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // 将图像转为cv格式 并且对其size
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(frame_width, frame_height));
    img_frame = cv_ptr->image;
}

void RTSP_Push::push_image(const ros::TimerEvent &event)
{
    // 将图像数据推送到RTSP服务器
    fwrite(img_frame.data, 1, img_frame.cols * img_frame.rows * 3, pipe);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtsp_push");
    ros::NodeHandle nh("~");
    RTSP_Push rtsp_push(nh);
    ros::spin();
    return 0;
}