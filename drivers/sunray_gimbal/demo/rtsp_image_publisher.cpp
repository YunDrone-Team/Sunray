#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <std_msgs/String.h>
#include <sunray_gimbal/GimbalParams.h>

// 读取相机内参
bool loadCameraInfo(const std::string& yaml_path, sensor_msgs::CameraInfo& cam_info)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        cam_info.width = config["image_width"].as<int>();
        cam_info.height = config["image_height"].as<int>();
        cam_info.distortion_model = config["distortion_model"].as<std::string>();
        cam_info.D = config["distortion_coefficients"]["data"].as<std::vector<double>>();
        std::vector<double> k_data = config["camera_matrix"]["data"].as<std::vector<double>>();
        std::copy(k_data.begin(), k_data.end(), cam_info.K.begin());
        std::vector<double> r_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
        std::copy(r_data.begin(), r_data.end(), cam_info.R.begin());
        std::vector<double> p_data = config["projection_matrix"]["data"].as<std::vector<double>>();
        std::copy(p_data.begin(), p_data.end(), cam_info.P.begin());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("读取CameraInfo YAML失败: %s", e.what());
        return false;
    }
}

std::string current_codec = "h265"; // 默认h265
bool codec_changed = false;

void gimbalParamCallback(const sunray_gimbal::GimbalParams::ConstPtr& msg) {
    std::string new_codec;
    if (msg->encoding_type == 2) new_codec = "h265";
    else if (msg->encoding_type == 1) new_codec = "h264";
    else new_codec = "h264";
    if (new_codec != current_codec) {
        ROS_WARN("检测到编码类型切换: %s", new_codec.c_str());
        current_codec = new_codec;
        codec_changed = true;
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "rtsp_image_publisher");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_pub;

    image_transport::ImageTransport it(nh_pub); // 用全局句柄
    image_transport::Publisher img_pub = it.advertise("/sunray/gimbal_cam/image_raw", 1);
    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    ros::Subscriber gimbal_param_sub = nh_pub.subscribe("/sunray/gimbal_param", 1, gimbalParamCallback);

    std::string rtsp_url, camera_info_path;
    nh.param<std::string>("rtsp_url", rtsp_url, "rtsp://192.168.144.25:8554/main.264");
    nh.param<std::string>("camera_info_path", camera_info_path, "camera.yaml");

    sensor_msgs::CameraInfo camera_info_msg;
    if (!loadCameraInfo(camera_info_path, camera_info_msg)) {
        ROS_ERROR("无法读取相机内参 YAML: %s", camera_info_path.c_str());
        return -1;
    }

    std::string depay, decoder;
    if (current_codec == "h265" || current_codec == "H265" || current_codec == "hevc") {
        depay = "rtph265depay";
        decoder = "avdec_h265";
        ROS_INFO("使用H265解码pipeline");
    } else {
        depay = "rtph264depay";
        decoder = "avdec_h264";
        ROS_INFO("使用H264解码pipeline");
    }
    std::string pipeline = "rtspsrc location=" + rtsp_url + " latency=50 ! " + depay + " ! " + decoder + " ! videoconvert ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        ROS_ERROR("RTSP流无法打开: %s", rtsp_url.c_str());
        return -1;
    }

    cv::Mat frame;
    cv_bridge::CvImage cv_img;
    ros::Rate loop_rate(30);
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        if (codec_changed) {
            cap.release();
            if (current_codec == "h265" || current_codec == "H265" || current_codec == "hevc") {
                depay = "rtph265depay";
                decoder = "avdec_h265";
                ROS_WARN("切换到H265解码pipeline");
            } else {
                depay = "rtph264depay";
                decoder = "avdec_h264";
                ROS_WARN("切换到H264解码pipeline");
            }
            pipeline = "rtspsrc location=" + rtsp_url + " latency=50 ! " + depay + " ! " + decoder + " ! videoconvert ! appsink";
            cap.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                ROS_ERROR("切换后RTSP流无法打开: %s", rtsp_url.c_str());
                return -1;
            }
            codec_changed = false;
        }
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN("未能读取到图像帧");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }
        ros::Time now = ros::Time::now();
        double fps = 1.0 / (now - last_time).toSec();
        last_time = now;
        ROS_INFO_THROTTLE(1.0, "当前帧率: %.2f FPS", fps);
        cv_img.header.stamp = now;
        cv_img.header.frame_id = "usb_cam";
        cv_img.encoding = "bgr8";
        cv_img.image = frame;
        sensor_msgs::ImagePtr msg = cv_img.toImageMsg();
        img_pub.publish(msg);
        camera_info_msg.header.stamp = cv_img.header.stamp;
        camera_info_msg.header.frame_id = cv_img.header.frame_id;
        cam_info_pub.publish(camera_info_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();
    return 0;
}
