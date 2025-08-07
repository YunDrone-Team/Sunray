//@描述：该节点实现云台跟随二维码，处理rtsp转ros

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sunray_msgs/TargetsInFrameMsg.h>
#include <geometry_msgs/Vector3.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <std_msgs/Bool.h>

ros::Publisher gimbal_speed_pub;
ros::Publisher camera_info_pub;
ros::Publisher gimbal_center_cmd_pub;
ros::Time last_detection_time;
bool centered = false;

sensor_msgs::CameraInfo camera_info_msg;

void detectionCallback(const sunray_msgs::TargetsInFrameMsg::ConstPtr& msg)
{
    if (msg->targets.empty())
        return;

    last_detection_time = ros::Time::now();  // 每次识别到就更新时间
    centered = false;  // 标记不是回中状态

    float image_cx = 0.25f;  // 图像中心 cx = 0.25
    float image_cy = 0.05f;  // 图像中心 cy = 0.05

    // 取第一个二维码
    float cx = msg->targets[0].cx;
    float cy = msg->targets[0].cy;

    ROS_INFO("Target position: cx=%.3f, cy=%.3f", cx, cy);

    if(cx>0||cy>0)
    {   
        if(cx>1)
        {
            cx=1;
        }

        float dx = cx - image_cx;
        float dy = cy - image_cy;

        // 控制系数（根据实际调整）
        float gain_px = 60.0;
        float gain_nx = 100.0;

        float gain_py = 200.0;
        float gain_ny = 150.0;

        // 速度控制
        geometry_msgs::Vector3 speed_cmd;
        if(dx>0)
        {
            speed_cmd.x = dx * gain_px; // yaw：左右
            if (speed_cmd.x>30)
            {
                speed_cmd.x = 30;
            }
        } 
        else
        {
            speed_cmd.x = dx * gain_nx;
        }
        
        if(dy<0)
        {
            speed_cmd.y = -dy * gain_py;
        }
        else
        {
            speed_cmd.y = -dy * gain_ny;
            if(speed_cmd.y<-25)
            {
                speed_cmd.y = -25;
            }
        }
        speed_cmd.z = 0;      // roll不动

        gimbal_speed_pub.publish(speed_cmd);
    }
}

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

//定时器回调函数
void checkTimeoutCallback(const ros::TimerEvent&)
{
    if (centered) return;

    ros::Duration since_last = ros::Time::now() - last_detection_time;
    if (since_last.toSec() > 10.0)
    {
        ROS_WARN("10 秒未检测到二维码，触发云台回中");

        std_msgs::Bool msg;
        msg.data = true;
        geometry_msgs::Vector3 speed_cmd;
        speed_cmd.x = 0;
        speed_cmd.y = 0;
        speed_cmd.z = 0;

        gimbal_center_cmd_pub.publish(msg);
        gimbal_speed_pub.publish(speed_cmd);

        centered = true;
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "gimbal_qrtracker");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/sunray/camera/monocular_down/image_raw", 1);

    gimbal_speed_pub = nh.advertise<geometry_msgs::Vector3>("/sunray/gimbal_set_speed", 10);
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/sunray/camera/monocular_down/camera_info", 10);
    gimbal_center_cmd_pub = nh.advertise<std_msgs::Bool>("/sunray/gimbal_center_cmd", 1);

    ros::Subscriber detection_sub = nh.subscribe("/uav1/sunray_detect/qrcode_detection_ros", 10, detectionCallback);

    last_detection_time = ros::Time::now();
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), checkTimeoutCallback);

    std::string rtsp_url;
    nh.param<std::string>("rtsp_url", rtsp_url, "rtsp://192.168.144.25:8554/main.264");

    std::string camera_info_path;
    nh.param<std::string>("camera_info_path", camera_info_path, "camera.yaml");

    if (!loadCameraInfo(camera_info_path, camera_info_msg)) {
        ROS_ERROR("无法读取相机内参 YAML: %s", camera_info_path.c_str());
        return -1;
    }

    // cv::VideoCapture cap(rtsp_url);

    std::string pipeline = "rtspsrc location=" + rtsp_url + " latency=50 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        ROS_ERROR("RTSP流无法打开: %s", rtsp_url.c_str());
        return -1;
    }

    // // 设置读取图像大小（如有必要）
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    cv_bridge::CvImage cv_img;
    ros::Rate loop_rate(30);

    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {

        cap>>frame;
        
        if (frame.empty()) {
            ROS_WARN("未能读取到图像帧");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }


        // 显示帧率
        ros::Time now = ros::Time::now();
        double fps = 1.0 / (now - last_time).toSec();
        last_time = now;
        ROS_INFO_THROTTLE(1.0, "当前帧率: %.2f FPS", fps);

        cv_img.header.stamp = now;
        cv_img.header.frame_id = "usb_cam";
        cv_img.encoding = "bgr8";
        cv_img.image = frame;
        msg = cv_img.toImageMsg();

        pub.publish(msg);

        camera_info_msg.header.stamp = cv_img.header.stamp;
        camera_info_msg.header.frame_id = cv_img.header.frame_id;
        camera_info_pub.publish(camera_info_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    return 0;
}
