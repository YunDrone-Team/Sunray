/*************************************************************************************************************************
 * Copyright 2024 Grifcc&Kylin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#include <memory>
#include <mutex>

#include <nodelet/nodelet.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>
#include "common/common.h"
#include "common/targets_in_frame.h"
#include "utils/gason.h"
#include "inference_backend/npu/npu_detector.h"
#include "postprocessor_utils.h"
#include <detection_msgs/TargetMsg.h>
#include <detection_msgs/TargetsInFrameMsg.h>

namespace sunray_detection
{
    namespace aidetection_yolov7_ros
    {
        class AIDetectorYolov7 : public nodelet::Nodelet
        {
        public:
            AIDetectorYolov7() = default;
            ~AIDetectorYolov7() = default;
            void onInit();

            void imageCallback(const sensor_msgs::ImageConstPtr &image_rect);
            void refreshParameters();

        private:
            std::mutex detection_mutex_;
            std::shared_ptr<BaseDetector> ai_detector_;
            bool draw_detections_image_ = false;
            std::string local_saving_path_;
            cv::Mat cv_image_;
            int frame_id_;
            std::string algorithm_params_json_;
            cv::Mat camera_matrix_; // Camera intrinsic matrix
            cv::Mat dist_coeffs_;   // Camera distortion coefficients
            float fov_x_;
            float fov_y_;
            int image_width_;
            int image_height_;

            std::string platform_;
            
            std::string model_path_;
            int npu_id_;
            int input_size_;
            int num_classes_;
            float conf_thres_;
            float iou_thres_;

            int anchors[3][6] = {{10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};
            int strides[3] = {8, 16, 32};
            std::map<int, std::string> category_map_;
            cv::Mat resize_img_;
            std::vector<std::vector<Object>> proposals_;
            std::map<std::string, std::vector<cv::Point3f>> object_points_map_;
            std::shared_ptr<image_transport::ImageTransport> it_;
            image_transport::Subscriber camera_image_subscriber_;
            ros::Publisher ai_detections_publisher_;
            image_transport::Publisher ai_detections_image_publisher_;

            ros::ServiceServer refresh_params_service_;
            bool refreshParamsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
            void post_process(const std::vector<void *> &input, std::vector<Target> *target);
            void decode(float *input, int *anchor, int grid_size, int stride, std::vector<std::vector<Object>> *proposals);
            Eigen::Isometry3d computeTransform(const std::vector<cv::Point3f> &object_points,
                                               const std::vector<cv::Point2f> &image_points,
                                               const cv::Mat &camera_matrix,
                                               const cv::Mat &dist_coeffs);
            std::string getCategoryName(int label);
            void draw_detections(cv::Mat &img, const std::vector<Target> &objects);
            bool LoadAlgorithmParams(
                const std::string &file_path,
                std::map<int, std::string> &category_map,
                std::map<std::string, std::vector<cv::Point3f>> &object_points_map);
        };
    }
}
