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
#include "aidetection_yolov7_detector.h"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(sunray_detection::aidetection_yolov7_ros::AIDetectorYolov7, nodelet::Nodelet);

namespace sunray_detection
{

    namespace aidetection_yolov7_ros
    {
        void AIDetectorYolov7::onInit()
        {
            ros::NodeHandle &nh = getNodeHandle();
            ros::NodeHandle &pnh = getPrivateNodeHandle();

            it_ = std::make_shared<image_transport::ImageTransport>(nh);

            // 从参数服务器加载文件路径
            std::string camera_params_file;
            pnh.param<std::string>("camera_parameters_file", camera_params_file, "");

            // 调用封装的函数读取相机参数
            loadCameraParams(camera_params_file, camera_matrix_, dist_coeffs_, image_width_, image_height_, fov_x_, fov_y_);

            std::string algorithm_params_file;
            pnh.param<std::string>("algorithm_parameters_file", algorithm_params_file, "");

            LoadAlgorithmParams(algorithm_params_file, category_map_, object_points_map_);

            // 加载其他参数（保持原逻辑）
            std::string transport_hint;
            pnh.param<std::string>("transport_hint", transport_hint, "raw");

            int queue_size;
            int uav_id;
            std::string uav_name;
            pnh.param<int>("queue_size", queue_size, 1);
            pnh.getParam("uav_id", uav_id);
            pnh.param<std::string>("uav_name", uav_name, "uav");
            pnh.getParam("draw_detections_image", draw_detections_image_);
            pnh.getParam("algorithm_parameters", algorithm_params_json_);
            pnh.getParam("local_saving_path", local_saving_path_);

            pnh.getParam("platform", platform_);
            pnh.getParam("model_path", model_path_);
            pnh.getParam("npu_id", npu_id_);
            pnh.getParam("input_size", input_size_);
            pnh.getParam("num_classes", num_classes_);
            pnh.getParam("conf_thres", conf_thres_);
            pnh.getParam("iou_thres", iou_thres_);

            uav_name = "/" + uav_name + std::to_string(uav_id);

            ROS_INFO_STREAM("Loaded uav_id: " << uav_id);
            ROS_INFO_STREAM("Loaded model_path: " << model_path_);
            ROS_INFO_STREAM("Loaded npu_id: " << npu_id_);
            ROS_INFO_STREAM("Loaded input_size: " << input_size_);

            ROS_INFO_STREAM("Draw detections image: " << draw_detections_image_);
            ROS_INFO_STREAM("Loaded algorithm parameters from: " << algorithm_params_json_);
            ROS_INFO_STREAM("Local saving path: " << local_saving_path_);

            resize_img_ = cv::Mat(input_size_, input_size_, CV_8UC3);
            ai_detector_ = std::shared_ptr<NPUDetector>(new NPUDetector(model_path_.c_str(), npu_id_));
            ai_detector_->setPostProcessCallback(std::bind(&AIDetectorYolov7::post_process, this, std::placeholders::_1, std::placeholders::_2));
            proposals_.resize(this->num_classes_);

            camera_image_subscriber_ =
                it_->subscribe("image_rect", queue_size,
                               &AIDetectorYolov7::imageCallback, this,
                               image_transport::TransportHints(transport_hint));

            refresh_params_service_ =
                pnh.advertiseService("refresh_params",
                                     &AIDetectorYolov7::refreshParamsCallback, this);
            ai_detections_publisher_ =
                nh.advertise<detection_msgs::TargetsInFrameMsg>(uav_name + "/sunray_detect/aidetection_ros", 1);
            ai_detections_image_publisher_ = it_->advertise(uav_name + "/sunray_detect/aidetection_ros/image_rect", 1);
        }

        void AIDetectorYolov7::refreshParameters()
        {
            // Resetting the ai detector will cause a new param server lookup
            // So if the parameters have changed (by someone/something),
            // they will be updated dynamically
            std::lock_guard<std::mutex> lock(detection_mutex_);

            ai_detector_.reset(new NPUDetector(model_path_.c_str(), npu_id_));
            ai_detector_->setPostProcessCallback(std::bind(&AIDetectorYolov7::post_process, this, std::placeholders::_1, std::placeholders::_2));
        }

        bool AIDetectorYolov7::refreshParamsCallback(std_srvs::Empty::Request &req,
                                                     std_srvs::Empty::Response &res)
        {
            refreshParameters();
            return true;
        }
        void AIDetectorYolov7::imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
        {

            std::lock_guard<std::mutex> lock(detection_mutex_);
            cv::Mat cp_image; // 拷贝图像，用于绘制框
            try
            {
                if (image_rect->encoding == "bgr8")
                {
                    cv::cvtColor(cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image, cv_image_, cv::COLOR_BGR2RGB);
                }
                else if (image_rect->encoding == "rgb8")
                {
                    cv_image_= cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image;
                }
                else if (image_rect->encoding == "mono8")
                {
                    cv::cvtColor(cv_bridge::toCvCopy(image_rect, image_rect->encoding)->image, cv_image_, cv::COLOR_GRAY2RGB);
                }
                else
                {
                    ROS_ERROR("Unsupported image encoding: %s", image_rect->encoding.c_str());
                }
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            if (cv_image_.empty())
            {
                ROS_ERROR("cv_image_ is empty");
                return;
            }

            cp_image = cv_image_.clone();

           // cv::imwrite("/home/PRR/image.png",  cp_image);

            // TODO: frame_id_ is not used
            TargetsInFrame tgts(0);
            tgts.width = image_rect->width;
            tgts.height = image_rect->height;
            auto t1 = std::chrono::high_resolution_clock::now();
            ai_detector_->detect(cv_image_, tgts);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            ROS_INFO_STREAM("Detected: " << tgts.targets.size() << " objects, cost: " << duration << " ms");

            // 绘制检测框
            if (draw_detections_image_)
            {
                drawTargetsInFrame(cp_image, tgts);
            }

            // 发布
            detection_msgs::TargetsInFrameMsg targets_in_frame_msg;
            targets_in_frame_msg.header = image_rect->header;
            for (const auto &result : tgts.targets)
            {
                detection_msgs::TargetMsg target_msg;

                // Normalize the bounding box coordinates
                target_msg.cx = result.cx;
                target_msg.cy = result.cy;
                target_msg.w = result.w;
                target_msg.h = result.h;

                // Set the detection score
                target_msg.score = result.score;

                // Set the category information
                target_msg.category_id = result.category_id;
                target_msg.category = result.category; // You need to implement this function or use a map

                float x1 = static_cast<float>(result.cx + result.w / 2);
                float y1 = static_cast<float>(result.cy - result.h / 2);
                float x2 = static_cast<float>(result.cx - result.w / 2);
                float y2 = static_cast<float>(result.cy + result.h / 2);

                std::vector<cv::Point2f> image_points = {
                    {x1, y1}, // 右肩 -> bbox top right
                    {x2, y1}, // 左肩 -> bbox top left
                    {x1, y2}, // 右脚 -> bbox bottom right
                    {x2, y2}  // 左脚 -> bbox bottom left
                };

                Eigen::Isometry3d transform;
                // if (object_points_map_.find(result.category) != object_points_map_.end())
                // {
                //     transform = computeTransform(object_points_map_[result.category], image_points, camera_matrix_, dist_coeffs_);
                // }

                geometry_msgs::PoseWithCovarianceStamped pose_msg;
                pose_msg.header = image_rect->header;

                // target_msg.px = transform.translation().x();
                // target_msg.py = transform.translation().y();
                // target_msg.pz = transform.translation().z();

                target_msg.px = 0.0f;
                target_msg.py = 0.0f;
                target_msg.pz = 0.0f;

                // Initialize other fields (tracked_id, 3D position, etc.)
                target_msg.tracked_id = -1; // Default to -1 if tracking is not available
                target_msg.los_ax = 0.0f;
                target_msg.los_ay = 0.0f;
                target_msg.yaw = 0.0f;
                target_msg.pitch = 0.0f;
                target_msg.roll = 0.0f;

                // Add the target to the message
                targets_in_frame_msg.targets.push_back(target_msg);
            }
            ai_detections_publisher_.publish(targets_in_frame_msg);
            if (draw_detections_image_)
            {
                sensor_msgs::ImagePtr detection_image_msg = cv_bridge::CvImage(image_rect->header, "rgb8", cp_image).toImageMsg();



                ai_detections_image_publisher_.publish(detection_image_msg);
            }
        }
        void AIDetectorYolov7::decode(float *input, int *anchor, int grid_size, int stride,
                                      std::vector<std::vector<Object>> *proposals)
        {
            int grid_len = grid_size * grid_size;
            for (int a = 0; a < 3; a++)
            {
                for (int i = 0; i < grid_size; i++)
                {
                    for (int j = 0; j < grid_size; j++)
                    {
                        float box_confidence = input[((this->num_classes_ + 5) * a + 4) * grid_len + i * grid_size + j];
                        if (box_confidence >= conf_thres_)
                        {
                            int offset = ((this->num_classes_ + 5) * a) * grid_len + i * grid_size + j;
                            float *in_ptr = input + offset;

                            float maxClassProbs = in_ptr[5 * grid_len];
                            int maxClassId = 0;
                            for (int k = 1; k < this->num_classes_; ++k)
                            {
                                float prob = in_ptr[(5 + k) * grid_len];
                                if (prob > maxClassProbs)
                                {
                                    maxClassId = k;
                                    maxClassProbs = prob;
                                }
                            }
                            float prob = box_confidence * maxClassProbs;
                            if (prob < conf_thres_)
                            {
                                continue;
                            }
                            float box_x = in_ptr[0] * 2.0 - 0.5;
                            float box_y = in_ptr[grid_len] * 2.0 - 0.5;
                            float box_w = in_ptr[2 * grid_len] * 2.0;
                            float box_h = in_ptr[3 * grid_len] * 2.0;
                            box_x = (box_x + j) * static_cast<float>(stride);
                            box_y = (box_y + i) * static_cast<float>(stride);
                            box_w = box_w * box_w * static_cast<float>(anchor[a * 2]);
                            box_h = box_h * box_h * static_cast<float>(anchor[a * 2 + 1]);
                            box_x -= (box_w / 2.0);
                            box_y -= (box_h / 2.0);

                            Object obj;
                            obj.rect.x = box_x;
                            obj.rect.y = box_y;
                            obj.rect.width = box_w;
                            obj.rect.height = box_h;
                            obj.label = maxClassId;
                            obj.prob = prob;
                            (*proposals)[maxClassId].push_back(obj);
                        }
                    }
                }
            }
        }

        Eigen::Isometry3d AIDetectorYolov7::computeTransform(
            const std::vector<cv::Point3f> &object_points,
            const std::vector<cv::Point2f> &image_points,
            const cv::Mat &camera_matrix,
            const cv::Mat &dist_coeffs)
        {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

            // 检查输入点的数量
            if (object_points.size() < 4 || image_points.size() < 4)
            {
                ROS_WARN("Warnning: At least 4 points are required for PnP solving");
                return T;
            }

            // 检查两组点的数量是否匹配
            if (object_points.size() != image_points.size())
            {
                ROS_WARN("Error: Number of object points and image points must match");
                return T;
            }

            // 检查相机矩阵
            if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3)
            {
                ROS_WARN("Error: Invalid camera matrix");
                return T;
            }

            cv::Mat rvec, tvec;
            try
            {
                // 使用 try-catch 捕获可能的 OpenCV 异常
                // bool success = cv::solvePnPRansac(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, 500, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_EPNP);
                bool success = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
                if (!success)
                {
                    ROS_WARN("Warning: PnP solving failed");
                    return T;
                }

                cv::Mat R;
                cv::Rodrigues(rvec, R);

                // 检查旋转矩阵的有效性
                if (R.empty() || R.rows != 3 || R.cols != 3)
                {
                    ROS_WARN("Error: Invalid rotation matrix");
                    return T;
                }

                T.linear() << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

                T.translation() << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
            }
            catch (const cv::Exception &e)
            {
                ROS_WARN("OpenCV error: %s", e.what());
                return T;
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Error: %s", e.what());
                return T;
            }

            return T;
        }

        bool AIDetectorYolov7::LoadAlgorithmParams(
            const std::string &file_path,
            std::map<int, std::string> &category_map,
            std::map<std::string, std::vector<cv::Point3f>> &object_points_map)
        {
            try
            {
                // 加载 YAML 文件
                YAML::Node config = YAML::LoadFile(file_path);

                // 加载 names 映射
                if (!config["names"] || !config["names"].IsMap())
                {
                    ROS_ERROR("Invalid or missing 'names' section in YAML file.");
                    return false;
                }
                for (auto it = config["names"].begin(); it != config["names"].end(); ++it)
                {
                    int id = it->first.as<int>();
                    std::string name = it->second.as<std::string>();
                    category_map[id] = name;
                }

                // 加载 object_points 映射
                if (!config["object_points"] || !config["object_points"].IsMap())
                {
                    ROS_ERROR("Invalid or missing 'object_points' section in YAML file.");
                    return false;
                }
                for (auto it = config["object_points"].begin(); it != config["object_points"].end(); ++it)
                {
                    std::string category = it->first.as<std::string>();
                    std::vector<cv::Point3f> points;

                    for (const auto &point : it->second)
                    {
                        if (!point.IsSequence() || point.size() != 3)
                        {
                            ROS_WARN("Invalid point format for category '%s'. Each point must have 3 coordinates.", category.c_str());
                            continue;
                        }
                        points.emplace_back(point[0].as<float>(), point[1].as<float>(), point[2].as<float>());
                    }

                    object_points_map[category] = points;
                }

                return true;
            }
            catch (const YAML::Exception &e)
            {
                ROS_ERROR_STREAM("Failed to parse YAML file: " << e.what());
                return false;
            }
        }

        void AIDetectorYolov7::post_process(const std::vector<void *> &input, std::vector<Target> *target)
        {
            if (this->platform_ == "nx")
            {
                int32_t *nums = static_cast<int32_t *>(input[0]);    // 检测到的目标数量
                float *bboxes = static_cast<float *>(input[1]);      // 边界框坐标
                float *scores = static_cast<float *>(input[2]);      // 置信度
                int32_t *classes = static_cast<int32_t *>(input[3]); // 类别ID

                int num_objects = nums[0]; // 检测到的目标数量

                for (int i = 0; i < num_objects; i++)
                {
                    Target box;

                    // 获取边界框坐标
                    float x1 = bboxes[i * 4 + 0];
                    float y1 = bboxes[i * 4 + 1];
                    float x2 = bboxes[i * 4 + 2];
                    float y2 = bboxes[i * 4 + 3];

                    // 转换为整数坐标
                    int ix1 = static_cast<int>(std::floor(x1));
                    int iy1 = static_cast<int>(std::floor(y1));
                    int ix2 = static_cast<int>(std::floor(x2));
                    int iy2 = static_cast<int>(std::floor(y2));

                    // 设置目标框
                    box.setBox(ix1, iy1, ix2, iy2, input_size_, input_size_);

                    // 设置类别ID和名称
                    box.category_id = classes[i];
                    box.category = getCategoryName(box.category_id);

                    // 设置置信度
                    box.score = scores[i];

                    // 添加到目标列表
                    target->push_back(box);
                }
            }
            else if (this->platform_ == "viobot")
            {
                for (int i = 0; i < static_cast<int>(input.size()); i++)
                {
                    int stride = strides[i];              // 当前特征层的步幅
                    int grid_size = input_size_ / stride; // 当前特征层的网格大小
                    this->decode(reinterpret_cast<float *>(input[i]), reinterpret_cast<int *>(this->anchors[i]), grid_size, stride, &proposals_);
                }

                for (int i = 0; i < this->num_classes_; i++)
                {
                    if (proposals_[i].empty())
                    {
                        continue; // 如果当前类别没有候选框，跳过
                    }

                    // 按置信度降序排序 proposals
                    std::sort(proposals_[i].begin(), proposals_[i].end(), compare);
                    std::vector<int> picked;

                    // 执行 NMS 操作
                    nms_sorted_bboxes(proposals_[i], &picked, this->iou_thres_);

                    int count = static_cast<int>(picked.size());
                    if (count == 0)
                    {
                        continue;
                    }

                    // 解析 NMS 筛选后的候选框
                    for (int j = 0; j < count; ++j)
                    {
                        auto rect = proposals_[i][picked[j]].rect;
                        Target box;
                        auto x1 = static_cast<int>(std::floor(rect.x));
                        auto y1 = static_cast<int>(std::floor(rect.y));
                        auto x2 = static_cast<int>(std::floor(rect.width + x1));
                        auto y2 = static_cast<int>(std::floor(rect.height + y1));
                        box.setBox(x1, y1, x2, y2, this->input_size_, this->input_size_);
                        box.category_id = proposals_[i][picked[j]].label;
                        box.category = getCategoryName(box.category_id);
                        box.score = proposals_[i][picked[j]].prob;
                        target->push_back(box);
                    }

                    proposals_[i].clear(); // 清理当前类别的 proposals
                }
            }
            else
            {
                ROS_ERROR("Unsupported platform: %s", this->platform_.c_str());
            }
        }
        std::string AIDetectorYolov7::getCategoryName(int label)
        {
            auto it = category_map_.find(label);
            if (it != category_map_.end())
            {
                return it->second;
            }
            else
            {
                return "unknown";
            }
        }
    }
}
