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
#include "common/common.h"
namespace sunray_detection
{
    std::string get_time_str()
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::chrono::system_clock::duration tp = now.time_since_epoch();
        tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);

        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        tm t = *std::localtime(&tt);

        char buf[128];
        sprintf(buf, "%4d-%02d-%02d_%02d-%02d-%02d_%03u", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, static_cast<unsigned>(tp / std::chrono::milliseconds(1)));

        return std::string(buf);
    }

   void loadCameraParams(std::string yaml_fn_, cv::Mat &camera_matrix, cv::Mat &distortion, int &image_width, int &image_height, float &fov_x, float &fov_y)
    {
        cv::FileStorage fs(yaml_fn_, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            throw std::runtime_error("Camera calib file NOT exist!");
        }
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> distortion;
        fs["image_width"] >> image_width;
        fs["image_height"] >> image_height;

        if (camera_matrix.rows != 3 || camera_matrix.cols != 3 ||
            distortion.rows != 1 || distortion.cols != 5 ||
            image_width == 0 || image_height == 0)
        {
            throw std::runtime_error("Camera parameters reading ERROR!");
        }
        fov_x = 2 * atan(image_width / 2. / camera_matrix.at<double>(0, 0)) * RAD2DEG;
        fov_y = 2 * atan(image_height / 2. / camera_matrix.at<double>(1, 1)) * RAD2DEG;
    }
}
