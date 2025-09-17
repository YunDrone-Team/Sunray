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
#pragma once
#include <chrono>
#include <ctime>
#include <string>
#include <opencv2/opencv.hpp>

#define CHECK_OPENCV_VERSION(major, minor, patch)               \
    ((CV_MAJOR_VERSION > major) ||                              \
     (CV_MAJOR_VERSION == major && CV_MINOR_VERSION > minor) || \
     (CV_MAJOR_VERSION == major && CV_MINOR_VERSION == minor && CV_SUBMINOR_VERSION >= patch))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define M_2__PI 6.28318530718
#define M_1_2_PI 1.57079632679
#define RAD2DEG 57.2957795

class NonCopyable
{
protected:
    NonCopyable() = default;
    ~NonCopyable() = default;

private:
    NonCopyable(const NonCopyable &) = delete;
    NonCopyable(NonCopyable &&) = delete;
    NonCopyable &operator=(const NonCopyable &) = delete;
    NonCopyable &operator=(NonCopyable &&) = delete;
};

namespace sunray_detection
{
    enum class MissionType
    {
        NONE,
        COMMON_DET,
        TRACKING,
        ARUCO_DET,
        LANDMARK_DET,
        ELLIPSE_DET
    };
    std::string get_time_str();
    void loadCameraParams(std::string yaml_fn_, cv::Mat &camera_matrix, cv::Mat &distortion, int &image_width, int &image_height, float &fov_x, float &fov_y);
}
