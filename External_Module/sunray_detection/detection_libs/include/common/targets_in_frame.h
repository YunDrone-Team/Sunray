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

#include <vector>
#include <string>

#include "common/common.h"
#include "common/box.h"
#include "common/target.h"
#include "common/ellipse.h"

namespace sunray_detection
{
    class TargetsInFrame
    {
    public:
        TargetsInFrame(int frame_id_);

        //! Frame number.
        int frame_id;
        //! Frame/image height.
        int height;
        //! Frame/image width.
        int width;

        //! Detection frame per second (FPS).
        double fps;
        //! The x-axis field of view (FOV) of the current camera.
        double fov_x;
        //! The y-axis field of view (FOV) of the current camera.
        double fov_y;

        //! 吊舱俯仰角
        double pod_patch;
        //! 吊舱滚转角
        double pod_roll;
        //! 吊舱航向角，东向为0，东北天为正，范围[-180,180]
        double pod_yaw;

        //! 当前经度
        double longitude;
        //! 当前纬度
        double latitude;
        //! 当前飞行高度
        double altitude;

        //! 飞行速度，x轴，东北天坐标系
        double uav_vx;
        //! 飞行速度，y轴，东北天坐标系
        double uav_vy;
        //! 飞行速度，z轴，东北天坐标系
        double uav_vz;
        //! 当前光照强度，Lux
        double illumination;

        //! Whether the detection FPS can be obtained.
        bool has_fps;
        //! Whether the FOV can be obtained.
        bool has_fov;
        //! Whether the processed image sub-region can be obtained.
        bool has_roi;

        bool has_pod_info;
        bool has_uav_pos;
        bool has_uav_vel;
        bool has_ill;

        MissionType type;

        //! The processed image sub-region, if size>0, it means no full image detection.
        std::vector<Box> rois;
        //! Detected Target Instances.
        std::vector<Target> targets;
        std::string date_captured;

        void setTimeNow();
        void setFPS(double fps_);
        void setFOV(double fov_x_, double fov_y_);
        void setSize(int width_, int height_);
        std::string getJsonStr();
    };
    void drawTargetsInFrame(
        cv::Mat &img_,
        const TargetsInFrame &tgts_,
        bool with_all = true,
        bool with_category = false,
        bool with_tid = false,
        bool with_seg = false,
        bool with_box = false,
        bool with_ell = false,
        bool with_aruco = false,
        bool with_yaw = false);
    cv::Mat drawTargetsInFrame(
        const cv::Mat img_,
        const TargetsInFrame &tgts_,
        const double scale,
        bool with_all,
        bool with_category,
        bool with_tid,
        bool with_seg,
        bool with_box);
}
