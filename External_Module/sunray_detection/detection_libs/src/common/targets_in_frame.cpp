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
#include "common/targets_in_frame.h"

namespace sunray_detection
{
    TargetsInFrame::TargetsInFrame(int frame_id_)
    {
        this->frame_id = frame_id_;
        this->height = -1;
        this->width = -1;
        this->has_fps = false;
        this->has_fov = false;
        this->has_roi = false;
        this->has_pod_info = false;
        this->has_uav_pos = false;
        this->has_uav_vel = false;
        this->has_ill = false;
        this->type = MissionType::NONE;
    }
    void TargetsInFrame::setTimeNow()
    {
        this->date_captured = get_time_str();
    }
    void TargetsInFrame::setSize(int width_, int height_)
    {
        this->width = width_;
        this->height = height_;
    }
    void TargetsInFrame::setFPS(double fps_)
    {
        this->fps = fps_;
        this->has_fps = true;
    }
    void TargetsInFrame::setFOV(double fov_x_, double fov_y_)
    {
        this->fov_x = fov_x_;
        this->fov_y = fov_y_;
        this->has_fov = true;
    }
    std::string TargetsInFrame::getJsonStr()
    {
        std::string json_str = "{";
        char buf[1024];

        if (this->has_fps)
        {
            sprintf(buf, "\"fps\":%.3f,", this->fps);
            json_str += std::string(buf);
        }
        if (this->has_fov)
        {
            sprintf(buf, "\"fov\":[%.3f,%.3f],", this->fov_x, this->fov_y);
            json_str += std::string(buf);
        }
        if (this->has_pod_info)
        {
            sprintf(buf, "\"pod\":[%.3f,%.3f,%.3f],", this->pod_patch, this->pod_roll, this->pod_yaw);
            json_str += std::string(buf);
        }
        if (this->has_uav_pos)
        {
            sprintf(buf, "\"uav_pos\":[%.7f,%.7f,%.3f],", this->longitude, this->latitude, this->altitude);
            json_str += std::string(buf);
        }
        if (this->has_uav_vel)
        {
            sprintf(buf, "\"uav_vel\":[%.3f,%.3f,%.3f],", this->uav_vx, this->uav_vy, this->uav_vz);
            json_str += std::string(buf);
        }
        if (this->has_ill)
        {
            sprintf(buf, "\"ill\":%.3f,", this->illumination);
            json_str += std::string(buf);
        }
        if (this->date_captured.size() > 0)
        {
            sprintf(buf, "\"time\":\"%s\",", this->date_captured.c_str());
            json_str += std::string(buf);
        }

        json_str += "\"rois\":[";
        for (int i = 0; (int)i < this->rois.size(); i++)
        {
            if (i == (int)this->rois.size() - 1)
            {
                sprintf(buf, "[%d,%d,%d,%d]", this->rois[i].x1, this->rois[i].y1, this->rois[i].x2 - this->rois[i].x1, this->rois[i].y2 - this->rois[i].y1);
            }
            else
            {
                sprintf(buf, "[%d,%d,%d,%d],", this->rois[i].x1, this->rois[i].y1, this->rois[i].x2 - this->rois[i].x1, this->rois[i].y2 - this->rois[i].y1);
            }
            json_str += std::string(buf);
        }
        json_str += "],";
        json_str += "\"tgts\":[";
        for (int i = 0; (int)i < this->targets.size(); i++)
        {
            if (i == (int)this->targets.size() - 1)
            {
                json_str += this->targets[i].getJsonStr();
            }
            else
            {
                json_str += this->targets[i].getJsonStr() + ",";
            }
        }
        json_str += "],";
        sprintf(buf, "\"h\":%d,\"w\":%d,\"fid\":%d}", this->height, this->width, this->frame_id);
        json_str += std::string(buf);
        return json_str;
    }

    void drawTargetsInFrame(
        cv::Mat &img_,
        const TargetsInFrame &tgts_,
        bool with_all,
        bool with_category,
        bool with_tid,
        bool with_seg,
        bool with_box,
        bool with_ell,
        bool with_aruco,
        bool with_yaw)
    {
        if (tgts_.rois.size() > 0)
        {
            cv::Mat image_ret;
            cv::addWeighted(img_, 0.5, cv::Mat::zeros(cv::Size(img_.cols, img_.rows), CV_8UC3), 0, 0, image_ret);
            cv::Rect roi = cv::Rect(tgts_.rois[0].x1, tgts_.rois[0].y1, tgts_.rois[0].x2 - tgts_.rois[0].x1, tgts_.rois[0].y2 - tgts_.rois[0].y1);
            img_(roi).copyTo(image_ret(roi));
            image_ret.copyTo(img_);
        }
        std::vector<std::vector<cv::Point2f>> aruco_corners;
        std::vector<int> aruco_ids;
        std::vector<Ellipse> ellipses;
        for (Target tgt : tgts_.targets)
        {
            cv::circle(img_, cv::Point(int(tgt.cx * tgts_.width), int(tgt.cy * tgts_.height)), 4, cv::Scalar(0, 255, 0), 2);
            if ((with_all || with_aruco) && tgt.has_aruco)
            {
                std::vector<cv::Point2f> a_corners;
                int a_id;
                if (tgt.getAruco(a_id, a_corners))
                {
                    aruco_ids.push_back(a_id);
                    aruco_corners.push_back(a_corners);
                }
            }
            if ((with_all || with_box) && tgt.has_box)
            {
                int x1, y1, w, h;
                x1 = int(tgt.cx * img_.cols - tgt.w * img_.cols / 2);
                y1 = int(tgt.cy * img_.rows - tgt.h * img_.rows / 2);
                w = int(tgt.w * img_.cols) + 1;
                h = int(tgt.h * img_.rows) + 1;
                cv::rectangle(img_, cv::Rect(x1, y1, w, h), cv::Scalar(0, 0, 255), 1, 1, 0);
                if ((with_all || with_category) && tgt.has_category)
                {
                    cv::putText(img_, tgt.category, cv::Point(x1, y1 - 4), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 0, 0));
                }
                if ((with_all || with_tid) && tgt.has_tid)
                {
                    char tmp[32];
                    sprintf(tmp, "TID: %d", tgt.tracked_id);
                    cv::putText(img_, tmp, cv::Point(x1, y1 - 14), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 255));
                }
            }
            if ((with_all || with_ell) && tgt.has_ellipse)
            {
                double xc, yc, a, b, rad;
                if (tgt.getEllipse(xc, yc, a, b, rad))
                {
                    ellipses.push_back(Ellipse(xc, yc, a, b, rad, tgt.score));
                }
            }
            if ((with_all || with_seg) && tgt.has_seg)
            {
                cv::Mat mask = tgt.getMask() * 255;
                cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
                mask.convertTo(mask, CV_8UC1);

                cv::resize(mask, mask, cv::Size(img_.cols, img_.rows));
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;

                cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::Mat mask_disp = img_.clone();
                cv::fillPoly(mask_disp, contours, cv::Scalar(255, 255, 255), cv::LINE_AA);
                cv::polylines(img_, contours, true, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

                double alpha = 0.6;
                cv::addWeighted(img_, alpha, mask_disp, 1.0 - alpha, 0, img_);
            }
        }
        if ((with_all || with_aruco) && aruco_ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(img_, aruco_corners, aruco_ids);
        }
    }
    cv::Mat drawTargetsInFrame(
        const cv::Mat img_,
        const TargetsInFrame &tgts_,
        const double scale,
        bool with_all,
        bool with_category,
        bool with_tid,
        bool with_seg,
        bool with_box)
    {
        cv::Mat img_show;
        cv::resize(img_, img_show, cv::Size(0, 0), scale, scale);
        if (tgts_.rois.size() > 0)
        {
            cv::Mat image_ret;
            cv::addWeighted(img_show, 0.5, cv::Mat::zeros(cv::Size(img_show.cols, img_show.rows), CV_8UC3), 0, 0, image_ret);
            cv::Rect roi = cv::Rect((int)(tgts_.rois[0].x1 * scale), (int)(tgts_.rois[0].y1 * scale), (int)((tgts_.rois[0].x2 - tgts_.rois[0].x1) * scale), (int)((tgts_.rois[0].y2 - tgts_.rois[0].y1) * scale));
            img_show(roi).copyTo(image_ret(roi));
            image_ret.copyTo(img_show);
        }

        for (Target tgt : tgts_.targets)
        {
            cv::circle(img_show, cv::Point(int(tgt.cx * tgts_.width * scale), int(tgt.cy * tgts_.height * scale)), 4, cv::Scalar(0, 255, 0), 2);
            if ((with_all || with_box) && tgt.has_box)
            {
                int x1, y1, w, h;
                x1 = int(tgt.cx * img_show.cols - tgt.w * img_show.cols / 2);
                y1 = int(tgt.cy * img_show.rows - tgt.h * img_show.rows / 2);
                w = int(tgt.w * img_show.cols) + 1;
                h = int(tgt.h * img_show.rows) + 1;
                cv::rectangle(img_show, cv::Rect(x1, y1, w, h), cv::Scalar(0, 0, 255), 1, 1, 0);
                if ((with_all || with_category) && tgt.has_category)
                {
                    cv::putText(img_show, tgt.category, cv::Point(x1, y1 - 4), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255, 0, 0));
                }
                if ((with_all || with_tid) && tgt.has_tid)
                {
                    char tmp[32];
                    sprintf(tmp, "TID: %d", tgt.tracked_id);
                    cv::putText(img_show, tmp, cv::Point(x1, y1 - 14), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 0, 255));
                }
            }
            if ((with_all || with_seg) && tgt.has_seg)
            {
                cv::Mat mask = tgt.getMask() * 255;
                cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
                mask.convertTo(mask, CV_8UC1);

                cv::resize(mask, mask, cv::Size(img_show.cols, img_show.rows));
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;

                cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::Mat mask_disp = img_show.clone();
                cv::fillPoly(mask_disp, contours, cv::Scalar(255, 255, 255), cv::LINE_AA);
                cv::polylines(img_show, contours, true, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

                double alpha = 0.6;
                cv::addWeighted(img_show, alpha, mask_disp, 1.0 - alpha, 0, img_show);
            }
        }
        return img_show;
    }
}
