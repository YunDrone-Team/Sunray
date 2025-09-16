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
#include "common/target.h"
namespace sunray_detection
{
    Target::Target()
    {
        this->has_hw = false;
        this->has_tid = false;
        this->has_position = false;
        this->has_los = false;
        this->has_seg = false;
        this->has_box = false;
        this->has_aruco = false;
        this->has_yaw = false;
        this->has_ellipse = false;
    }
    void Target::setAruco(int id_, std::vector<cv::Point2f> corners_, cv::Vec3d rvecs_, cv::Vec3d tvecs_, int img_w_, int img_h_, cv::Mat camera_matrix_)
    {
        this->_a_id = id_;
        this->_a_corners = corners_;
        this->_a_rvecs = rvecs_;
        this->_a_tvecs = tvecs_;

        double x_mid = (corners_[0].x + corners_[1].x) / 2.;
        double y_mid = (corners_[0].y + corners_[1].y) / 2.;

        double left = std::min(std::min(corners_[0].x, corners_[1].x), std::min(corners_[2].x, corners_[3].x));
        double right = std::max(std::max(corners_[0].x, corners_[1].x), std::max(corners_[2].x, corners_[3].x));
        double top = std::min(std::min(corners_[0].y, corners_[1].y), std::min(corners_[2].y, corners_[3].y));
        double bottom = std::max(std::max(corners_[0].y, corners_[1].y), std::max(corners_[2].y, corners_[3].y));

        double x_vec = x_mid - (left + right) / 2.;
        double y_vec = y_mid - (top + bottom) / 2.;

        this->setYaw(x_vec, y_vec);
        this->setPitchAndRoll(rvecs_);

        this->score = 1.;
        char cate[256];
        sprintf(cate, "aruco-%d", id_);
        this->setCategory(cate, id_);
        this->setTrackID(id_);
        this->setLOS(this->cx, this->cy, camera_matrix_, img_w_, img_h_);
        this->setPosition(tvecs_[0], tvecs_[1], tvecs_[2]);

        this->has_aruco = true;
    }

    void Target::setPitchAndRoll(const cv::Vec3d &rvecs_)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvecs_, rotation_matrix); // 将旋转向量转换为旋转矩阵

        // 从旋转矩阵中提取 roll 和 pitch 角度
        double sy = sqrt(rotation_matrix.at<double>(0, 0) * rotation_matrix.at<double>(0, 0) +
                         rotation_matrix.at<double>(1, 0) * rotation_matrix.at<double>(1, 0));
        bool singular = sy < 1e-6; // 如果接近奇异值
        if (!singular)
        {
            this->pitch = atan2(rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)) * 180 / CV_PI;
            this->roll = -atan2(-rotation_matrix.at<double>(2, 0), sy) * 180 / CV_PI;
            // yaw = atan2(rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(0, 0));
        }
        else
        {
            this->pitch = atan2(-rotation_matrix.at<double>(1, 2), rotation_matrix.at<double>(1, 1)) * 180 / CV_PI;
            this->roll = -atan2(-rotation_matrix.at<double>(2, 0), sy) * 180 / CV_PI;
            // yaw = 0;
        }
    }

    void Target::setYaw(double vec_x_, double vec_y_)
    {
        if (vec_x_ == 0. && vec_y_ > 0.)
        {
            this->yaw = 180;
        }
        else if (vec_x_ == 0. && vec_y_ < 0.)
        {
            this->yaw = 0;
        }
        else if (vec_x_ > 0. && vec_y_ == 0.)
        {
            this->yaw = 90;
        }
        else if (vec_x_ > 0. && vec_y_ > 0.)
        {
            this->yaw = 180 - atan(vec_x_ / vec_y_) * RAD2DEG;
        }
        else if (vec_x_ > 0. && vec_y_ < 0.)
        {
            this->yaw = atan(vec_x_ / -vec_y_) * RAD2DEG;
        }
        else if (vec_x_ < 0. && vec_y_ == 0.)
        {
            this->yaw = -90;
        }
        else if (vec_x_ < 0. && vec_y_ > 0.)
        {
            this->yaw = atan(-vec_x_ / vec_y_) * RAD2DEG - 180;
        }
        else if (vec_x_ < 0. && vec_y_ < 0.)
        {
            this->yaw = -atan(-vec_x_ / -vec_y_) * RAD2DEG;
        }
        this->has_yaw = true;
    }
    void Target::setEllipse(double xc_, double yc_, double a_, double b_, double rad_, double score_, int img_w_, int img_h_, cv::Mat camera_matrix_, double radius_in_meter_)
    {
        this->_e_xc = xc_;
        this->_e_yc = yc_;
        this->_e_a = a_;
        this->_e_b = b_;
        this->_e_rad = rad_;
        this->score = score_;
        this->has_ellipse = true;

        cv::Rect rect;
        Ellipse ell(xc_, yc_, a_, b_, rad_);
        ell.GetRectangle(rect);
        this->setBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, img_w_, img_h_);
        this->setCategory("ellipse", 0);
        this->setLOS(this->cx, this->cy, camera_matrix_, img_w_, img_h_);

        if (radius_in_meter_ > 0)
        {
            double z = camera_matrix_.at<double>(0, 0) * radius_in_meter_ / b_;
            double x = tan(this->los_ax / RAD2DEG) * z;
            double y = tan(this->los_ay / RAD2DEG) * z;
            this->setPosition(x, y, z);
        }
    }
    void Target::setLOS(double cx_, double cy_, cv::Mat camera_matrix_, int img_w_, int img_h_)
    {
        this->los_ax = atan((cx_ * img_w_ - img_w_ / 2.) / camera_matrix_.at<double>(0, 0)) * RAD2DEG;
        this->los_ay = atan((cy_ * img_h_ - img_h_ / 2.) / camera_matrix_.at<double>(1, 1)) * RAD2DEG;
        this->has_los = true;
    }

    void Target::setCategory(std::string cate_, int cate_id_, float score_)
    {
        this->category = cate_;
        this->category_id = cate_id_;
        this->score = score_;
        this->has_category = true;
    }

    void Target::setCategory(std::string cate_, int cate_id_)
    {
        this->category = cate_;
        this->category_id = cate_id_;
        this->has_category = true;
    }

    void Target::setTrackID(int id_)
    {
        this->tracked_id = id_;
        this->has_tid = true;
    }

    void Target::setPosition(double x_, double y_, double z_)
    {
        // 坐标旋转
        this->px = -y_;
        this->py = x_;
        this->pz = z_;
        this->has_position = true;
    }

    void Target::setBox(int x1_, int y1_, int x2_, int y2_, int img_w_, int img_h_)
    {
        this->cx = (double)(x2_ + x1_) / 2 / img_w_;
        this->cy = (double)(y2_ + y1_) / 2 / img_h_;
        this->w = (double)(x2_ - x1_) / img_w_;
        this->h = (double)(y2_ - y1_) / img_h_;

        // std::cout << this->cx << ", " << this->cy << ", " << this->w << "," << this->h << std::endl;

        this->has_box = true;
        this->has_hw = true;
    }

    void Target::setMask(cv::Mat mask_)
    {
        this->_mask = mask_;
        this->has_seg = true;
    }
    cv::Mat Target::getMask()
    {
        return this->_mask;
    }

    bool Target::getAruco(int &id, std::vector<cv::Point2f> &corners)
    {
        id = this->_a_id;
        corners = this->_a_corners;
        return this->has_aruco;
    }

    bool Target::getAruco(int &id, std::vector<cv::Point2f> &corners, cv::Vec3d &rvecs, cv::Vec3d &tvecs)
    {
        id = this->_a_id;
        corners = this->_a_corners;
        rvecs = this->_a_rvecs;
        tvecs = this->_a_tvecs;
        return this->has_aruco;
    }
    bool Target::getEllipse(double &xc_, double &yc_, double &a_, double &b_, double &rad_)
    {
        xc_ = this->_e_xc;
        yc_ = this->_e_yc;
        a_ = this->_e_a;
        b_ = this->_e_b;
        rad_ = this->_e_rad;
        return this->has_ellipse;
    }
    std::string Target::getJsonStr()
    {
        std::string json_str = "{";
        char buf[1024];
        if (this->has_box)
        {
            sprintf(buf, "\"box(cxcywh)\":[%f,%f,%f,%f],", cx, cy, w, h); // xywh
            json_str += std::string(buf);
        }
        if (this->has_ellipse)
        {
            sprintf(buf, "\"ell\":[%.3f,%.3f,%.3f,%.3f,%.3f],", this->_e_xc, this->_e_yc, this->_e_a, this->_e_b, this->_e_rad); // xyabr
            json_str += std::string(buf);
        }
        if (this->has_yaw)
        {
            sprintf(buf, "\"yaw\":%.3f,", this->yaw);
            json_str += std::string(buf);
        }
        if (this->has_los)
        {
            sprintf(buf, "\"los\":[%.3f,%.3f],", this->los_ax, this->los_ay);
            json_str += std::string(buf);
        }
        if (this->has_position)
        {
            sprintf(buf, "\"pos\":[%.3f,%.3f,%.3f],", this->px, this->py, this->pz);
            json_str += std::string(buf);
        }
        if (this->has_tid)
        {
            sprintf(buf, "\"tid\":%d,", this->tracked_id);
            json_str += std::string(buf);
        }
        if (this->has_category)
        {
            sprintf(buf, "\"cat\":\"%s\",", this->category.c_str());
            json_str += std::string(buf);
        }
        sprintf(buf, "\"sc\":%.3f,\"cet\":[%.3f,%.3f]}", this->score, this->cx, this->cy);
        json_str += std::string(buf);
        return json_str;
    }
}
