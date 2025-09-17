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
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include "common/common.h"
#include "common/box.h"
#include "common/ellipse.h"

namespace sunray_detection
{
  class Target
  {
  public:
    Target();

    //! X coordinate of object center point, [0, 1], (Required)
    double cx;
    //! Y coordinate of object center point, [0, 1], (Required)
    double cy;
    //! Object-width / image-width, (0, 1]
    double w;
    //! Object-height / image-heigth, (0, 1]
    double h;

    //! Objectness, Confidence, (0, 1]
    double score;
    //! Category of target.
    std::string category;
    //! Category ID of target.
    int category_id;
    //! The same target in different frames shares a unique ID.
    int tracked_id;

    //! X coordinate of object position in Camera-Frame (unit: meter).
    double px;
    //! Y coordinate of object position in Camera-Frame (unit: meter).
    double py;
    //! Z coordinate of object position in Camera-Frame (unit: meter).
    double pz;

    //! Line of sight (LOS) angle on X-axis (unit: degree).
    double los_ax;
    //! Line of sight (LOS) angle on Y-axis (unit: degree).
    double los_ay;
    //! The angle of the target in the image coordinate system,  (unit: degree) [-180, 180].
    double yaw;
    double roll;
    double pitch;

    //! Similarity, Confidence, (0, 1]
    double sim_score;

    //! Whether the height&width of the target can be obtained.
    bool has_hw;
    //! Whether the category of the target can be obtained.
    bool has_category;
    //! Whether the tracking-ID of the target can be obtained.
    bool has_tid;
    //! Whether the 3D-position of the target can be obtained.
    bool has_position;
    //! Whether the LOS-angle of the target can be obtained.
    bool has_los;
    //! Whether the segmentation of the target can be obtained.
    bool has_seg;
    //! Whether the bounding-box of the target can be obtained.
    bool has_box;
    //! Whether the aruco-parameters of the target can be obtained.
    bool has_aruco;
    //! Whether the direction of the target can be obtained.
    bool has_yaw;
    //! Whether the ellipse of the target can be obtained.
    bool has_ellipse;

    void setCategory(std::string cate_, int cate_id_);
    void setCategory(std::string cate_, int cate_id_, float score_);
    void setLOS(double cx_, double cy_, cv::Mat camera_matrix_, int img_w_, int img_h_);
    void setTrackID(int id_);
    void setPosition(double x_, double y_, double z_);
    void setBox(int x1_, int y1_, int x2_, int y2_, int img_w_, int img_h_);
    void setAruco(int id_, std::vector<cv::Point2f> corners_, cv::Vec3d rvecs_, cv::Vec3d tvecs_, int img_w_, int img_h_, cv::Mat camera_matrix_);
    void setYaw(double vec_x_, double vec_y);
    void setPitchAndRoll(const cv::Vec3d &rvecs_);
    void setMask(cv::Mat mask_);
    cv::Mat getMask();
    void setEllipse(double xc_, double yc_, double a_, double b_, double rad_, double score_, int img_w_, int img_h_, cv::Mat camera_matrix_, double radius_in_meter_);
    bool getEllipse(double &xc_, double &yc_, double &a_, double &b_, double &rad_);
    bool getAruco(int &id, std::vector<cv::Point2f> &corners);
    bool getAruco(int &id, std::vector<cv::Point2f> &corners, cv::Vec3d &rvecs, cv::Vec3d &tvecs);
    std::string getJsonStr();

  private:
    //! segmentation [[x1,y1, x2,y2, x3,y3,...],...]
    /*!
      SEG variables: (_s_) segmentation, segmentation_size_h, segmentation_size_w, segmentation_counts, area
    */
    std::vector<std::vector<double>> _s_segmentation;
    int _s_segmentation_size_h;
    int _s_segmentation_size_w;
    std::string _s_segmentation_counts;
    cv::Mat _mask;
    double _s_area;
    //! bounding box [x, y, w, h]
    /*!
      BOX variables: (_b_) box
    */
    /*!
      ELL variables: (_e_) xc, yc, a, b, rad
    */
    double _e_xc;
    double _e_yc;
    double _e_a;
    double _e_b;
    double _e_rad;
    //! Aruco Marker ID
    /*!
      ARUCO variables: (_a_) id, corners, rvecs, tvecs
    */
    int _a_id;
    std::vector<cv::Point2f> _a_corners;
    cv::Vec3d _a_rvecs;
    cv::Vec3d _a_tvecs;
  };

}
