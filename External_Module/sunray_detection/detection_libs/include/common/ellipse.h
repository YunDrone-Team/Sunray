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

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <dirent.h>

#include "common/common.h"
namespace sunray_detection
{
    // Elliptical struct definition
    class Ellipse
    {
    public:
        float xc_;
        float yc_;
        float a_;
        float b_;
        float rad_;
        float score_;

        // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + 1 = 0
        float A_;
        float B_;
        float C_;
        float D_;
        float E_;
        float F_;

        Ellipse();
        Ellipse(float xc, float yc, float a, float b, float rad, float score = 0.f);
        Ellipse(const Ellipse &other);

        void Draw(cv::Mat &img, const cv::Scalar &color, const int thickness);
        void Draw(cv::Mat3b &img, const int thickness);

        bool operator<(const Ellipse &other) const;

        // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
        void TransferFromGeneral();

        // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
        void TransferToGeneral();

        void GetRectangle(cv::Rect &rect) const;

        float Perimeter();

        float Area();

        bool IsValid();
    };

    // Data available after selection strategy.
    // They are kept in an associative array to:
    // 1) avoid recomputing data when starting from same arcs
    // 2) be reused in firther proprecessing
    struct EllipseData
    {
        bool isValid;
        float ta;              // arc_a center line gradient
        float tb;              // arc_b
        float ra;              // gradient of a (slope of start of chord_1 and center of chord_2)
        float rb;              // gradient of b (slope of center of chord_1 and last of chord_2)
        cv::Point2f Ma;        // arc_a center of element
        cv::Point2f Mb;        // arc_b
        cv::Point2f Cab;       // center of ellipse
        std::vector<float> Sa; // arc_a's center line of parallel chords
        std::vector<float> Sb; // arc_b's center line of parallel chords
    };

    struct EllipseThreePoint
    {
        bool isValid;
        cv::Point Cab;
        std::vector<cv::Point> ArcI;
        std::vector<cv::Point> ArcJ;
        std::vector<cv::Point> ArcK;
    };
}
