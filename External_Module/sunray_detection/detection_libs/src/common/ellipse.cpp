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
#include "common/ellipse.h"

namespace sunray_detection
{
    Ellipse::Ellipse() : xc_(0.f), yc_(0.f), a_(0.f), b_(0.f), rad_(0.f), score_(0.f),
                         A_(0.f), B_(0.f), C_(0.f), D_(0.f), E_(0.f), F_(1.f) {}
    Ellipse::Ellipse(float xc, float yc, float a, float b, float rad, float score) : xc_(xc), yc_(yc), a_(a), b_(b), rad_(rad), score_(score) {}
    Ellipse::Ellipse(const Ellipse &other) : xc_(other.xc_), yc_(other.yc_), a_(other.a_), b_(other.b_), rad_(other.rad_), score_(other.score_),
                                             A_(other.A_), B_(other.B_), C_(other.C_), D_(other.D_), E_(other.E_) {}

    void Ellipse::Draw(cv::Mat &img, const cv::Scalar &color, const int thickness)
    {
        if (IsValid())
            cv::ellipse(img, cv::Point(cvRound(xc_), cvRound(yc_)), cv::Size(cvRound(a_), cvRound(b_)), rad_ * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

    void Ellipse::Draw(cv::Mat3b &img, const int thickness)
    {
        cv::Scalar color(0, cvFloor(255.f * score_), 0);
        if (IsValid())
            cv::ellipse(img, cv::Point(cvRound(xc_), cvRound(yc_)), cv::Size(cvRound(a_), cvRound(b_)), rad_ * 180.0 / CV_PI, 0.0, 360.0, color, thickness);
    }

    bool Ellipse::operator<(const Ellipse &other) const
    { // use for sorting
        if (score_ == other.score_)
        {
            float lhs_e = b_ / a_;
            float rhs_e = other.b_ / other.a_;
            if (lhs_e == rhs_e)
            {
                return false;
            }
            return lhs_e > rhs_e;
        }
        return score_ > other.score_;
    }

    // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    void Ellipse::TransferFromGeneral()
    {
        float denominator = (B_ * B_ - 4 * A_ * C_);

        xc_ = (2 * C_ * D_ - B_ * E_) / denominator;
        yc_ = (2 * A_ * E_ - B_ * D_) / denominator;

        float pre = 2 * (A_ * E_ * E_ + C_ * D_ * D_ - B_ * D_ * E_ + denominator * F_);
        float lst = sqrt((A_ - C_) * (A_ - C_) + B_ * B_);

        a_ = -sqrt(pre * (A_ + C_ + lst)) / denominator;
        b_ = -sqrt(pre * (A_ + C_ - lst)) / denominator;

        if (B_ == 0 && A_ < C_)
            rad_ = 0;
        else if (B_ == 0 && A_ > C_)
            rad_ = CV_PI / 2;
        else
            rad_ = atan((C_ - A_ - lst) / B_);
    }

    // Elliptic General equations Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
    void Ellipse::TransferToGeneral()
    {
        A_ = a_ * a_ * sin(rad_) * sin(rad_) + b_ * b_ * cos(rad_) * cos(rad_);
        B_ = 2.f * (b_ * b_ - a_ * a_) * sin(rad_) * cos(rad_);
        C_ = a_ * a_ * cos(rad_) * cos(rad_) + b_ * b_ * sin(rad_) * sin(rad_);
        D_ = -2.f * A_ * xc_ - B_ * yc_;
        E_ = -B_ * xc_ - 2.f * C_ * yc_;
        F_ = A_ * xc_ * xc_ + B_ * xc_ * yc_ + C_ * yc_ * yc_ - a_ * a_ * b_ * b_;
    }

    void Ellipse::GetRectangle(cv::Rect &rect) const
    {
        float sin_theta = sin(-rad_);
        float cos_theta = cos(-rad_);
        float A = a_ * a_ * sin_theta * sin_theta + b_ * b_ * cos_theta * cos_theta;
        float B = 2 * (a_ * a_ - b_ * b_) * sin_theta * cos_theta;
        float C = a_ * a_ * cos_theta * cos_theta + b_ * b_ * sin_theta * sin_theta;
        float F = -a_ * a_ * b_ * b_;

        float y = sqrt(4 * A * F / (B * B - 4 * A * C));
        float y1 = -abs(y), y2 = abs(y);
        float x = sqrt(4 * C * F / (B * B - 4 * C * A));
        float x1 = -abs(x), x2 = abs(x);

        rect = cv::Rect(int(round(xc_ + x1)), int(round(yc_ + y1)), int(round(x2 - x1)), int(round(y2 - y1)));
    }

    float Ellipse::Perimeter()
    {
        // return 2*CV_PI*b_ + 4*(a_ - b_);
        return CV_PI * (3.f * (a_ + b_) - sqrt((3.f * a_ + b_) * (a_ + 3.f * b_)));
    }

    float Ellipse::Area()
    {
        return CV_PI * a_ * b_;
    }

    bool Ellipse::IsValid()
    {
        bool nan = std::isnan(xc_) || std::isnan(yc_) || std::isnan(a_) || std::isnan(b_) || std::isnan(rad_);
        return !nan;
    }
}
