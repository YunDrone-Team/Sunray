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

namespace sunray_detection
{
    class Box
    {
    public:
        Box();

        int x1;
        int y1;
        int x2;
        int y2;

        //! Set the parameters of the bounding-box by XYXY-format.
        /*!
          \param x1_: The x-axis pixel coordinates of the top-left point.
          \param y1_: The y-axis pixel coordinates of the top-left point.
          \param x2_: The x-axis pixel coordinates of the bottom-right point.
          \param y2_: The y-axis pixel coordinates of the bottom-right point.
        */
        void setXYXY(int x1_, int y1_, int x2_, int y2_);

        //! Set the parameters of the bounding-box by XYWH-format.
        /*!
          \param x1_: The x-axis pixel coordinates of the top-left point.
          \param y1_: The y-axis pixel coordinates of the top-left point.
          \param w_: The width of the bounding rectangle.
          \param h_: The height of the bounding rectangle.
        */
        void setXYWH(int x_, int y_, int w_, int h_);
    };
}
