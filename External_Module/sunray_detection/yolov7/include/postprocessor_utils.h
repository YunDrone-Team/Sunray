/*************************************************************************************************************************
 * Copyright 2024 Grifcc
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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>
#include "opencv2/opencv.hpp"

struct Object {
  cv::Rect_<float> rect;
  int label;
  float prob;
};


inline float sigmoid(float x) { return 1.0 / (1.0 + expf(-x)); }

inline float unsigmoid(float y) { return -1.0 * logf((1.0 / y) - 1.0); }

inline float clip(float val, float min, float max) { return val <= min ? min : (val >= max ? max : val); }

inline bool compare(const Object x, const Object y) { return x.prob > y.prob; }

inline float intersection_area(const Object& a, const Object& b);

void nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>* picked, float nms_threshold);
