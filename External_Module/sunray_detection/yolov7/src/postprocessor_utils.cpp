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
#include "postprocessor_utils.h"

inline float intersection_area(const Object& a, const Object& b) {
  cv::Rect_<float> inter = a.rect & b.rect;
  return inter.area();
}

void nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>* picked, float nms_threshold) {
  picked->clear();
  const int n = objects.size();

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++) {
    areas[i] = objects[i].rect.area();
  }

  for (int i = 0; i < n; i++) {
    const Object& a = objects[i];

    int keep = 1;
    for (size_t  j = 0; j < picked->size(); j++) {
      const Object& b = objects[(*picked)[j]];

      // intersection over union
      float inter_area = intersection_area(a, b);
      float union_area = areas[i] + areas[(*picked)[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold) keep = 0;
    }

    if (keep) picked->push_back(i);
  }
}

