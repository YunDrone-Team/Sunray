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

#include <queue>
#include <vector>
#include <memory>

#include "rknn_api.h"
namespace sunray_detection
{
  class RKNNRunner
  {
  public:
  explicit RKNNRunner(const char* model_path, const int npu_id);

    float process(uint8_t *in_data, std::vector<void *> &out_data);

    std::vector<std::vector<size_t>> get_inputs_shape() const;
    std::vector<std::vector<size_t>> get_outputs_shape() const;
    ~RKNNRunner();

  private:
    int cost_time_;
    int input_num_;
    int output_num_;
    float sum_time_;
    float npu_performance_;
    std::queue<float> history_time_;
    rknn_context ctx_;
    rknn_tensor_attr *input_attrs_;
    rknn_tensor_attr *output_attrs_;
    rknn_tensor_mem **input_mems_;
    rknn_tensor_mem **output_mems_;

    float cal_NPU_performance(std::queue<float> *history_time, float *sum_time, float cost_time);
    void dump_tensor_attr(rknn_tensor_attr *attr);
  };
}