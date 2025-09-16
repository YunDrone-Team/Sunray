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

#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "cuda_utils.h"
namespace sunray_detection
{
  class CudaRunner
  {
  public:
    explicit CudaRunner(const char *model,const int npu_id);
    float process(uint8_t *in_data, std::vector<void *> &out_data);
    std::vector<std::vector<size_t>> get_inputs_shape() const;
    std::vector<std::vector<size_t>> get_outputs_shape() const;
    ~CudaRunner();

  private:
    void warmup();
    int batchsize_;
    cudaStream_t stream_;                             // CUDA Stream
    IExecutionContext *context_;                      // profile 索引 -> 执行上下文
    cudaGraphExec_t graph_;                           // CUDA graph execution object
    std::vector<std::vector<size_t>> inputs_shapes_;  // Input shapes
    std::vector<std::vector<size_t>> outputs_shapes_; // Output shapes

    void *i_data_H_{nullptr}; // host memory for  uint8 data
    void *i_data_D_{nullptr}; // device memory for  uint8 data

    void *o_nums_D_{nullptr};    // device memory for  int32 nums
    void *o_bbox_D_{nullptr};    // device memory for  float32 bbox
    void *o_scores_D_{nullptr};  // device memory for  float32 scores
    void *o_classes_D_{nullptr}; // device memory for  int32 classes
  };
}