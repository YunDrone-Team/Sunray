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
#include "rknn_runner.h"

#include <algorithm>
#include <chrono> // NOLINT
#include <iostream>
#include <cstring>
#include <string>
#include <thread> // NOLINT
#include <vector>
#include <string>

namespace sunray_detection
{
  inline float deqnt_affine_to_f32(int8_t qnt, int zp, float scale)
  {
    return (static_cast<float>(qnt) - static_cast<float>(zp)) * scale;
  }

  rknn_core_mask NPU_SERIAL_NUM[5] = {RKNN_NPU_CORE_0, RKNN_NPU_CORE_1, RKNN_NPU_CORE_2, RKNN_NPU_CORE_0_1,
                                      RKNN_NPU_CORE_0_1_2};

RKNNRunner::RKNNRunner(const char *model_path, const int npu_id)
      : cost_time_(0), sum_time_(0.0f), npu_performance_(0.0f)
  {
#ifdef DEBUG
    printf(__PRETTY_FUNCTION__);
#endif
    auto core_mask = NPU_SERIAL_NUM[npu_id];
    int ret = 0;
  FILE *fp = fopen(model_path, "rb");
  if (fp == NULL)
  {
    perror("fopen  fail!");
    throw std::runtime_error("fail fopen");
  }
  // 文件的长度(单位字节)
  fseek(fp, 0, SEEK_END);
  int model_len = ftell(fp);
  // 创建一个存储空间model且读入
  void *model = malloc(model_len);
  fseek(fp, 0, SEEK_SET);
  if (model_len != fread(model, 1, model_len, fp))
  {
    free(model);
    fclose(fp);
    perror("fread fail!");
    throw std::runtime_error("fail fopen");
  }
  fclose(fp);
  ret = rknn_init(&ctx_, model, model_len, 0, NULL);
    if (ret != RKNN_SUCC)
    {
      perror("rknn_init fail!");
      free(model);
      rknn_destroy(ctx_);
      throw std::runtime_error("rknn_init fail!");
    }
    ret = rknn_set_core_mask(ctx_, core_mask);
    if (ret != RKNN_SUCC)
    {
      perror("set NPU core_mask fail!");
      free(model);
      rknn_destroy(ctx_);
      throw std::runtime_error("fail set NPU core_mask");
    }
    // io_num
    rknn_input_output_num io_num;
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
      perror("rknn_query IO_NUM fail!");
      free(model);
      rknn_destroy(ctx_);
      throw std::runtime_error("rknn_query IO_NUM fail!");
    }
    input_num_ = io_num.n_input;
    output_num_ = io_num.n_output;

    // rknn inputs
    input_attrs_ = new rknn_tensor_attr[input_num_];
    memset(input_attrs_, 0, input_num_ * sizeof(rknn_tensor_attr));
    for (uint32_t i = 0; i < input_num_; i++)
    {
      input_attrs_[i].index = i;
      // query info
      ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs_[i]), sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC)
      {
        perror("rknn_query INPUT_ATTR fail!");
        free(model);
        rknn_destroy(ctx_);
        throw std::runtime_error("rknn_query INPUT_ATTR fail!");
      }
      this->dump_tensor_attr(&input_attrs_[i]);
    }

    // Create input tensor memory
    rknn_tensor_type input_type =
        RKNN_TENSOR_UINT8;                              // default input type is int8 (normalize and quantize need compute in outside)
    rknn_tensor_format input_layout = RKNN_TENSOR_NHWC; // default fmt is NHWC, npu only support NHWC in zero copy mode
    input_attrs_[0].type = input_type;
    input_attrs_[0].fmt = input_layout;
    input_mems_ = new rknn_tensor_mem *[input_num_];
    input_mems_[0] = rknn_create_mem(ctx_, input_attrs_[0].size_with_stride);

    // rknn outputs
    output_attrs_ = new rknn_tensor_attr[output_num_];
    memset(output_attrs_, 0, output_num_ * sizeof(rknn_tensor_attr));
    for (uint32_t i = 0; i < output_num_; i++)
    {
      output_attrs_[i].index = i;
      // query info
      ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs_[i]), sizeof(rknn_tensor_attr));
      if (ret != RKNN_SUCC)
      {
        perror("rknn_query OUTPUT_ATTR fail!");
        free(model);
        rknn_destroy(ctx_);
        throw std::runtime_error("rknn_query OUTPUT_ATTR fail!");
      }
      this->dump_tensor_attr(&output_attrs_[i]);
    }

    // Create output tensor memory
    output_mems_ = new rknn_tensor_mem *[output_num_];
    for (uint32_t i = 0; i < output_num_; ++i)
    {
      // default output type is depend on model, this require float32 to compute top5
      // allocate float32 output tensor
      int output_size = output_attrs_[i].n_elems * sizeof(int8_t);
      output_mems_[i] = rknn_create_mem(ctx_, output_size);
    }

    // Set input tensor memory
    ret = rknn_set_io_mem(ctx_, input_mems_[0], &input_attrs_[0]);
    if (ret != RKNN_SUCC)
    {
      perror("rknn_set_io_mem fail!");
      free(model);
      rknn_destroy(ctx_);
      throw std::runtime_error("rknn_set_io_mem fail!");
    }

    // Set output tensor memory
    for (uint32_t i = 0; i < output_num_; ++i)
    {
      // default output type is depend on model, this require float32 to compute top5
      output_attrs_[i].type = RKNN_TENSOR_INT8;
      // set output memory and attribute
      ret = rknn_set_io_mem(ctx_, output_mems_[i], &output_attrs_[i]);
      if (ret != RKNN_SUCC)
      {
        perror("rknn_set_io_mem fail!");
        free(model);
        rknn_destroy(ctx_);
        throw std::runtime_error("rknn_set_io_mem fail!");
      }
    }
  }

  float RKNNRunner::process(uint8_t *in_data, std::vector<void *> &out_data)
  {
    int ret;
    memcpy(input_mems_[0]->virt_addr, in_data, input_mems_[0]->size);
#if RKNN_VERSION_DEF >= 160
    rknn_mem_sync(ctx_, input_mems_[0], RKNN_MEMORY_SYNC_TO_DEVICE);
#endif

    ret = rknn_run(ctx_, NULL);
    if (ret != RKNN_SUCC)
    {
      perror("rknn_run fail!");
      return -1.0;
    }
    rknn_perf_run perf_run;
    ret = rknn_query(ctx_, RKNN_QUERY_PERF_RUN, &perf_run, sizeof(perf_run));
    cost_time_ = perf_run.run_duration;
    if (cost_time_ == -1)
    {
      perror("NPU inference Error");
    }
    for (int i = 0; i < output_num_; i++)
    {
#if RKNN_VERSION_DEF >= 160
      rknn_mem_sync(ctx_, output_mems_[i], RKNN_MEMORY_SYNC_FROM_DEVICE);
#endif
      float *buf = reinterpret_cast<float *>(out_data[i]);
      int zp = output_attrs_[i].zp;
      float scale = output_attrs_[i].scale;
      for (int j = 0; j < output_attrs_[i].n_elems; j++)
      {
        buf[j] = deqnt_affine_to_f32(reinterpret_cast<int8_t *>(output_mems_[i]->virt_addr)[j], zp, scale);
      }
    }

    npu_performance_ = this->cal_NPU_performance(&history_time_, &sum_time_, cost_time_ / 1.0e3);
#ifdef DEBUG
    printf("NPU performance : %f", npu_performance_);
#endif
    return npu_performance_;
  }

  std::vector<std::vector<size_t>> RKNNRunner::get_outputs_shape() const
  {
    std::vector<std::vector<size_t>> outputs_shapes;
    for (int i = 0; i < output_num_; i++)
    {
      std::vector<size_t> output_shape;
      for (int j = 0; j < output_attrs_[i].n_dims; j++)
      {
        output_shape.push_back(output_attrs_[i].dims[j]);
      }
      outputs_shapes.push_back(output_shape);
    }
    return outputs_shapes;
  }

  std::vector<std::vector<size_t>> RKNNRunner::get_inputs_shape() const
  {
    std::vector<std::vector<size_t>> inputs_shapes;
    for (int i = 0; i < input_num_; i++)
    {
      std::vector<size_t> input_shape;
      for (int j = 0; j < input_attrs_[i].n_dims; j++)
      {
        input_shape.push_back(input_attrs_[i].dims[j]);
      }
      inputs_shapes.push_back(input_shape);
    }
    return inputs_shapes;
  }

  void RKNNRunner::dump_tensor_attr(rknn_tensor_attr *attr)
  {
#ifdef DEBUG
    printf(
        "  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
        "zp=%d, scale=%f",
        attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3], attr->n_elems,
        attr->size, get_format_string(attr->fmt), get_type_string(attr->type), get_qnt_type_string(attr->qnt_type),
        attr->zp, attr->scale);
#endif
    return;
  }

  float RKNNRunner::cal_NPU_performance(std::queue<float> *history_time, float *sum_time, float cost_time)
  {
    if (history_time->size() < 10)
    {
      history_time->push(cost_time);
      *sum_time += cost_time;
    }
    else if (history_time->size() == 10)
    {
      *sum_time -= history_time->front();
      *sum_time += cost_time;
      history_time->pop();
      history_time->push(cost_time);
    }
    else
    {
      perror("cal_NPU_performance Error");
      return -1;
    }
    return (*sum_time) / static_cast<float>(history_time->size());
  }
  RKNNRunner::~RKNNRunner()
  {
#ifdef DEBUG
    printf(__PRETTY_FUNCTION__);
#endif
    for (size_t i = 0; i < input_num_; i++)
    {
      rknn_destroy_mem(ctx_, input_mems_[i]);
    }
    for (size_t i = 0; i < output_num_; i++)
    {
      rknn_destroy_mem(ctx_, output_mems_[i]);
    }
    delete input_attrs_;
    delete output_attrs_;
    rknn_destroy(ctx_);
  }
}