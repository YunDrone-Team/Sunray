#include "cuda_runner.h"
#include "cuda_utils.h"
#include <cuda_fp16.h> // 提供__half和相关转换函数
#include <cuda_runtime_api.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <vector>
#include <chrono>

#include "NvInfer.h"
#include <opencv2/opencv.hpp>

namespace sunray_detection
{
  CudaRunner::CudaRunner(const char *model, const int npu_id)
  {

#ifdef DEBUG
    static Logger gLogger(ILogger::Severity::kINFO);
#else
    static Logger gLogger(ILogger::Severity::kERROR);
#endif
  auto engine = InitEngine(model, gLogger);
    batchsize_ = engine->getProfileDimensions(0, 0, OptProfileSelector::kMAX).d[0];
    context_ = engine->createExecutionContext();
    context_->setOptimizationProfile(0);

    CHECK(cudaStreamCreate(&stream_));

    CHECK(cudaMallocHost(&i_data_H_, batchsize_ * 3 * 640 * 640 * sizeof(uint8_t), cudaHostAllocMapped));
    CHECK(cudaMalloc(&i_data_D_, batchsize_ * 3 * 640 * 640 * sizeof(uint8_t)));
    CHECK(cudaMalloc(&o_nums_D_, batchsize_ * sizeof(int)));
    CHECK(cudaMalloc(&o_bbox_D_, batchsize_ * 4 * 100 * sizeof(float)));
    CHECK(cudaMalloc(&o_scores_D_, batchsize_ * 100 * sizeof(float)));
    CHECK(cudaMalloc(&o_classes_D_, batchsize_ * 100 * sizeof(int)));

    auto shape = engine->getTensorShape("images");
    context_->setBindingDimensions(0, shape);
    context_->setTensorAddress("images", i_data_H_);
    inputs_shapes_.push_back(shapeToVec(engine->getTensorShape("images")));
    context_->setTensorAddress("nums", o_nums_D_);
    outputs_shapes_.push_back(shapeToVec(engine->getTensorShape("nums")));
    context_->setTensorAddress("bboxes", o_bbox_D_);
    outputs_shapes_.push_back(shapeToVec(engine->getTensorShape("bboxes")));
    context_->setTensorAddress("scores", o_scores_D_);
    outputs_shapes_.push_back(shapeToVec(engine->getTensorShape("scores")));
    context_->setTensorAddress("classes", o_classes_D_);
    outputs_shapes_.push_back(shapeToVec(engine->getTensorShape("classes")));

    warmup();
  }

  void CudaRunner::warmup()
  {
    // 遍历每个 batch size 的配置

    cudaGraph_t graph;

    // 捕获 CUDA 图
    cudaStreamBeginCapture(stream_, cudaStreamCaptureModeGlobal);

    // dump in_data_ 数据
    cudaMemcpyAsync(i_data_D_, i_data_H_, batchsize_ * 3 * 640 * 640 * sizeof(uint8_t), cudaMemcpyHostToDevice, stream_);
    context_->enqueueV3(stream_);

    // 结束 CUDA 图捕获
    cudaStreamEndCapture(stream_, &graph);

    // 实例化 CUDA 图
    cudaGraphNode_t errorNode = nullptr;
    char logBuffer[1024];
    CHECK(cudaGraphInstantiate(&graph_, graph, &errorNode, logBuffer, sizeof(logBuffer)));

    // 执行 CUDA 图多次以进行 warmup
    for (int j = 0; j < 100; ++j)
    {
      CHECK(cudaGraphLaunch(graph_, stream_));
    }

    // 确保 GPU 任务完成
    CHECK(cudaStreamSynchronize(stream_));
  }

  float CudaRunner::process(uint8_t *in_data, std::vector<void *> &out_data)
  {
    if (in_data == nullptr || out_data.empty())
    {
      printf("Invalid input or output buffers.");
      return -1.0f;
    }

    // 拷贝输入数据到Host内存
    memcpy(i_data_H_, in_data, batchsize_ * 3 * 640 * 640 * sizeof(uint8_t));

    // 异步将数据传输到Device内存
    cudaMemcpyAsync(i_data_D_, i_data_H_, batchsize_ * 3 * 640 * 640 * sizeof(uint8_t), cudaMemcpyHostToDevice, stream_);

    // 执行TensorRT推理
    context_->enqueueV3(stream_);

    // 同步Stream
    CHECK(cudaStreamSynchronize(stream_));

    // 复制推理结果到输出缓冲区
    // 假设out_data按照输出顺序依次是：nums, bboxes, scores, classes
    int32_t *nums_output = static_cast<int32_t *>(out_data[0]);    // INT32, shape: (1, 1)
    float *bboxes_output = static_cast<float *>(out_data[1]);      // FP32, shape: (1, 50, 4)
    float *scores_output = static_cast<float *>(out_data[2]);      // FP32, shape: (1, 50)
    int32_t *classes_output = static_cast<int32_t *>(out_data[3]); // INT32, shape: (1, 50)

    // 将Device内存的结果拷贝回Host内存
    cudaMemcpyAsync(nums_output, o_nums_D_, sizeof(int32_t), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(bboxes_output, o_bbox_D_, batchsize_ * 50 * 4 * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(scores_output, o_scores_D_, batchsize_ * 50 * sizeof(float), cudaMemcpyDeviceToHost, stream_);
    cudaMemcpyAsync(classes_output, o_classes_D_, batchsize_ * 50 * sizeof(int32_t), cudaMemcpyDeviceToHost, stream_);

    // 再次同步Stream确保数据传输完成
    CHECK(cudaStreamSynchronize(stream_));
    // 可以返回一个性能指标，如推理时间等
    return 0.0f;
  }

  std::vector<std::vector<size_t>> CudaRunner::get_outputs_shape() const
  {
    return outputs_shapes_;
  }

  std::vector<std::vector<size_t>> CudaRunner::get_inputs_shape() const
  {
    return inputs_shapes_;
  }

  CudaRunner::~CudaRunner()
  {

    cudaFreeHost(i_data_H_);
    cudaFree(o_nums_D_);
    cudaFree(o_bbox_D_);
    cudaFree(o_scores_D_);
    cudaFree(o_classes_D_);

    delete context_;

    cudaGraphExecDestroy(graph_);
    cudaStreamDestroy(stream_);
  }
}