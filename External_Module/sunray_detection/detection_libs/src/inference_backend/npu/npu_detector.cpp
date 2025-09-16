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
#include "inference_backend/npu/npu_detector.h"
#if defined(NX)
#include "cuda_runner.h"
using RUNNERTYPE = sunray_detection::CudaRunner;
#elif defined(VIOBOT)
#include "rknn_runner.h"
using RUNNERTYPE = sunray_detection::RKNNRunner;
#endif
namespace sunray_detection
{
    NPUDetector::NPUDetector(const char *model_path, const int npu_id) :allocator_(new CPUAllocator())
    {
        runner_ = reinterpret_cast<void *>(new RUNNERTYPE(model_path, npu_id));
        inputs_shape_ = reinterpret_cast<RUNNERTYPE *>(runner_)->get_inputs_shape();
        outputs_shape_ = reinterpret_cast<RUNNERTYPE *>(runner_)->get_outputs_shape();
    }
    NPUDetector::~NPUDetector()
    {
        delete reinterpret_cast<RUNNERTYPE *>(runner_);
        delete allocator_;
    }

    void NPUDetector::detect(const cv::Mat &img, TargetsInFrame &tgts_)
    {
        std::vector<void *> out_data;
        std::vector<Target> targets;
        for (auto shape : outputs_shape_)
        {
            /* code */
            size_t size = 1;
            for (auto i : shape)
            {
                size *= i;
            }
            out_data.push_back(allocator_->malloc(size * sizeof(float)));
        }

        // 预处理
        int w1 = img.cols;
        int h1 = img.rows;
        int w2 = inputs_shape_[0][2];
        int h2 = inputs_shape_[0][1];

        float scale = std::min(static_cast<float>(w2) / w1, static_cast<float>(h2) / h1);

        // // 计算填充后的尺寸
        int new_width = static_cast<int>(w1 * scale);
        int new_height = static_cast<int>(h1 * scale);

        // 缩放图像到填充后的尺寸
        cv::Mat img_resized;
        cv::resize(img, img_resized, cv::Size(new_width, new_height));

        // 创建一个目标尺寸的黑色图像
        cv::Mat img_padded = cv::Mat::zeros(h2, w2, CV_8UC3);
        img_resized.copyTo(img_padded(cv::Rect(0, 0, new_width, new_height)));
        // 推理
        reinterpret_cast<RUNNERTYPE *>(runner_)->process(img_padded.data, out_data);

        // // 后处理
        post_process(out_data, &targets);
        // 放缩到原图大小
        for (auto &target : targets)
        {
            Target tgt;
            int x1 = (target.cx - target.w / 2) * w2 / scale;
            int y1 = (target.cy - target.h / 2) * h2 / scale;
            int x2 = (target.cx + target.w / 2) * w2 / scale;
            int y2 = (target.cy + target.h / 2) * h2 / scale;
            tgt.setBox(x1, y1, x2, y2, img.cols, img.rows);
            tgt.setCategory(target.category, target.category_id, target.score);
            tgts_.targets.push_back(tgt);
        }
        // // 释放内存
        for (auto data : out_data)
        {
            allocator_->free(data);
        }
    }
} // namespace name
