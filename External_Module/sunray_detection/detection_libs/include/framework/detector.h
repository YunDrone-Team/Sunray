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

#include <functional>

#include "opencv2/opencv.hpp"
#include "common/common.h"
#include "common/targets_in_frame.h"

namespace sunray_detection
{
    using CallbackType = std::function<void(const std::vector<void *> &input, std::vector<Target> *target)>;

    class BaseDetector : public NonCopyable
    {
    public:
        BaseDetector() {};
        virtual ~BaseDetector() {};
        virtual void detect(const cv::Mat &img, TargetsInFrame &tgts_) {};
        virtual void post_process(const std::vector<void *> &input, std::vector<Target> *target)
        {
            if (post_process_callback_)
            {
                post_process_callback_(input, target);
            }
            else
            {
                std::runtime_error("post_process is not implemented");
            }
        };

        void setPostProcessCallback(CallbackType cb)
        {
            post_process_callback_ = cb;
        }

    private:
        CallbackType post_process_callback_;
    };
}
