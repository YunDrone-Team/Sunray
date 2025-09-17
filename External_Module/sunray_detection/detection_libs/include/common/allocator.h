/*************************************************************************************************************************
 * Copyright 2023 Grifcc
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
 *
 * Reference from https://github.com/Tencent/ncnn/blob/master/src/allocator.cpp
 *
 *************************************************************************************************************************/

#pragma once

#include <list>
#include <string>
#include <utility>

#include "common/common.h"

#define MALLOC_ALIGN 8

enum class ReallocType
{
    INCREASE,
    REUSE,
    DECREASE,
};

// thread-unsave
class Allocator : public NonCopyable
{
public:
    Allocator() {}
    virtual void *malloc(size_t size) = 0;
    virtual void free(void *ptr) = 0;
    virtual ~Allocator() {}
};

// thread-unsave
class CPUAllocator : public Allocator
{
private:
    unsigned int size_compare_ratio_; // 0~256
    size_t size_drop_threshold_;
    std::list<std::pair<size_t, void *>> budgets_;
    std::list<std::pair<size_t, void *>> payouts_;

public:
    CPUAllocator(const unsigned int size_compare_ratio, const size_t size_drop_threshold);
    CPUAllocator();
    ~CPUAllocator() override;
    void *malloc(size_t size) override;
    void free(void *ptr) override;
    void clear();
};
