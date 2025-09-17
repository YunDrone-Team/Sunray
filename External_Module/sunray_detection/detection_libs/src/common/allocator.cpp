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
#include <list>
#include <string>
#include <utility>

#include "common/common.h"
#include "common/allocator.h"

// Aligns a pointer to the specified number of bytes
// ptr Aligned pointer
// n Alignment size that must be a power of two
template <typename _Tp>
static inline _Tp *alignPtr(_Tp *ptr, int n = (int)sizeof(_Tp))
{                                               // NOLINT
    return (_Tp *)(((size_t)ptr + n - 1) & -n); // NOLINT
}

static inline void *fastMalloc(size_t size)
{
    unsigned char *udata = (unsigned char *)malloc(size + sizeof(void *) + MALLOC_ALIGN);
    if (!udata)
        return 0;
    unsigned char **adata = alignPtr((unsigned char **)udata + 1, MALLOC_ALIGN);
    adata[-1] = udata;
    return adata;
}

static inline void fastFree(void *ptr)
{
    if (ptr)
    {
        unsigned char *udata = ((unsigned char **)ptr)[-1];
        free(udata);
    }
}

CPUAllocator::CPUAllocator(const unsigned int size_compare_ratio, const size_t size_drop_threshold)
    : size_compare_ratio_(size_compare_ratio), size_drop_threshold_(size_drop_threshold) {}
CPUAllocator::CPUAllocator()
    : size_compare_ratio_(128), size_drop_threshold_(100) {}
CPUAllocator::~CPUAllocator()
{
    clear();
    if (!this->payouts_.empty())
    {
        std::list<std::pair<size_t, void *>>::iterator it = this->payouts_.begin();
        for (; it != this->payouts_.end(); ++it)
        {
            void *ptr = it->second;
            printf("%p still in use", ptr);
        }
    }
    std::runtime_error("FATAL ERROR! pool allocator destroyed too early");
}
void *CPUAllocator::malloc(size_t size)
{
    // find free budget
    std::list<std::pair<size_t, void *>>::iterator it = this->budgets_.begin(), it_max = this->budgets_.begin(),
                                                   it_min = this->budgets_.begin();
    for (; it != this->budgets_.end(); ++it)
    {
        size_t bs = it->first;

        // size_compare_ratio ~ 100%
        if (bs >= size && ((bs * this->size_compare_ratio_) >> 8) <= size)
        {
            void *ptr = it->second;

            this->budgets_.erase(it);

            this->payouts_.push_back(std::make_pair(bs, ptr));
#ifdef DEBUG
            printf("Reuse %p, size is %d", ptr, bs);
#endif
            return ptr;
        }

        if (bs < it_min->first)
        {
            it_min = it;
        }
        if (bs > it_max->first)
        {
            it_max = it;
        }
    }

    if (this->budgets_.size() >= this->size_drop_threshold_)
    {
        // All chunks in pool are not chosen. Then try to drop some outdated
        // chunks and return them to OS.
        if (it_max->first < size)
        {
            // Current query is asking for a chunk larger than any cached chunks.
            // Then remove the smallest one.
            fastFree(it_min->second);
            this->budgets_.erase(it_min);
        }
        else if (it_min->first > size)
        {
            // Current query is asking for a chunk smaller than any cached chunks.
            // Then remove the largest one.
            fastFree(it_max->second);
            this->budgets_.erase(it_max);
        }
    }

    // new
    void *ptr = fastMalloc(size);
    this->payouts_.push_back(std::make_pair(size, ptr));
    return ptr;
}

void CPUAllocator::free(void *ptr)
{
    // return to budgets
    std::list<std::pair<size_t, void *>>::iterator it = this->payouts_.begin();
    for (; it != this->payouts_.end(); ++it)
    {
        if (it->second == ptr)
        {
            size_t size = it->first;

            this->payouts_.erase(it);

            this->budgets_.push_back(std::make_pair(size, ptr));

            return;
        }
    }
    fastFree(ptr);
    std::runtime_error("FATAL ERROR! pool allocator get wild" + std::to_string(reinterpret_cast<uintptr_t>(ptr)));
}

void CPUAllocator::clear()
{
    std::list<std::pair<size_t, void *>>::iterator it = this->budgets_.begin();
    for (; it != this->budgets_.end(); ++it)
    {
        void *ptr = it->second;
        fastFree(ptr);
    }
    this->budgets_.clear();
}
