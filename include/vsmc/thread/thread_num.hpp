//============================================================================
// vSMC/include/vsmc/thread/thread_num.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_THREAD_THREAD_NUM_HPP
#define VSMC_THREAD_THREAD_NUM_HPP

#include <vsmc/internal/common.hpp>
#include <thread>

namespace vsmc {

/// \brief Number of threads used by algorithms
/// \ingroup Thread
class ThreadNum
{
    public :

    static ThreadNum &instance ()
    {
        static ThreadNum num;

        return num;
    }

    std::size_t thread_num () const {return thread_num_;}

    /// \brief Set a new number of threads, return the old number
    std::size_t thread_num (std::size_t num)
    {
        std::size_t old_num = thread_num_;
        thread_num_ = num;

        return old_num;
    }

    template <typename Range>
    std::vector<Range> partition (const Range &range) const
    {
        typedef typename Range::const_iterator size_type;
        size_type N = range.end() - range.begin();
        size_type tn = static_cast<size_type>(thread_num());
        size_type block_size =  0;

        if (N < tn)
            block_size = 1;
        else if (N % tn)
            block_size = N / tn + 1;
        else
            block_size = N / tn;

        std::vector<Range> range_vec;
        range_vec.reserve(thread_num());
        size_type B = range.begin();
        while (N > 0) {
            size_type next = N < block_size ? N : block_size;
            range_vec.push_back(Range(B, B + next));
            B += next;
            N -= next;
        }

        return range_vec;
    }

    private :

    std::size_t thread_num_;

    ThreadNum () : thread_num_(
            static_cast<std::size_t>(1) >
            static_cast<std::size_t>(std::thread::hardware_concurrency()) ?
            static_cast<std::size_t>(1) :
            static_cast<std::size_t>(std::thread::hardware_concurrency()))
    {
#ifdef VSMC_MSVC
#pragma warning(push)
#pragma warning(disable:4996)
#endif
        const char *num_str = std::getenv("VSMC_THREAD_NUM");
#ifdef VSMC_MSVC
#pragma warning(pop)
#endif
        if (num_str) {
            int num = std::atoi(num_str);
            thread_num_ = num > 0 ? static_cast<std::size_t>(num) : 1;
        }
    }

    ThreadNum (const ThreadNum &) = delete;
    ThreadNum &operator= (const ThreadNum &) = delete;
}; // class ThreadInfo

} // namespace vsmc

#endif // VSMC_THREAD_THREAD_NUM_HPP
