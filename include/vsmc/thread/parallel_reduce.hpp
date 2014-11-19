//============================================================================
// vSMC/include/vsmc/thread/parallel_reduce.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013,2014, Yan Zhou
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

#ifndef VSMC_THREAD_PARALLEL_REDUCE_HPP
#define VSMC_THREAD_PARALLEL_REDUCE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/thread_guard.hpp>
#include <vsmc/thread/thread_num.hpp>
#include <thread>

namespace vsmc {

/// \brief Parallel reduce using C++11 concurrency
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range);
/// Work.join(other_work);
/// ~~~
template <typename Range, typename WorkType>
inline void parallel_reduce (const Range &range, WorkType &work)
{
    std::vector<Range> range_vec(ThreadNum::instance().partition(range));
    std::vector<WorkType> work_vec(range_vec.size(), work);
    {
        std::vector<ThreadGuard<std::thread>> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard<std::thread>(std::thread(
                            std::ref(work_vec[i]), range_vec[i])));
        }
    }
    for (std::size_t i = 0; i != work_vec.size(); ++i) work.join(work_vec[i]);
}

} // namespace vsmc

#endif // VSMC_THREAD_PARALLEL_REDUCE_HPP
