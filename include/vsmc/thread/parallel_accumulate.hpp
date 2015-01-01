//============================================================================
// vSMC/include/vsmc/thread/parallel_accumulate.hpp
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

#ifndef VSMC_THREAD_PARALLEL_ACCUMULATE_HPP
#define VSMC_THREAD_PARALLEL_ACCUMULATE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/thread_guard.hpp>
#include <vsmc/thread/thread_num.hpp>
#include <thread>

namespace vsmc {

/// \brief Parallel accumulate using C++11 concurrency
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range, res); // res: T reference type
/// ~~~
template <typename Range, typename T, typename WorkType>
inline T parallel_accumulate (const Range &range, WorkType &&work, T init)
{
    std::vector<Range> range_vec(ThreadNum::instance().partition(range));
    std::vector<T> result(range_vec.size());
    // start parallelization
    {
        std::vector<ThreadGuard<std::thread>> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard<std::thread>(std::thread(
                            std::forward<WorkType>(work),
                            range_vec[i], std::ref(result[i]))));
        }
    }
    // stop parallelization
    T acc(init);
    for (std::size_t i = 0; i != result.size(); ++i)
        acc += result[i];

    return acc;
}

/// \brief Parallel accumulate using std::thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType
/// ~~~{.cpp}
/// WorkType work;
/// work(range, res); // res: T reference type
/// ~~~
template <typename Range, typename T, typename Bin, typename WorkType>
inline T parallel_accumulate (const Range &range, WorkType &&work,
        T init, Bin bin_op)
{
    std::vector<Range> range_vec(ThreadNum::instance().partition(range));
    std::vector<T> result(range_vec.size());
    // start parallelization
    {
        std::vector<ThreadGuard<std::thread>> tg;
        tg.reserve(range_vec.size());
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard<std::thread>(std::thread(
                            std::forward<WorkType>(work),
                            range_vec[i], std::ref(result[i]))));
        }
    }
    // stop parallelization
    T acc(init);
    for (std::size_t i = 0; i != result.size(); ++i)
        acc = bin_op(acc, result[i]);

    return acc;
}

} // namespace vsmc

#endif // VSMC_THREAD_PARALLEL_ACCUMULATE_HPP
