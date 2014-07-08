//============================================================================
// include/vsmc/thread/parallel_reduce.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_THREAD_PARALLEL_REDUCE_HPP
#define VSMC_THREAD_PARALLEL_REDUCE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/thread_guard.hpp>
#include <vsmc/thread/thread_num.hpp>
#include <thread>
#include <utility>
#include <vector>

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
