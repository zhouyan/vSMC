//============================================================================
// include/vsmc/thread/parallel_for.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_THREAD_PARALLE_FOR_HPP
#define VSMC_THREAD_PARALLE_FOR_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/thread_guard.hpp>
#include <vsmc/thread/thread_num.hpp>
#include <thread>
#include <utility>
#include <vector>

namespace vsmc {

/// \brief Parallel for using std::thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// ~~~{.cpp}
/// WorkType work;
/// work(range);
/// ~~~
template <typename Range, typename WorkType>
inline void parallel_for (const Range &range, WorkType &&work)
{
    std::vector<Range> range_vec(ThreadNum::instance().partition(range));
    std::vector<ThreadGuard> tg;
    tg.reserve(range_vec.size());
    {
        for (std::size_t i = 0; i != range_vec.size(); ++i) {
            tg.push_back(ThreadGuard(std::thread(std::forward<WorkType>(work),
                            range_vec[i])));
        }
    }
}

} // namespace vsmc

#endif // VSMC_THREAD_PARALLE_FOR_HPP
