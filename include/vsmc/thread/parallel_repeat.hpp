//============================================================================
// include/vsmc/thread/parallel_repeat.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_THREAD_PARALLEL_REPEAT_HPP
#define VSMC_THREAD_PARALLEL_REPEAT_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/thread/blocked_range.hpp>
#include <vsmc/thread/parallel_for.hpp>
#include <utility>

namespace vsmc {

/// \brief Parallel repeat using std::thread
/// \ingroup Thread
///
/// \details
/// Requirement: WorkType:
/// ~~~{.cpp}
/// WorkType work;
/// work(std::size_t);
/// ~~~
template <typename WorkType>
inline void parallel_repeat (std::size_t n, WorkType &&work)
{
    struct body
    {
        body (const WorkType &w) : work_(w) {}

        body (WorkType &&w) : work_(std::move(w)) {}

        void operator() (const BlockedRange<std::size_t> &range) const
        {
            for (std::size_t i = range.begin(); i != range.end(); ++i)
                work_(i);
        }

        private :

        WorkType work_;
    };
    parallel_for(BlockedRange<std::size_t>(0, n),
            body(std::forward<WorkType>(work)));
}

} // namesapce vsmc

#endif // VSMC_THREAD_PARALLEL_REPEAT_HPP
