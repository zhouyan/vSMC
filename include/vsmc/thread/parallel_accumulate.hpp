//============================================================================
// include/vsmc/thread/parallel_accumulate.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
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
