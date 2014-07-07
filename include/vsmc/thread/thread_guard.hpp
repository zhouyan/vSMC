//============================================================================
// include/vsmc/thread/thread_guard.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_THREAD_THREAD_GUARD_HPP
#define VSMC_THREAD_THREAD_GUARD_HPP

#include <vsmc/internal/common.hpp>
#include <thread>
#include <utility>

namespace vsmc {

/// \brief Strictly scope-based std::thread ownership wrapper
/// \ingroup Thread
class ThreadGuard
{
    public :

    ThreadGuard () noexcept {}

    ThreadGuard (const ThreadGuard &) = delete;

    ThreadGuard &operator= (const ThreadGuard &) = delete;

    ThreadGuard (std::thread &&thr) noexcept :
        thread_(std::move(thr)) {}

    ThreadGuard (ThreadGuard &&other) noexcept :
        thread_(std::move(other.thread_)) {}

    ThreadGuard &operator= (ThreadGuard &&other) noexcept
    {thread_ = std::move(other.thread_); return *this;}

    ~ThreadGuard () noexcept {if (thread_.joinable()) thread_.join();}

    private :

    std::thread thread_;
}; // class ThreadGuard

} // namespace vsmc

#endif // VSMC_THREAD_THREAD_GUARD_HPP
