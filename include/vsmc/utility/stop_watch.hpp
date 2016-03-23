//============================================================================
// vSMC/include/vsmc/utility/stop_watch.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2016, Yan Zhou
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

#ifndef VSMC_UTILITY_STOP_WATCH_HPP
#define VSMC_UTILITY_STOP_WATCH_HPP

#include <vsmc/internal/common.hpp>

/// \brief Default C++11 clock used as StopWatch
/// \ingroup Config
#ifndef VSMC_STOP_WATCH_CLOCK_TYPE
#define VSMC_STOP_WATCH_CLOCK_TYPE std::chrono::high_resolution_clock
#endif

namespace vsmc
{

/// \brief Start and stop a StopWatch in scope (similiar to a mutex lock
/// guard)
/// \ingroup StopWatch
template <typename WatchType>
class StopWatchGuard
{
    public:
    using watch_type = WatchType;

    StopWatchGuard(watch_type &watch, bool start = true)
        : start_(start), watch_(watch)
    {
        if (start_)
            watch_.start();
    }

    ~StopWatchGuard()
    {
        if (start_)
            watch_.stop();
    }

    private:
    const bool start_;
    watch_type &watch_;
}; // class StopWatchGuard

/// \brief StopWatch as an adapter of C++11 clock
/// \ingroup StopWatch
template <typename ClockType>
class StopWatchClockAdapter
{
    public:
    using clock_type = ClockType;

    StopWatchClockAdapter() : elapsed_(0), running_(false) { reset(); }

    /// \brief If the watch is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running() const { return running_; }

    /// \brief Start the watch, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the elapsed time will
    /// be incremented next time `stop()` is called. The increment will be
    /// relative to the time point of this call. `false` if it is already
    /// started earlier.
    bool start()
    {
        if (running_)
            return false;

        running_ = true;
        start_time_ = clock_type::now();

        return true;
    }

    /// \brief Stop the watch, no effect if already stopped
    ///
    /// \return `true` if it is stoped by this call, and the elapsed time has
    /// been incremented. `false` if it is already stopped or wasn't started
    /// before.
    bool stop()
    {
        if (!running_)
            return false;

        typename clock_type::time_point stop_time = clock_type::now();
        elapsed_ += stop_time - start_time_;
        running_ = false;

        return true;
    }

    /// \brief Stop and reset the elapsed time to zero
    void reset()
    {
        start();
        elapsed_ = typename clock_type::duration(0);
        running_ = false;
    }

    /// \brief Return the accumulated elapsed time in nanoseconds
    double nanoseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::nano>>(elapsed_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in microseconds
    double microseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::micro>>(elapsed_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in milliseconds
    double milliseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::milli>>(elapsed_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in seconds
    double seconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<1>>>(elapsed_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in minutes
    double minutes() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<60>>>(elapsed_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in hours
    double hours() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<3600>>>(elapsed_)
            .count();
    }

    private:
    typename clock_type::duration elapsed_;
    typename clock_type::time_point start_time_;
    bool running_;
}; // class StopWatchClockAdapter

/// \brief Stop watch using `<chrono>`
/// \ingroup StopWatch
using StopWatch = StopWatchClockAdapter<VSMC_STOP_WATCH_CLOCK_TYPE>;

} // namespace vsmc

#endif // VSMC_UTILITY_STOP_WATCH_HPP
