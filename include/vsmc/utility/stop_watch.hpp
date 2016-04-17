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

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

/// \brief Default C++11 clock used as StopWatch
/// \ingroup Config
#ifndef VSMC_STOP_WATCH_CLOCK_TYPE
#define VSMC_STOP_WATCH_CLOCK_TYPE std::chrono::high_resolution_clock
#endif

namespace vsmc
{

namespace internal
{

inline std::uint64_t rdtsc()
{
#if VSMC_HAS_X86

#if defined(VSMC_CLANG) || defined(VSMC_GCC) || defined(VSMC_INTEL)
    unsigned hi = 0;
    unsigned lo = 0;
#if VSMC_HAS_X86_64
    asm volatile("CPUID\n\t"
                 "RDTSC\n\t"
                 "mov %%edx, %0\n\t"
                 "mov %%eax, %1\n\t"
                 : "=r"(hi), "=r"(lo)::"%rax", "%rbx", "%rcx", "%rdx");
#else  // VSMC_HAS_X64_64
    asm volatile("CPUID\n\t"
                 "RDTSC\n\t"
                 "mov %%edx, %0\n\t"
                 "mov %%eax, %1\n\t"
                 : "=r"(lo), "=r"(lo)::"%eax", "%ebx", "%ecx", "%edx");
#endif // VSMC_HAS_X86_64
    return (static_cast<std::uint64_t>(hi) << 32) + lo;
#elif defined(VSMC_MSVC)
    return static_cast<std::uint64_t>(__rdtsc());
#else  // defined(VSMC_CLANG) || defined(VSMC_GCC) || defined(VSMC_INTEL)
    return 0;
#endif // defined(VSMC_CLANG) || defined(VSMC_GCC) || defined(VSMC_INTEL)

#else  // VSMC_HAS_X86
    return 0;
#endif // VSMC_HAS_X86
}

} // namespace vsmc::internal

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

    StopWatchClockAdapter()
        : time_(0), cycles_(0), cycles_start_(0), running_(false)
    {
        reset();
    }

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
        time_start_ = clock_type::now();
        cycles_start_ = internal::rdtsc();

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

        cycles_ += static_cast<double>(internal::rdtsc() - cycles_start_);
        typename clock_type::time_point time_stop = clock_type::now();
        time_ += time_stop - time_start_;
        running_ = false;

        return true;
    }

    /// \brief Stop and reset the elapsed time to zero
    void reset()
    {
        start();
        time_ = typename clock_type::duration(0);
        cycles_ = 0;
        running_ = false;
    }

    /// \brief Return the accumulated cycles
    double cycles() const { return cycles_; }

    /// \brief Return the accumulated elapsed time in nanoseconds
    double nanoseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::nano>>(time_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in microseconds
    double microseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::micro>>(time_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in milliseconds
    double milliseconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::milli>>(time_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in seconds
    double seconds() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<1>>>(time_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in minutes
    double minutes() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<60>>>(time_)
            .count();
    }

    /// \brief Return the accumulated elapsed time in hours
    double hours() const
    {
        return std::chrono::duration_cast<
                   std::chrono::duration<double, std::ratio<3600>>>(time_)
            .count();
    }

    private:
    typename clock_type::duration time_;
    typename clock_type::time_point time_start_;
    double cycles_;
    std::uint64_t cycles_start_;
    bool running_;
}; // class StopWatchClockAdapter

/// \brief Stop watch using `<chrono>`
/// \ingroup StopWatch
using StopWatch = StopWatchClockAdapter<VSMC_STOP_WATCH_CLOCK_TYPE>;

} // namespace vsmc

#endif // VSMC_UTILITY_STOP_WATCH_HPP
