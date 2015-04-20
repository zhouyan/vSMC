//============================================================================
// vSMC/include/vsmc/utility/rdtsc.hpp
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

#ifndef VSMC_UTILITY_RDTSC_HPP
#define VSMC_UTILITY_RDTSC_HPP

#include <vsmc/internal/common.hpp>

#ifdef VSMC_MSVC
#include <intrin.h>
#endif

namespace vsmc
{

/// \brief Return the TSC value using RDTSC instruction
/// \ingroup RDTSC
///
/// \note This function does not sync by itself. Call `cpuid` to sync if more
/// accurate measurement is needed.
inline uint64_t rdtsc()
{
#ifdef VSMC_MSVC
    return static_cast<uint64_t>(__rdtsc());
#else  // VSMC_MSVC
    unsigned eax = 0;
    unsigned edx = 0;
    __asm__ volatile("rdtsc\n" : "=a"(eax), "=d"(edx));

    return (static_cast<uint64_t>(edx) << 32) + static_cast<uint64_t>(eax);
#endif // VSMC_MSVC
}

/// \brief Return the TSC and TSC_AUX values using RDTSCP instruction
/// \ingroup RDTSC
inline uint64_t rdtscp(unsigned *aux)
{
#ifdef VSMC_MSVC
    return static_cast<uint64_t>(__rdtscp(aux));
#else  // VSMC_MSVC
    unsigned eax = 0;
    unsigned ecx = 0;
    unsigned edx = 0;
    __asm__ volatile("rdtscp\n" : "=a"(eax), "=c"(ecx), "=d"(edx));
    *aux = ecx;

    return static_cast<uint64_t>(eax) + (static_cast<uint64_t>(edx) << 32);
#endif // VSMC_MSVC
}

/// \brief CPU clock cycle counter using `rdtsc`
/// \ingroup RDTSC
class RDTSCCounter
{
    public:
    RDTSCCounter() : elapsed_(0), start_(0), running_(false) {}

    /// \brief If the counter is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running() const { return running_; }

    /// \brief Start the counter, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the elapsed cycle
    /// count will be incremented next time `stop()` is called. The increment
    /// will be relative to the time point of this call. `false` if it is
    /// already started earlier.
    bool start()
    {
        if (running_)
            return false;

        running_ = true;
        start_ = rdtsc();

        return true;
    }

    /// \brief Stop the counter, no effect if already stopped
    ///
    /// \return `true` if it is stoped by this call, and the elapsed cycle
    /// count has been incremented. `false` in one of the following
    /// situations.
    /// In all these situations, the elapsed cycle count will not be
    /// incremented.
    /// - It already stopped or wasn't started before.
    /// - The cycle count appears to decrease. This is most likely in the case
    /// that the TSC MSR has been reset.
    bool stop()
    {
        if (!running_)
            return false;

        uint64_t stop = rdtsc();
        if (stop < start_)
            return false;

        elapsed_ += stop - start_;
        running_ = false;

        return true;
    }

    /// \brief Stop and reset the elapsed cycle count to zero
    void reset()
    {
        elapsed_ = 0;
        running_ = false;
    }

    /// \brief Return the accumulated elapsed cycle count
    uint64_t cycles() const { return elapsed_; }

    private:
    uint64_t elapsed_;
    uint64_t start_;
    bool running_;
}; // class RDTSCCounter

/// \brief CPU clock cycle counter using `rdtscp`
/// \ingroup RDTSC
///
/// \details
/// This class shall only be used if RDTSCP is supported. For example,
/// ~~~{.cpp}
/// RDTSCCouner c1;
/// RDTSCPCouner c2;
/// CPUID::has_feature<CPUIDFeatureExtRDTSCP>() ? c2.start() : c1.start();
/// ~~~
class RDTSCPCounter
{
    public:
    RDTSCPCounter() : elapsed_(0), start_(0), start_id_(0), running_(false) {}

    /// \brief If the counter is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running() const { return running_; }

    /// \brief Start the counter, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the elapsed cycle
    /// count will be incremented next time `stop()` is called. The increment
    /// will be relative to the time point of this call. `false` if it is
    /// already started earlier.
    bool start()
    {
        if (running_)
            return false;

        running_ = true;
        start_ = rdtscp(&start_id_);

        return true;
    }

    /// \brief Stop the counter, no effect if already stopped
    ///
    /// \return `true` if it is stoped by this call, and the elapsed cycle
    /// count has been incremented. `false` in one of the following
    /// situations.
    /// In all these situations, the elapsed cycle count will not be
    /// incremented.
    /// - It already stopped or wasn't started before.
    /// - The logical processor has changed since the last successful
    /// `start()`
    /// call. The user is repsonsible for assure threads affinity.
    /// - The cycle count appears to decrease. This is most likely in the case
    /// that the TSC MSR has been reset.
    bool stop()
    {
        if (!running_)
            return false;

        unsigned stop_id = 0;
        uint64_t stop = rdtscp(&stop_id);
        if (stop_id != start_id_ || stop < start_)
            return false;

        elapsed_ += stop - start_;
        running_ = false;

        return true;
    }

    /// \brief Stop and reset the elapsed cycle count to zero
    void reset()
    {
        elapsed_ = 0;
        running_ = false;
    }

    /// \brief Return the accumulated elapsed cycle count
    uint64_t cycles() const { return elapsed_; }

    private:
    uint64_t elapsed_;
    uint64_t start_;
    unsigned start_id_;
    bool running_;
}; // class RDTSCPCounter

} // namespace vsmc

#endif // VSMC_UTILITY_RDTSC_HPP
