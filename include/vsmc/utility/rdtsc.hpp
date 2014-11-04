//============================================================================
// include/vsmc/utility/rdtsc.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_RDTSC_HPP
#define VSMC_UTILITY_RDTSC_HPP

#include <vsmc/internal/common.hpp>

#ifdef _MSC_VER
#include <intrin.h>
#endif

namespace vsmc {

/// \brief Return the TSC value using RDTSC instruction after synchronization
/// with CPUID instruction
/// \ingroup RDTSC
inline uint64_t rdtsc ()
{
#ifdef _MSC_VER
    int aux[4];
    __cpuidex(aux, 0, 0);

    return static_cast<uint64_t>(__rdtsc());
#else // _MSC_VER
    unsigned eax = 0;
    unsigned ecx = 0;
    unsigned edx = 0;
    __asm__ volatile
        (
         "cpuid\n\t"
         "rdtsc\n"
         : "=a" (eax), "=d" (edx)
         :  "a" (eax),  "c" (ecx)
        );

    return (static_cast<uint64_t>(edx) << 32) + static_cast<uint64_t>(eax);
#endif // _MSC_VER
}

/// \brief Return the TSC and TSC_AUX values using RDTSCP instruction
/// \ingroup RDTSC
inline uint64_t rdtscp (unsigned *aux)
{
#ifdef _MSC_VER
    return static_cast<uint64_t>(__rdtscp(aux));
#else // _MSC_VER
    unsigned eax = 0;
    unsigned edx = 0;
    unsigned ecx = 0;
    __asm__ volatile
        (
         "rdtscp\n"
         : "=a" (eax), "=c" (ecx), "=d" (edx)
        );
    *aux = ecx;

    return static_cast<uint64_t>(eax) + (static_cast<uint64_t>(edx) << 32);
#endif // _MSC_VER
}

/// \brief CPU clock cycle counter using RDTSC
/// \ingroup RDTSC
class RDTSCCounter
{
    public :

    RDTSCCounter () : elapsed_(0), start_(0), running_(false) {}

    /// \brief If the counter is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running () const {return running_;}

    /// \brief Start the counter, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the elapsed cycle
    /// count will be incremented next time `stop()` is called. The increment
    /// will be relative to the time point of this call. `false` if it is
    /// already started earlier.
    bool start ()
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
    /// count has been incremented. `false` in one of the following situations.
    /// In all these situations, the elapsed cycle count will not be
    /// incremented.
    /// - It already stopped or wasn't started before.
    /// - The cycle count appears to decrease. This is most likely in the case
    /// that the TSC MSR has been reset.
    bool stop ()
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
    void reset ()
    {
        elapsed_ = 0;
        running_ = false;
    }

    /// \brief Return the accumulated elapsed cycle count
    uint64_t cycles () const {return elapsed_;}

    private :

    uint64_t elapsed_;
    uint64_t start_;
    bool running_;
}; // class RDTSCCounter

/// \brief CPU clock cycle counter using RDTSCP
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
    public :

    RDTSCPCounter () : elapsed_(0), start_(0), start_id_(0), running_(false) {}

    /// \brief If the counter is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running () const {return running_;}

    /// \brief Start the counter, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the elapsed cycle
    /// count will be incremented next time `stop()` is called. The increment
    /// will be relative to the time point of this call. `false` if it is
    /// already started earlier.
    bool start ()
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
    /// count has been incremented. `false` in one of the following situations.
    /// In all these situations, the elapsed cycle count will not be
    /// incremented.
    /// - It already stopped or wasn't started before.
    /// - The logical processor has changed since the last successful `start()`
    /// call. The user is repsonsible for assure threads affinity.
    /// - The cycle count appears to decrease. This is most likely in the case
    /// that the TSC MSR has been reset.
    bool stop ()
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
    void reset ()
    {
        elapsed_ = 0;
        running_ = false;
    }

    /// \brief Return the accumulated elapsed cycle count
    uint64_t cycles () const {return elapsed_;}

    private :

    uint64_t elapsed_;
    uint64_t start_;
    unsigned start_id_;
    bool running_;
}; // class RDTSCPCounter

} // namespace vsmc

#endif // VSMC_UTILITY_RDTSC_HPP
