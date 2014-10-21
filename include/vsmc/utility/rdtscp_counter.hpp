//============================================================================
// include/vsmc/utility/rdtscp_counter.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_RDTSCP_COUNTER_HPP
#define VSMC_UTILITY_RDTSCP_COUNTER_HPP

#include <vsmc/internal/common.hpp>
#include <stdint.h>

#ifdef _MSC_VER
#include <intrin.h>
#endif

namespace vsmc {

/// \brief CPU clock cycle counter using RDTSCP
/// \ingroup RDTSCP
class RDTSCPCounter
{
    public :

    RDTSCPCounter () : elapsed_(0), start_(0), start_id_(0), running_(false)
    {reset();}

    /// \brief If the counter is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running () const {return false;}

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
        start_ = now(&start_id_);

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
        uint64_t stop = now(&stop_id);
        bool same_id = stop_id == start_id_;
        if (same_id)
            elapsed_ += stop - start_;
        running_ = false;

        return same_id;
    }

    /// \brief Stop and reset the elapsed cycle count to zero
    void reset ()
    {
        start();
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

    uint64_t now (unsigned *aux)
    {
#ifdef _MSC_VER
        return static_cast<uint64_t>(__rdtscp(&ecx));
#else // _MSC_VER
        unsigned eax = 0x00;
        unsigned edx = 0x00;
        unsigned ecx = 0x00;
        __asm__ volatile(
                "rdtscp\n"
                : "=a" (eax), "=d" (edx), "=c" (ecx)
               );
        *aux = ecx;

        return (static_cast<uint64_t>(edx) << 32) + static_cast<uint64_t>(eax);
#endif // _MSC_VER
    }
}; // class RDTSCPCounter

} // namespace vsmc

#endif // VSMC_UTILITY_RDTSCP_COUNTER_HPP
