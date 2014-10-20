//============================================================================
// include/vsmc/utility/stop_watch.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_STOP_WATCH_HPP
#define VSMC_UTILITY_STOP_WATCH_HPP

#include <vsmc/internal/common.hpp>

/// \brief Use native timing library if `VSMC_HAS_CXX11LIB_CHRONO` test fails
/// \ingroup Config
#ifndef VSMC_HAS_NATIVE_TIME_LIBRARY
#define VSMC_HAS_NATIVE_TIME_LIBRARY 1
#endif

/// \brief The fallback StopWatch type
/// \ingroup Config
///
/// \details
/// The class defined by `VSMC_STOP_WATCH_TYPE` need to be defined before
/// including the `<vsmc/utility/stop_watch.hpp>` header. It shall provide the
/// same interface as StopWatchNull. This is only used when both
/// `VSMC_HAS_CXX11LIB_CHRONO` and `VSMC_HAS_NATIVE_TIME_LIBRARY` are zero, in
/// which case StopWatch is a typedef of this macro.
#ifndef VSMC_STOP_WATCH_TYPE
#define VSMC_STOP_WATCH_TYPE ::vsmc::StopWatchNull
#endif

/// \brief Default C++11 clock used as StopWatch if `VSMC_HAS_CXX11LIB_CHRONO`
/// test successes
/// \ingroup Config
#ifndef VSMC_STOP_WATCH_CHRONO_CLOCK_TYPE
#define VSMC_STOP_WATCH_CHRONO_CLOCK_TYPE std::chrono::high_resolution_clock
#endif

namespace vsmc {

/// \brief A StopWatch that provides the interface but does nothing and thus
/// cost no CPU cycles itself.
/// \ingroup StopWatch
class StopWatchNull
{
    public :

    bool running () const {return false;}
    bool start () {return false;}
    bool stop () {return false;}
    void reset () {}

    double nanoseconds  () const {return 1;}
    double microseconds () const {return 1e-3;}
    double milliseconds () const {return 1e-6;}
    double seconds      () const {return 1e-9;}
    double minutes      () const {return 1e-9 / 60;}
    double hours        () const {return 1e-9 / 3600;}
}; // class StopWatchNull

/// \brief Start and stop a StopWatch in scope (similiar to a mutex lock guard)
/// \ingroup StopWatch
template <typename WatchType>
class StopWatchGuard
{
    public :

    typedef WatchType watch_type;

    StopWatchGuard (watch_type &watch, bool start = true) :
        start_(start), watch_(watch) {if (start_) watch_.start();}

    ~StopWatchGuard () {if (start_) watch_.stop();}

    private :

    const bool start_;
    watch_type &watch_;
}; // class StopWatchGuard

} // namespace vsmc

#if VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_STOP_WATCH_DEFINED

#include <chrono>

namespace vsmc {

/// \brief StopWatch as an adapter of C++11 clock
/// \ingroup StopWatch
template <typename ClockType>
class StopWatchClockAdapter
{
    public :

    typedef ClockType clock_type;

    StopWatchClockAdapter () : elapsed_(0), running_(false) {reset();}

    /// \brief If the watch is running
    ///
    /// \details
    /// If `start()` has been called and no `stop()` call since, then it is
    /// running, otherwise it is stoped.
    bool running () const {return running_;}

    /// \brief Start the watch, no effect if already started
    ///
    /// \return `true` if it is started by this call, and the `elapsed` time
    /// will be incremented next time `stop()` is called. The increment will be
    /// relative to the time point of this call. `false` if it is already
    /// started earlier.
    bool start ()
    {
        if (running_)
            return false;

        running_ = true;
        start_time_ = clock_type::now();

        return true;
    }

    /// \brief Stop the watch, no effect if already stopped
    ///
    /// \return `true` if it is stoped by this call, and the `elapsed` time has
    /// been incremented. `false` if it is already stopped or wasn't started
    /// before.
    bool stop ()
    {
        if (!running_)
            return false;

        typename clock_type::time_point stop_time = clock_type::now();
        elapsed_ += stop_time - start_time_;
        running_ = false;

        return true;
    }

    /// \brief Stop and reset the `elapsed` time to zero
    void reset ()
    {
        start();
        elapsed_ = typename clock_type::duration(0);
        running_ = false;
    }

    /// \brief Return the accumulated elapsed time in nanoseconds
    double nanoseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::nano> >(elapsed_).count();
    }

    /// \brief Return the accumulated elapsed time in microseconds
    double microseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::micro> >(elapsed_).count();
    }

    /// \brief Return the accumulated elapsed time in milliseconds
    double milliseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::milli> >(elapsed_).count();
    }

    /// \brief Return the accumulated elapsed time in seconds
    double seconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<1> > >(elapsed_).count();
    }

    /// \brief Return the accumulated elapsed time in minutes
    double minutes () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<60> > >(elapsed_).count();
    }

    /// \brief Return the accumulated elapsed time in hours
    double hours () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<3600> > >(elapsed_).count();
    }

    private :

    typename clock_type::duration elapsed_;
    typename clock_type::time_point start_time_;
    bool running_;
}; // class StopWatchClockAdapter

/// \brief Stop watch
/// \ingroup StopWatch
typedef StopWatchClockAdapter<VSMC_STOP_WATCH_CHRONO_CLOCK_TYPE> StopWatch;

} // namespace vsmc

#elif VSMC_HAS_NATIVE_TIME_LIBRARY

#if defined(__APPLE__) || defined(__MACOSX)
#define VSMC_STOP_WATCH_DEFINED

#include <mach/mach_time.h>

namespace vsmc {

class StopWatch
{
    public :

    StopWatch () : running_(false) {reset();}

    bool running () {return running_;}

    bool start ()
    {
        if (running_)
            return false;

        running_ = true;
        start_time_ = ::mach_absolute_time();

        return true;
    }

    bool stop ()
    {
        if (!running_)
            return false;

        uint64_t stop_time = ::mach_absolute_time();
        uint64_t elapsed_abs = stop_time - start_time_;
        uint64_t elapsed_nsec = elapsed_abs *
            timebase_.numer / timebase_.denom;
        uint64_t inc_sec = elapsed_nsec / ratio_;
        uint64_t inc_nsec = elapsed_nsec % ratio_;
        elapsed_sec_ += inc_sec;
        elapsed_nsec_ += inc_nsec;
        running_ = false;

        return true;
    }

    void reset ()
    {
        start();
        elapsed_sec_ = 0;
        elapsed_nsec_ = 0;
        ::mach_timebase_info(&timebase_);
        running_ = false;
    }

    double nanoseconds () const
    {
        return static_cast<double>(elapsed_sec_) * 1e9 +
            static_cast<double>(elapsed_nsec_);
    }

    double microseconds () const
    {
        return static_cast<double>(elapsed_sec_) * 1e6 +
            static_cast<double>(elapsed_nsec_) * 1e-3;
    }

    double milliseconds () const
    {
        return static_cast<double>(elapsed_sec_) * 1e3 +
            static_cast<double>(elapsed_nsec_) * 1e-6;
    }

    double seconds () const
    {
        return static_cast<double>(elapsed_sec_) +
            static_cast<double>(elapsed_nsec_) * 1e-9;
    }

    double minutes () const
    {
        return static_cast<double>(elapsed_sec_) / 60.0 +
            static_cast<double>(elapsed_nsec_) * 1e-9 / 60.0;
    }

    double hours () const
    {
        return static_cast<double>(elapsed_sec_) / 3600.0 +
            static_cast<double>(elapsed_nsec_) * 1e-9 / 3600.0;
    }

    private :

    uint64_t elapsed_sec_;
    uint64_t elapsed_nsec_;
    uint64_t start_time_;
    ::mach_timebase_info_data_t timebase_;
    bool running_;
    static VSMC_CONSTEXPR const uint64_t ratio_ =
        static_cast<uint64_t>(1000000000ULL); // 9 zero
}; // class StopWatch

} // namespace vsmc

#elif defined(__linux__)

#include <time.h>
#include <features.h>

#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 199309L
#define VSMC_STOP_WATCH_DEFINED

namespace vsmc {

class StopWatch
{
    public :

    StopWatch () : running_(false) {reset();}

    bool running () {return running_;}

    bool start ()
    {
        if (running_)
            return false;

        running_ = true;
        ::clock_gettime(CLOCK_REALTIME, &start_time_);

        return true;
    }

    bool stop ()
    {
        if (!running_)
            return false;

        timespec stop_time;
        ::clock_gettime(CLOCK_REALTIME, &stop_time);
        time_t sec = stop_time.tv_sec - start_time_.tv_sec;
        long nsec = stop_time.tv_nsec - start_time_.tv_nsec;

        time_t inc_sec = sec + nsec / ratio_;
        long inc_nsec = nsec % ratio_;
        elapsed_.tv_sec += inc_sec;
        elapsed_.tv_nsec += inc_nsec;
        running_ = false;

        true;
    }

    void reset ()
    {
        start();
        elapsed_.tv_sec = 0;
        elapsed_.tv_nsec = 0;
        running_ = false;
    }

    double nanoseconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) * 1e9 +
            static_cast<double>(elapsed_.tv_nsec);
    }

    double microseconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) * 1e6 +
            static_cast<double>(elapsed_.tv_nsec) * 1e-3;
    }

    double milliseconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) * 1e3 +
            static_cast<double>(elapsed_.tv_nsec) * 1e-6;
    }

    double seconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) +
            static_cast<double>(elapsed_.tv_nsec) * 1e-9;
    }

    double minutes () const
    {
        return static_cast<double>(elapsed_.tv_sec) / 60.0 +
            static_cast<double>(elapsed_.tv_nsec) * 1e-9 / 60.0;
    }

    double hours () const
    {
        return static_cast<double>(elapsed_.tv_sec) / 3600.0 +
            static_cast<double>(elapsed_.tv_nsec) * 1e-9 / 3600.0;
    }

    private :

    ::timespec elapsed_;
    ::timespec start_time_;
    bool running_;
    static VSMC_CONSTEXPR const long ratio_ = 1000000000L; // 9 zero
}; // class StopWatch

} // namespace vsmc

#endif // _POSIX_C_SOURCE

#elif defined(WIN32)

#include <windows.h>
#define VSMC_STOP_WATCH_DEFINED

namespace vsmc {

class StopWatch
{
    public :

    StopWatch () :
        elapsed_(0), start_time_(0), frequency_(0), running_(false)
    {reset();}

    bool running () {return running_;}

    bool start ()
    {
        if (running_)
            return false;

        running_ = true;
        LARGE_INTEGER time;
        ::QueryPerformanceCounter(&time);
        start_time_ = time.QuadPart;

        return true;
    }

    bool stop ()
    {
        if (!running_)
            return false;

        LARGE_INTEGER time;
        ::QueryPerformanceCounter(&time);
        elapsed_ += time.QuadPart - start_time_;
        running_ = false;

        return true;
    }

    void reset ()
    {
        start();
        elapsed_ = 0;
        LARGE_INTEGER freq;
        ::QueryPerformanceFrequency(&freq);
        frequency_ = static_cast<double>(freq.QuadPart);
        running_ = false;
    }

    double nanoseconds () const
    {return static_cast<double>(elapsed_) / frequency_ * 1e9;}

    double microseconds () const
    {return static_cast<double>(elapsed_) / frequency_ * 1e6;}

    double milliseconds () const
    {return static_cast<double>(elapsed_) / frequency_ * 1e3;}

    double seconds () const
    {return static_cast<double>(elapsed_) / frequency_;}

    double minutes () const
    {return static_cast<double>(elapsed_) / frequency_ / 60.0;}

    double hours () const
    {return static_cast<double>(elapsed_) / frequency_ / 3600.0;}

    private :

    __int64 elapsed_;
    __int64 start_time_;
    double frequency_;
    bool running_;
}; // class StopWatch

} // namespace vsmc

#endif // VSMC_HAS_NATIVE_TIME_LIBRARY

#endif // VSMC_HAS_CXX11LIB_CHRONO

#ifndef VSMC_STOP_WATCH_DEFINED
namespace vsmc {
typedef VSMC_STOP_WATCH_TYPE StopWatch;
} // namespace vsmc
#else
#undef VSMC_STOP_WATCH_DEFINED
#endif

#endif // VSMC_UTILITY_STOP_WATCH_HPP
