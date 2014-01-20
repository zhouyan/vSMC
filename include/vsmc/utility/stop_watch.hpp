#ifndef VSMC_UTILITY_STOP_WATCH_HPP
#define VSMC_UTILITY_STOP_WATCH_HPP

#include <vsmc/internal/common.hpp>

namespace vsmc {

/// \brief A dummy StopWatch that does nothing
/// \ingroup StopWatch
class DummyStopWatch
{
    public :

    bool running () const {return running_;}
    void start () const {running_ = true;}
    void stop  () const {running_ = false;}
    void reset () const {running_ = false;}

    double nanoseconds  () const {return 0;}
    double microseconds () const {return 0;}
    double milliseconds () const {return 0;}
    double seconds      () const {return 0;}
    double minutes      () const {return 0;}
    double hours        () const {return 0;}

    private :

    mutable bool running_;
}; // class StopWatch

/// \brief StopWatch as a wrapper of C++11 clock
/// \ingroup StopWatch
template <typename ClockType>
class StopWatchClockWrapper
{
    public :

    typedef ClockType clock_type;

    StopWatchClockWrapper () : elapsed_(0), running_(false) {}

    bool running () const {return running_;}

    void start () const
    {
        running_ = true;
        start_time_ = clock_type::now();
    }

    void stop () const
    {
        typename clock_type::time_point stop_time = clock_type::now();
        elapsed_ += stop_time - start_time_;
        running_ = false;
    }

    void reset () const
    {
        elapsed_ = typename clock_type::duration(0);
        running_ = false;
    }

    typename clock_type::duration elapsed () const {return elapsed_;}

    StopWatchClockWrapper<ClockType> &operator+= (
            const StopWatchClockWrapper<ClockType> &other)
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(stopped);

        elapsed_ += other.elapsed_;

        return *this;
    }

    StopWatchClockWrapper<ClockType> operator+ (
            const StopWatchClockWrapper<ClockType> &other) const
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(stopped);

        StopWatchClockWrapper<ClockType> watch(*this);
        watch += other;

        return watch;
    }


    private :

    mutable typename clock_type::duration elapsed_;
    mutable typename clock_type::time_point start_time_;
    mutable bool running_;
}; // class StopWatchClockWrapper

/// \brief Start and stop a StopWatch in scope
/// \ingroup StopWatch
template <typename WatchType>
class ScopedStopWatch
{
    public :

    typedef WatchType watch_type;

    ScopedStopWatch (watch_type &watch, bool start = true) :
        start_(start), watch_(watch) {if (start_) watch_.start();}

    ~ScopedStopWatch () {if (start_) watch_.stop();}

    private :

    const bool start_;
    watch_type &watch_;
}; // class ScopedStopWatch

} // namespace vsmc

#if VSMC_HAS_CXX11LIB_CHRONO
#define VSMC_STOP_WATCH_DEFINED

#include <chrono>

namespace vsmc {

/// \brief Stop watch
/// \ingroup StopWatch
class StopWatch :
    public StopWatchClockWrapper<std::chrono::high_resolution_clock>
{
    public :

    typedef StopWatchClockWrapper<std::chrono::high_resolution_clock>
        base_watch_type;

    StopWatch () {}

    StopWatch (const base_watch_type &base_watch) {}

    double nanoseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::nano> >(this->elapsed()).count();
    }

    double microseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::micro> >(this->elapsed()).count();
    }

    double milliseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::milli> >(this->elapsed()).count();
    }

    double seconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<1> > >(this->elapsed()).count();
    }

    double minutes () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<60> > >(this->elapsed()).count();
    }

    double hours () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<3600> > >(this->elapsed()).count();
    }
}; // class StopWatch

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

    bool running () const {return running_;}

    void start () const
    {
        running_ = true;
        start_time_ = mach_absolute_time();
    }

    void stop () const
    {
        uint64_t stop_time = mach_absolute_time();
        uint64_t elapsed_abs = stop_time - start_time_;
        uint64_t elapsed_nano = elapsed_abs *
            timebase_.numer / timebase_.denom;
        uint64_t sec = elapsed_nano / ratio_;
        elapsed_sec_ += sec;
        elapsed_nsec_ += elapsed_nano - sec * ratio_;
        running_ = false;
    }

    void reset () const
    {
        elapsed_sec_ = 0;
        elapsed_nsec_ = 0;
        mach_timebase_info(&timebase_);
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
            static_cast<double>(elapsed_nsec_) / 1e3;
    }

    double milliseconds () const
    {
        return static_cast<double>(elapsed_sec_) * 1e3 +
            static_cast<double>(elapsed_nsec_) / 1e6;
    }

    double seconds () const
    {
        return static_cast<double>(elapsed_sec_) +
            static_cast<double>(elapsed_nsec_) / 1e9;
    }

    double minutes () const
    {
        return static_cast<double>(elapsed_sec_) / 60.0 +
            static_cast<double>(elapsed_nsec_) / 1e9 / 60.0;
    }

    double hours () const
    {
        return static_cast<double>(elapsed_sec_) / 3600.0 +
            static_cast<double>(elapsed_nsec_) / 1e9 / 3600.0;
    }

    StopWatch &operator+= (const StopWatch &other)
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(stopped);

        elapsed_sec_  += other.elapsed_sec_;
        elapsed_nsec_ += other.elapsed_nsec_;

        return *this;
    }

    StopWatch operator+ (const StopWatch &other) const
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(stopped);

        StopWatch watch(*this);
        watch += other;

        return watch;
    }

    private :

    mutable uint64_t elapsed_sec_;
    mutable uint64_t elapsed_nsec_;
    mutable uint64_t start_time_;
    mutable mach_timebase_info_data_t timebase_;
    mutable bool running_;
    static const uint64_t ratio_ = 1000000000L; // 9 zero
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

    bool running () const {return running_;}

    void start () const
    {
        running_ = true;
        clock_gettime(CLOCK_REALTIME, &start_time_);
    }

    void stop () const
    {
        timespec stop_time;
        clock_gettime(CLOCK_REALTIME, &stop_time);
        time_t sec = stop_time.tv_sec - start_time_.tv_sec;
        long nsec = stop_time.tv_nsec - start_time_.tv_nsec;

        elapsed_.tv_sec += sec;
        elapsed_.tv_nsec += nsec;
        long inc_sec = elapsed_.tv_nsec / ratio_;
        elapsed_.tv_sec += static_cast<time_t>(inc_sec);
        elapsed_.tv_nsec -= inc_sec * ratio_;
        running_ = false;
    }

    void reset () const
    {
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
            static_cast<double>(elapsed_.tv_nsec) / 1e3;
    }

    double milliseconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) * 1e3 +
            static_cast<double>(elapsed_.tv_nsec) / 1e6;
    }

    double seconds () const
    {
        return static_cast<double>(elapsed_.tv_sec) +
            static_cast<double>(elapsed_.tv_nsec) / 1e9;
    }

    double minutes () const
    {
        return static_cast<double>(elapsed_.tv_sec) / 60.0 +
            static_cast<double>(elapsed_.tv_nsec) / 1e9 / 60.0;
    }

    double hours () const
    {
        return static_cast<double>(elapsed_.tv_sec) / 3600.0 +
            static_cast<double>(elapsed_.tv_nsec) / 1e9 / 3600.0;
    }

    StopWatch &operator+= (const StopWatch &other)
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(running);

        elapsed_.tv_sec  += other.elapsed_.tv_sec;
        elapsed_.tv_nsec += other.elapsed_.tv_nsec;

        return *this;
    }

    StopWatch operator+ (const StopWatch &other) const
    {
        bool stopped = !(running_ || other.running());
        VSMC_RUNTIME_ASSERT_ADDING_RUNNING_WATCH(stopped);

        StopWatch watch(*this);
        watch += other;

        return watch;
    }

    private :

    mutable timespec elapsed_;
    mutable timespec start_time_;
    mutable bool running_;
    static const long ratio_ = 1000000000L; // 9 zero
}; // class StopWatch

} // namespace vsmc

#endif // _POSIX_C_SOURCE

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
