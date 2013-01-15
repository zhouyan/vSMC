#ifndef VSMC_UTILITY_STOP_WATCH_HPP
#define VSMC_UTILITY_STOP_WATCH_HPP

#include <vsmc/internal/config.hpp>
#include <vsmc/internal/assert.hpp>
#include <vsmc/internal/defines.hpp>
#include <vsmc/internal/forward.hpp>

#if VSMC_HAS_CXX11LIB_CHRONO

#include <chrono>

namespace vsmc {

class StopWatch
{
    public :

    typedef std::chrono::high_resolution_clock clock_type;

    StopWatch () : elapsed_(0) {}

    void start () const
    {
        start_time_ = clock_type::now();
    }

    void stop () const
    {
        clock_type::time_point stop_time = clock_type::now();
        elapsed_ += stop_time - start_time_;
    }

    void reset () const
    {
        elapsed_ = clock_type::duration(0);
    }

    double nanoseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::nano> >(elapsed_).count();
    }

    double microseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::micro> >(elapsed_).count();
    }

    double milliseconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::milli> >(elapsed_).count();
    }

    double seconds () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<1> > >(elapsed_).count();
    }

    double minutes () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<60> > >(elapsed_).count();
    }

    double hours () const
    {
        return std::chrono::duration_cast<std::chrono::duration<
            double, std::ratio<3600> > >(elapsed_).count();
    }

    private :

    mutable clock_type::duration elapsed_;
    mutable clock_type::time_point start_time_;
}; // class StopWatch

} // namespace vsmc

#elif defined(__APPLE__) || defined(__MACOSX)

namespace vsmc {

class StopWatch
{
    public :

    void start () const {}
    void stop () const {}
    void reset () const {}

    double nanoseconds () const { return 0; }
    double microseconds () const { return 0; }
    double milliseconds () const { return 0; }
    double seconds () const { return 0; }
    double minutes () const { return 0; }
    double hours () const { return 0; }
}; // class StopWatch

} // namespace vsmc

#elif defined(__linux__)

namespace vsmc {

class StopWatch
{
    public :

    void start () const {}
    void stop () const {}
    void reset () const {}

    double nanoseconds () const { return 0; }
    double microseconds () const { return 0; }
    double milliseconds () const { return 0; }
    double seconds () const { return 0; }
    double minutes () const { return 0; }
    double hours () const { return 0; }
}; // class StopWatch

} // namespace vsmc

#elif defined(__posix__)

namespace vsmc {

class StopWatch
{
    public :

    void start () const {}
    void stop () const {}
    void reset () const {}

    double nanoseconds () const { return 0; }
    double microseconds () const { return 0; }
    double milliseconds () const { return 0; }
    double seconds () const { return 0; }
    double minutes () const { return 0; }
    double hours () const { return 0; }
}; // class StopWatch

} // namespace vsmc

#else // VSMC_HAS_CXX11LIB_CHRONO

namespace vsmc {

class StopWatch
{
    public :

    void start () const {}
    void stop () const {}
    void reset () const {}

    double nanoseconds () const { return 0; }
    double microseconds () const { return 0; }
    double milliseconds () const { return 0; }
    double seconds () const { return 0; }
    double minutes () const { return 0; }
    double hours () const { return 0; }
}; // class StopWatch

} // namespace vsmc

#endif // VSMC_HAS_CXX11LIB_CHRONO

#endif // VSMC_UTILITY_STOP_WATCH_HPP
