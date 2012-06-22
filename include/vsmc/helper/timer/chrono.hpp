#ifndef VSMC_HELPER_TIMER_CHRONO_HPP
#define VSMC_HELPER_TIMER_CHRONO_HPP

#include <vsmc/internal/config.hpp>

#if VSMC_HAS_CXX11LIB_CHRONO
#include <chrono>
namespace vsmc { namespace internal { namespace chrono {
using std::chrono::system_clock;
using std::chrono::duration;
} } }
#else
#include <boost/chrono.hpp>
namespace vsmc { namespace internal { namespace chrono {
using boost::chrono::system_clock;
using boost::chrono::duration;
} } }
#endif // VSMC_USE_

namespace vsmc {

class ChronoTimer
{
    public :

    ChronoTimer () : time_(0), running_(false) {}

    void start () const
    {
	if (!running_) {
	    running_ = true;
	    last_ = internal::chrono::system_clock::now();
	}
    }

    void stop () const
    {
	if (running_) {
	    time_ += internal::chrono::duration<double>(
		    internal::chrono::system_clock::now() - last_).count();
	    running_ = false;
	}
    }

    void reset () const
    {
	time_ = 0;
	running_ = false;
    }

    double duration () const
    {
	return time_;
    }

    private :

    mutable double time_;
    mutable internal::chrono::system_clock::time_point last_;
    mutable bool running_;
}; // class ChronoTimer

} // namespace vsmc

#endif // VSMC_HELPER_TIMER_CHRONO_HPP
