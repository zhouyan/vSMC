#ifndef VSMC_TIMER_LAPACK_TIMER_HPP
#define VSMC_TIMER_LAPACK_TIMER_HPP

extern "C" { double dsecnd(void); }

namespace vsmc {

/// \brief A timer use Lapack dsecnd
/// \ingroup Timer
///
/// \details
/// This timer use the \c dsecnd utility of Lapack.
class LapackTimer
{
    public :

    LapackTimer () : time_(0), last_(0), running_(false) {}

    void start () const
    {
        if (!running_) {
            running_ = true;
            last_ = ::dsecnd();
        }
    }

    void stop () const
    {
        if (running_) {
            time_ += ::dsecnd() - last_;
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
    mutable double last_;
    mutable bool running_;
}; // class LapackTimer

} // namespace vsmc

#endif // VSMC_TIMER_LAPACK_TIMER_HPP
