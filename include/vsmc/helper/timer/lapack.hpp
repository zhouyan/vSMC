#ifndef VSMC_INTERNAL_TIMER_LAPACK_HPP
#define VSMC_INTERNAL_TIMER_LAPACK_HPP

extern "C" { double dsecnd(void); }

namespace vsmc {

/// A timer use Lapack dsecnd
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

#endif // VSMC_INTERNAL_TIMER_LAPACK_HPP
