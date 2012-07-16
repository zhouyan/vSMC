#ifndef VSMC_TIMER_NULL_TIMER_HPP
#define VSMC_TIMER_NULL_TIMER_HPP

namespace vsmc {

/// \brief A null timer
/// \ingroup Timer
///
/// \details
/// This is the default timer, which does exactly nothing but provides the
/// required interface. Using this one in places where a timer is expected and
/// no runtime cost shall be incurred.
class NullTimer
{
    public :

    NullTimer () : time_(0) {}

    void start () const {}

    void stop () const {}

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
}; // class NullTimer

} // namespace vsmc

#endif // VSMC_TIMER_NULL_TIMER_HPP
