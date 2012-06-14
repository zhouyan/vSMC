#ifndef VSMC_INTERNAL_TIMER
#define VSMC_INTERNAL_TIMER

namespace vsmc {

/// \brief A null timer
///
/// This is the default timer for StateTBB and StateCL, which does exactly
/// nothing but provides an interface.
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

#endif // VSMC_INTERNAL_Timer
