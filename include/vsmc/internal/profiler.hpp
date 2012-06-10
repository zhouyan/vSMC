#ifndef VSMC_INTERNAL_PROFILER
#define VSMC_INTERNAL_PROFILER

namespace vsmc {

/// \brief A null profiler
///
/// This is the default profiler for StateTBB and StateCL, which does exactly
/// nothing but provides an interface.
class NullProfiler
{
    public :

    NullProfiler () : time_(0) {}

    void start () const {}

    void stop () const {}

    void reset () const
    {
        time_ = 0;
    }

    double time () const
    {
        return time_;
    }

    private :

    mutable double time_;
}; // class NullProfiler

} // namespace vsmc

#endif // VSMC_INTERNAL_PROFILER
