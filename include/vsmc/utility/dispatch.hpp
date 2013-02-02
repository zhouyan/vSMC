#ifndef VSMC_UTILITY_PARALLE_DISPATCH_HPP
#define VSMC_UTILITY_PARALLE_DISPATCH_HPP

#include <dispatch/dispatch.h>

namespace vsmc {

/// \brief Dispatch queue used by vSMC
/// \ingroup Utility
class DispatchQueue
{
    public :

    static DispatchQueue &instance ()
    {
        static DispatchQueue queue;

        return queue;
    }

    dispatch_queue_t queue () const
    {
        return queue_;
    }

    void queue (dispatch_queue_t new_queue)
    {
        queue_ = new_queue;
    }

    private :

    dispatch_queue_t queue_;

    DispatchQueue () : queue_(
            dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0)) {}

    DispatchQueue (const DispatchQueue &);
    DispatchQueue &operator= (const DispatchQueue &);
}; // class DispatchQueue

} // namespace vsmc

#endif // VSMC_UTILITY_PARALLE_DISPATCH_HPP
