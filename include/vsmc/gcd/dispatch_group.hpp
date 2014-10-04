//============================================================================
// include/vsmc/gcd/dispatch_group.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_GCD_DISPATCH_GROUP_HPP
#define VSMC_GCD_DISPATCH_GROUP_HPP

#include <vsmc/gcd/dispatch_queue.hpp>

namespace vsmc {

/// \brief A Dispatch group
/// \ingroup Dispatch
class DispatchGroup : public DispatchObject< ::dispatch_group_t>
{
    public :

    DispatchGroup () :
        DispatchObject< ::dispatch_group_t>(::dispatch_group_create()) {}

    DispatchGroup (const DispatchGroup &other) :
        DispatchObject< ::dispatch_group_t>(other)
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_retain(this->object());
    }

    DispatchGroup &operator= (const DispatchGroup &other)
    {
        if (this != &other) {
            DispatchObject< ::dispatch_group_t>::operator=(other);
            if (this->object() != VSMC_NULLPTR)
                ::dispatch_retain(this->object());
        }

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchGroup (DispatchGroup &&other) :
        DispatchObject< ::dispatch_group_t>(cxx11::move(other)) {}

    DispatchGroup &operator= (DispatchGroup &&other)
    {
        if (this != &other)
            DispatchObject< ::dispatch_group_t>::operator=(cxx11::move(other));

        return *this;
    }
#endif

    ~DispatchGroup ()
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_release(this->object());
    }

    void enter () const {::dispatch_group_enter(this->object());}

    void leave () const {::dispatch_group_leave(this->object());}

    long wait (::dispatch_time_t timeout) const
    {return ::dispatch_group_wait(this->object(), timeout);}

    template <DispatchQueueType Type>
    void async_f (const DispatchQueue<Type> &queue, void *context,
            ::dispatch_function_t work) const
    {::dispatch_group_async_f(this->object(), queue.object(), context, work);}

    void async_f (::dispatch_queue_t queue, void *context,
            ::dispatch_function_t work) const
    {::dispatch_group_async_f(this->object(), queue, context, work);}

    template <DispatchQueueType Type>
    void notify_f (const DispatchQueue<Type> &queue, void *context,
            ::dispatch_function_t work) const
    {::dispatch_group_notify_f(this->object(), queue.object(), context, work);}

    void notify_f (::dispatch_queue_t queue, void *context,
            ::dispatch_function_t work) const
    {::dispatch_group_notify_f(this->object(), queue, context, work);}

#ifdef __BLOCKS__
    template <DispatchQueueType Type>
    void async (const DispatchQueue<Type> &queue,
            ::dispatch_block_t block) const
    {::dispatch_group_async(this->object(), queue.object(), block);}

    void async (::dispatch_queue_t queue,
            ::dispatch_block_t block) const
    {::dispatch_group_async(this->object(), queue, block);}

    template <DispatchQueueType Type>
    void notify (const DispatchQueue<Type> &queue,
            ::dispatch_block_t block) const
    {::dispatch_group_notify(this->object(), queue.object(), block);}

    void notify (::dispatch_queue_t queue,
            ::dispatch_block_t block) const
    {::dispatch_group_notify(this->object(), queue, block);}
#endif // __BLOCKS__
}; // class DispatchGroup

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_GROUP_HPP
