//============================================================================
// vSMC/include/vsmc/gcd/dispatch_queue.hpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#ifndef VSMC_GCD_DISPATCH_QUEUE_HPP
#define VSMC_GCD_DISPATCH_QUEUE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/gcd/dispatch_object.hpp>
#include <dispatch/dispatch.h>

namespace vsmc
{

/// \brief Types of DispatchQueue
/// \ingroup Dispatch
enum DispatchQueueType {
    DispatchMain,    ///< The queue obtained by `dispatch_get_main_queue`
    DispatchGlobal,  ///< The queue obtained by `dispatch_get_gloal_queue`
    DispatchPrivate  ///< The queue created by `dispatch_queue_create`
};                   // enum DispatchQueueType

template <DispatchQueueType> class DispatchQueue;

/// \brief Base class of DispatchQueue
/// \ingroup Dispatch
class DispatchQueueBase : public DispatchObject<::dispatch_queue_t>
{
    public:
    void resume() const { ::dispatch_resume(this->object()); }

    void suspend() const { ::dispatch_suspend(this->object()); }

    const char *get_label() const
    {
        return ::dispatch_queue_get_label(this->object());
    }

#if VSMC_HAS_GCD_LION
    void *get_specific(const void *key) const
    {
        return ::dispatch_queue_get_specific(this->object(), key);
    }

    void set_specific(const void *key,
                      void *context,
                      ::dispatch_function_t destructor) const
    {
        ::dispatch_queue_set_specific(
            this->object(), key, context, destructor);
    }
#endif  // VSMC_HAS_GCD_LION

    template <typename DispatchType>
    void set_target_queue(const DispatchObject<DispatchType> &object) const
    {
        ::dispatch_set_target_queue(object.object(), this->object());
    }

    void set_target_queue(::dispatch_object_t object) const
    {
        ::dispatch_set_target_queue(object, this->object());
    }

    void after_f(::dispatch_time_t when,
                 void *context,
                 ::dispatch_function_t f) const
    {
        ::dispatch_after_f(when, this->object(), context, f);
    }

    void apply_f(std::size_t iterations,
                 void *context,
                 void (*work)(void *, std::size_t)) const
    {
        ::dispatch_apply_f(iterations, this->object(), context, work);
    }

    void async_f(void *context, ::dispatch_function_t work) const
    {
        ::dispatch_async_f(this->object(), context, work);
    }

    void sync_f(void *context, ::dispatch_function_t work) const
    {
        ::dispatch_sync_f(this->object(), context, work);
    }

#if VSMC_HAS_GCD_LION
    void barrier_async_f(void *context, ::dispatch_function_t work) const
    {
        ::dispatch_barrier_async_f(this->object(), context, work);
    }

    void barrier_sync_f(void *context, ::dispatch_function_t work) const
    {
        ::dispatch_barrier_sync_f(this->object(), context, work);
    }
#endif  // VSMC_HAS_GCD_LION

#ifdef __BLOCKS__
    void after(::dispatch_time_t when, ::dispatch_block_t block) const
    {
        ::dispatch_after(when, this->object(), block);
    }

    void apply(std::size_t iterations, void (^block)(std::size_t)) const
    {
        ::dispatch_apply(iterations, this->object(), block);
    }

    void async(::dispatch_block_t block) const
    {
        ::dispatch_async(this->object(), block);
    }

    void sync(::dispatch_block_t block) const
    {
        ::dispatch_sync(this->object(), block);
    }

#if VSMC_HAS_GCD_LION
    void barrier_async(::dispatch_block_t block) const
    {
        ::dispatch_barrier_async(this->object(), block);
    }

    void barrier_sync(::dispatch_block_t block) const
    {
        ::dispatch_barrier_sync(this->object(), block);
    }
#endif  // VSMC_HAS_GCD_LION
#endif  // __BLOCKS__

    protected:
    DispatchQueueBase(::dispatch_queue_t queue, bool retained)
        : DispatchObject<::dispatch_queue_t>(queue, retained)
    {
    }
};  // class DispatchQueueBase

/// \brief The main dispatch queue (`dipatch_get_main_queue`)
/// \ingroup Dispatch
template <> class DispatchQueue<DispatchMain> : public DispatchQueueBase
{
    public:
    DispatchQueue() : DispatchQueueBase(dispatch_get_main_queue(), false) {}
};  // class DispatchQueue

/// \brief The global dispatch queue (`dispatch_get_gloal_queue`)
/// \ingroup Dispatch
template <> class DispatchQueue<DispatchGlobal> : public DispatchQueueBase
{
    public:
#if VSMC_HAS_GCD_LION
    DispatchQueue(::dispatch_queue_priority_t priority =
                      DISPATCH_QUEUE_PRIORITY_DEFAULT,
                  unsigned long flags = 0)
        : DispatchQueueBase(::dispatch_get_global_queue(priority, flags),
                            false)
    {
    }
#else   // VSMC_HAS_GCD_LION
    DispatchQueue(long priority = 0, unsigned long flags = 0)
        : DispatchQueueBase(::dispatch_get_global_queue(priority, flags),
                            false)
    {
    }
#endif  // VSMC_HAS_GCD_LION
};      // class DispatchQueue

/// \brief A private dispatch queue (`dispatch_queue_create`)
/// \ingroup Dispatch
template <> class DispatchQueue<DispatchPrivate> : public DispatchQueueBase
{
    public:
    DispatchQueue(const char *label = nullptr,
                  ::dispatch_queue_attr_t attr = nullptr)
        : DispatchQueueBase(::dispatch_queue_create(label, attr), true)
    {
    }
};  // class DispatchQueue

}  // namespace vsmc

#endif  // VSMC_GCD_DISPATCH_QUEUE_HPP
