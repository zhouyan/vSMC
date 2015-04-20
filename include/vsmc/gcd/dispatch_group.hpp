//============================================================================
// vSMC/include/vsmc/gcd/dispatch_group.hpp
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

#ifndef VSMC_GCD_DISPATCH_GROUP_HPP
#define VSMC_GCD_DISPATCH_GROUP_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/gcd/dispatch_object.hpp>
#include <vsmc/gcd/dispatch_queue.hpp>
#include <dispatch/dispatch.h>

namespace vsmc
{

/// \brief A Dispatch group
/// \ingroup Dispatch
class DispatchGroup : public DispatchObject<::dispatch_group_t>
{
    public:
    DispatchGroup()
        : DispatchObject<::dispatch_group_t>(::dispatch_group_create(), true)
    {
    }

    void enter() const { ::dispatch_group_enter(this->object()); }

    void leave() const { ::dispatch_group_leave(this->object()); }

    long wait(::dispatch_time_t timeout) const
    {
        return ::dispatch_group_wait(this->object(), timeout);
    }

    template <DispatchQueueType Type>
    void async_f(const DispatchQueue<Type> &queue, void *context,
        ::dispatch_function_t work) const
    {
        ::dispatch_group_async_f(
            this->object(), queue.object(), context, work);
    }

    void async_f(::dispatch_queue_t queue, void *context,
        ::dispatch_function_t work) const
    {
        ::dispatch_group_async_f(this->object(), queue, context, work);
    }

    template <DispatchQueueType Type>
    void notify_f(const DispatchQueue<Type> &queue, void *context,
        ::dispatch_function_t work) const
    {
        ::dispatch_group_notify_f(
            this->object(), queue.object(), context, work);
    }

    void notify_f(::dispatch_queue_t queue, void *context,
        ::dispatch_function_t work) const
    {
        ::dispatch_group_notify_f(this->object(), queue, context, work);
    }

#ifdef __BLOCKS__
    template <DispatchQueueType Type>
    void async(
        const DispatchQueue<Type> &queue, ::dispatch_block_t block) const
    {
        ::dispatch_group_async(this->object(), queue.object(), block);
    }

    void async(::dispatch_queue_t queue, ::dispatch_block_t block) const
    {
        ::dispatch_group_async(this->object(), queue, block);
    }

    template <DispatchQueueType Type>
    void notify(
        const DispatchQueue<Type> &queue, ::dispatch_block_t block) const
    {
        ::dispatch_group_notify(this->object(), queue.object(), block);
    }

    void notify(::dispatch_queue_t queue, ::dispatch_block_t block) const
    {
        ::dispatch_group_notify(this->object(), queue, block);
    }
#endif // __BLOCKS__
};     // class DispatchGroup

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_GROUP_HPP
