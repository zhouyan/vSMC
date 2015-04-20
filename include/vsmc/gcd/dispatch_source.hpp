//============================================================================
// vSMC/include/vsmc/gcd/dispatch_source.hpp
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

#ifndef VSMC_GCD_DISPATCH_SOURCE_HPP
#define VSMC_GCD_DISPATCH_SOURCE_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/gcd/dispatch_object.hpp>
#include <vsmc/gcd/dispatch_queue.hpp>

namespace vsmc
{

/// \brief Types of DispatchSource
/// \ingroup Dispatch
enum DispatchSourceType {
    DispatchDataAdd,  ///< DISPATCH_SOURCE_TYPE_DATA_ADD
    DispatchDataOr,   ///< DISPATCH_SOURCE_TYPE_DATA_OR
    DispatchMachRecv, ///< DISPATCH_SOURCE_TYPE_MACH_RECV
    DispatchMachSend, ///< DISPATCH_SOURCE_TYPE_MACH_SEND
    DispatchProc,     ///< DISPATCH_SOURCE_TYPE_PROC
    DispatchRead,     ///< DISPATCH_SOURCE_TYPE_READ
    DispatchSignal,   ///< DISPATCH_SOURCE_TYPE_SIGNAL
    DispatchTimer,    ///< DISPATCH_SOURCE_TYPE_TIMER
    DispatchVnode,    ///< DISPATCH_SOURCE_TYPE_VNODE
    DispatchWrite     ///< DISPATCH_SOURCE_TYPE_WRITE
};                    // enum DispatchSourceType

template <DispatchSourceType> class DispatchSource;

/// \brief Base class of DispatchSource
/// \ingroup Dispatch
///
/// \bug A DispachSource object is manually retained when created. It is
/// supposed to be retained by `dispatch_source_create` according to the
/// documents. But this seems not to be the case in the current implementation
/// (Mac OS X 10.9). The worst case is that a source object is retained one
/// more time than it is released. A simple test example is,
/// ~~~{.cpp}
/// ::dispatch_source_t source = ::dispatch_source_create( /* arguments */ );
/// ::dispatch_release(source); // generate error
/// ~~~
template <DispatchSourceType Type>
class DispatchSourceBase : public DispatchObject<::dispatch_source_t>
{
    public:
    void resume() const { ::dispatch_resume(this->object()); }

    void suspend() const { ::dispatch_suspend(this->object()); }

    void cancel() const { ::dispatch_source_cancel(this->object()); }

    long testcancel() const
    {
        return ::dispatch_source_testcancel(this->object());
    }

    unsigned long get_data() const
    {
        return ::dispatch_source_get_data(this->object());
    }

    uintptr_t get_handle() const
    {
        return ::dispatch_source_get_handle(this->object());
    }

    unsigned long get_mask() const
    {
        return ::dispatch_source_get_mask(this->object());
    }

    void set_cancel_handler_f(::dispatch_function_t cancel_handler) const
    {
        ::dispatch_source_set_cancel_handler_f(
            this->object(), cancel_handler);
    }

    void set_event_handler_f(::dispatch_function_t event_handler) const
    {
        ::dispatch_source_set_event_handler_f(this->object(), event_handler);
    }

#if VSMC_HAS_GCD_LION
    void set_registration_handler_f(
        ::dispatch_function_t registration_handler) const
    {
        ::dispatch_source_set_registration_handler_f(
            this->object(), registration_handler);
    }
#endif // VSMC_HAS_GCD_LION

#ifdef __BLOCKS__
    void set_cancel_handler(::dispatch_block_t cancel_handler) const
    {
        ::dispatch_source_set_cancel_handler(this->object(), cancel_handler);
    }

    void set_event_handler(::dispatch_block_t event_handler) const
    {
        ::dispatch_source_set_event_handler(this->object(), event_handler);
    }

#if VSMC_HAS_GCD_LION
    void set_registration_handler(
        ::dispatch_block_t registration_handler) const
    {
        ::dispatch_source_set_registration_handler(
            this->object(), registration_handler);
    }
#endif // VSMC_HAS_GCD_LION
#endif // __BLOCKS__

    private:
    template <DispatchSourceType> struct source_type {
    };

    protected:
    DispatchSourceBase(
        uintptr_t handle, unsigned long mask, ::dispatch_queue_t queue)
        : DispatchObject<::dispatch_source_t>(
              ::dispatch_source_create(
                  source_type_t(source_type<Type>()), handle, mask, queue),
              false)
    {
    }

    private:
    static ::dispatch_source_type_t source_type_t(
        source_type<DispatchDataAdd>)
    {
        return DISPATCH_SOURCE_TYPE_DATA_ADD;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchDataOr>)
    {
        return DISPATCH_SOURCE_TYPE_DATA_OR;
    }

    static ::dispatch_source_type_t source_type_t(
        source_type<DispatchMachRecv>)
    {
        return DISPATCH_SOURCE_TYPE_MACH_RECV;
    }

    static ::dispatch_source_type_t source_type_t(
        source_type<DispatchMachSend>)
    {
        return DISPATCH_SOURCE_TYPE_MACH_SEND;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchProc>)
    {
        return DISPATCH_SOURCE_TYPE_PROC;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchRead>)
    {
        return DISPATCH_SOURCE_TYPE_READ;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchSignal>)
    {
        return DISPATCH_SOURCE_TYPE_SIGNAL;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchTimer>)
    {
        return DISPATCH_SOURCE_TYPE_TIMER;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchVnode>)
    {
        return DISPATCH_SOURCE_TYPE_VNODE;
    }

    static ::dispatch_source_type_t source_type_t(source_type<DispatchWrite>)
    {
        return DISPATCH_SOURCE_TYPE_WRITE;
    }
}; // class DispatchSourceBase

/// \brief A dispatch source
/// \ingroup Dispatch
template <DispatchSourceType Type>
class DispatchSource : public DispatchSourceBase<Type>
{
    public:
    template <DispatchQueueType QType>
    DispatchSource(uintptr_t handle, unsigned long mask,
        const DispatchQueue<QType> &queue)
        : DispatchSourceBase<Type>(handle, mask, queue.object())
    {
    }

    DispatchSource(
        uintptr_t handle, unsigned long mask, ::dispatch_queue_t queue)
        : DispatchSourceBase<Type>(handle, mask, queue)
    {
    }
}; // class DispatchSource

/// \brief Data (ADD) dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchDataAdd>
    : public DispatchSourceBase<DispatchDataAdd>
{
    public:
    template <DispatchQueueType QType>
    DispatchSource(uintptr_t handle, unsigned long mask,
        const DispatchQueue<QType> &queue)
        : DispatchSourceBase<DispatchDataAdd>(handle, mask, queue.object())
    {
    }

    DispatchSource(
        uintptr_t handle, unsigned long mask, ::dispatch_queue_t queue)
        : DispatchSourceBase<DispatchDataAdd>(handle, mask, queue)
    {
    }

    void merge_data(unsigned long value) const
    {
        ::dispatch_source_merge_data(this->object(), value);
    }
}; // class DispatchSource

/// \brief Data (OR) dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchDataOr>
    : public DispatchSourceBase<DispatchDataOr>
{
    public:
    template <DispatchQueueType QType>
    DispatchSource(uintptr_t handle, unsigned long mask,
        const DispatchQueue<QType> &queue)
        : DispatchSourceBase<DispatchDataOr>(handle, mask, queue.object())
    {
    }

    DispatchSource(
        uintptr_t handle, unsigned long mask, ::dispatch_queue_t queue)
        : DispatchSourceBase<DispatchDataOr>(handle, mask, queue)
    {
    }

    void merge_data(unsigned long value) const
    {
        ::dispatch_source_merge_data(this->object(), value);
    }
}; // class DispatchSource

/// \brief Timer dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchTimer> : public DispatchSourceBase<DispatchTimer>
{
    public:
    template <DispatchQueueType QType>
    DispatchSource(uintptr_t handle, unsigned long mask,
        const DispatchQueue<QType> &queue)
        : DispatchSourceBase<DispatchTimer>(handle, mask, queue.object())
    {
    }

    DispatchSource(
        uintptr_t handle, unsigned long mask, ::dispatch_queue_t queue)
        : DispatchSourceBase<DispatchTimer>(handle, mask, queue)
    {
    }

    void set_timer(
        ::dispatch_time_t start, uint64_t interval, uint64_t leeway) const
    {
        ::dispatch_source_set_timer(this->object(), start, interval, leeway);
    }
}; // class DispatchSource

} // namespace vsmc

#endif // VSMC_GCD_DISPATCH_SOURCE_HPP
