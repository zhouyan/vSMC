//============================================================================
// include/vsmc/utility/dispatch.hpp
//----------------------------------------------------------------------------
//
//                         vSMC: Scalable Monte Carlo
//
// This file is distribured under the 2-clauses BSD License.
// See LICENSE for details.
//============================================================================

#ifndef VSMC_UTILITY_DISPATCH_HPP
#define VSMC_UTILITY_DISPATCH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <dispatch/dispatch.h>
#include <iostream>

namespace vsmc {

/// \brief Types of DispatchQueue
/// \ingroup Dispatch
enum DispatchQueueType {
    DispatchMain,    ///< The queue obtained through `dispatch_get_main_queue`
    DispatchGlobal,  ///< The queue obtained through `dispatch_get_gloal_queue`
    DispatchPrivate  ///< The queue created by `dispatch_queue_create`
}; // enum DispatchQueueType

/// \brief Types of DispatchSource
/// \ingroup Dispatch
enum DispatchSourceType {
    DispatchDataAdd,   ///< DISPATCH_SOURCE_TYPE_DATA_ADD
    DispatchDataOr,    ///< DISPATCH_SOURCE_TYPE_DATA_OR
    DispatchMachRecv,  ///< DISPATCH_SOURCE_TYPE_MACH_RECV
    DispatchMachSend,  ///< DISPATCH_SOURCE_TYPE_MACH_SEND
    DispatchProc,      ///< DISPATCH_SOURCE_TYPE_PROC
    DispatchRead,      ///< DISPATCH_SOURCE_TYPE_READ
    DispatchSignal,    ///< DISPATCH_SOURCE_TYPE_SIGNAL
    DispatchTimer,     ///< DISPATCH_SOURCE_TYPE_TIMER
    DispatchVnode,     ///< DISPATCH_SOURCE_TYPE_VNODE
    DispatchWrite      ///< DISPATCH_SOURCE_TYPE_WRITE
}; // enum DispatchSourceType

template <DispatchQueueType> class DispatchQueue;
template <DispatchSourceType> class DispatchSource;

/// \brief Wrap a callable object into a `dispatch_function_t` type pointer
/// \ingroup Dispatch
///
/// \details
/// It is important that this object live at least after the completion of the
/// work. This is a thin wrapper around a regular C++ callable object
/// (including lambda expressions) that make it easier to interface them with
/// GCD. The memory management issue is the same as using raw GCD interface
/// with a context variable, which shall not be destroyed before the execution
/// of the function finishes. The original object does not need to exist after
/// the creation of DispatchFunction.
template <typename T>
class DispatchFunction
{
    public :

    DispatchFunction (const T &work) : work_(work) {}

    DispatchFunction (const DispatchFunction<T> &other) :
        work_(other.work_) {}

    DispatchFunction<T> &operator= (const DispatchFunction<T> &other)
    {
        if (this != &other)
            work_ = other.work_;

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchFunction (T &&work) VSMC_NOEXCEPT : work_(cxx11::move(work)) {}

    DispatchFunction (DispatchFunction<T> &&other) VSMC_NOEXCEPT :
        work_(cxx11::move(other.work_)) {}

    DispatchFunction<T> &operator= (DispatchFunction<T> &&other) VSMC_NOEXCEPT
    {
        if (this != &other)
            work_ = cxx11::move(other.work_);

        return *this;
    }
#endif

    void *context () {return static_cast<void *>(this);}

    dispatch_function_t function () const {return function_;}

    private :

    T work_;

    static void function_ (void *ctx)
    {
        DispatchFunction<T> *df_ptr = static_cast<DispatchFunction<T> *>(ctx);
        df_ptr->work_();
    }
}; // class DispatchFunction

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Make a DispatchFunction object from an arbitrary callable object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(void)`
template <typename T>
inline DispatchFunction<
typename cxx11::remove_cv<typename cxx11::remove_reference<T>::type>::type>
*dispatch_function_new (T &&work) VSMC_NOEXCEPT
{
    typedef typename cxx11::remove_reference<T>::type U;
    typedef typename cxx11::remove_cv<U>::type V;
    return new DispatchFunction<V>(cxx11::forward<T>(work));
}
#else
template <typename T>
inline DispatchFunction<T> *dispatch_function_new (const T &work)
{return new DispatchFunction<T>(work);}
#endif

/// \brief Base class of Dispatch objects
/// \ingroup Dispatch
///
/// \details All Dispatch objects are reference counting shared objects
template <typename DispatchType>
class DispatchObject
{
    public :

    /// \brief Create a DispatchObject from its C-type object
    ///
    /// \details
    /// The original object will be retained by this object
    explicit DispatchObject (const DispatchType &object) : object_(object)
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object);
    }

    DispatchObject (const DispatchObject<DispatchType> &other) :
        object_(other.object_)
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);
    }

    DispatchObject<DispatchType> &operator= (
            const DispatchObject<DispatchType> &other)
    {
        if (this == &other)
            return *this;

        if (object_ == other.object_)
            return *this;

        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
        object_ = other.object_;
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);

        return *this;
    }

    ~DispatchObject ()
    {
        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
    }

    /// \brief Return the underlying Dispatch object
    DispatchType object () const {return object_;}

    /// \brief Set the underlying Dispatch object and retain it
    void object (DispatchType obj)
    {
        if (object_ == obj)
            return;

        if (object_ != VSMC_NULLPTR)
            ::dispatch_release(object_);
        object_ = obj;
        if (object_ != VSMC_NULLPTR)
            ::dispatch_retain(object_);
    }

    void *get_context () const
    {return ::dispatch_get_context(object_);}

    void set_context (void *context) const
    {::dispatch_set_context(object_, context);}

    void set_finalizer_f (dispatch_function_t finalizer) const
    {::dispatch_set_finalizer_f(object_, finalizer);}

    private :

    DispatchType object_;
}; // class DispatchObject

/// \brief Base class of DispatchQueue
/// \ingroup Dispatch
class DispatchQueueBase : public DispatchObject<dispatch_queue_t>
{
    public :

    void resume () const {::dispatch_resume(this->object());}

    void suspend () const {::dispatch_suspend(this->object());}

    const char *get_label () const
    {return ::dispatch_queue_get_label(this->object());}

#if VSMC_USE_GCD_LION
    void *get_specific (const void *key) const
    {return ::dispatch_queue_get_specific(this->object(), key);}

    void set_specific (const void *key, void *context,
            dispatch_function_t destructor) const
    {::dispatch_queue_set_specific(this->object(), key, context, destructor);}
#endif // VSMC_USE_GCD_LION

    template <typename DispatchType>
    void set_target_queue (const DispatchObject<DispatchType> &object) const
    {::dispatch_set_target_queue(object.object(), this->object());}

    void set_target_queue (dispatch_object_t object) const
    {::dispatch_set_target_queue(object, this->object());}

    void after_f (dispatch_time_t when, void *context,
            dispatch_function_t f) const
    {::dispatch_after_f(when, this->object(), context, f);}

    void apply_f (std::size_t iterations, void *context,
            void (*work) (void *, std::size_t)) const
    {::dispatch_apply_f(iterations, this->object(), context, work);}

    void async_f (void *context, dispatch_function_t work) const
    {::dispatch_async_f(this->object(), context, work);}

    void sync_f (void *context, dispatch_function_t work) const
    {::dispatch_sync_f(this->object(), context, work);}

#if VSMC_USE_GCD_LION
    void barrier_async_f (void *context, dispatch_function_t work) const
    {::dispatch_barrier_async_f(this->object(), context, work);}

    void barrier_sync_f (void *context, dispatch_function_t work) const
    {::dispatch_barrier_sync_f(this->object(), context, work);}
#endif // VSMC_USE_GCD_LION

#ifdef __BLOCKS__
    void after (dispatch_time_t when, dispatch_block_t block) const
    {::dispatch_after(when, this->object(), block);}

    void apply (std::size_t iterations, void (^block) (std::size_t)) const
    {::dispatch_apply(iterations, this->object(), block);}

    void async (dispatch_block_t block) const
    {::dispatch_async(this->object(), block);}

    void sync (dispatch_block_t block) const
    {::dispatch_sync(this->object(), block);}

#if VSMC_USE_GCD_LION
    void barrier_async (dispatch_block_t block) const
    {::dispatch_barrier_async(this->object(), block);}

    void barrier_sync (dispatch_block_t block) const
    {::dispatch_barrier_sync(this->object(), block);}
#endif // VSMC_USE_GCD_LION
#endif // __BLOCKS__

    protected :

    DispatchQueueBase (dispatch_queue_t queue) :
        DispatchObject<dispatch_queue_t>(queue) {}

    DispatchQueueBase (const DispatchQueueBase &other) :
        DispatchObject<dispatch_queue_t>(other) {}

    DispatchQueueBase &operator= (const DispatchQueueBase &other)
    {
        if (this != &other)
            DispatchObject<dispatch_queue_t>::operator=(other);

        return *this;
    }
}; // class DispatchQueueBase

/// \brief The main dispatch queue (`dipatch_get_main_queue`)
/// \ingroup Dispatch
template <>
class DispatchQueue<DispatchMain> : public DispatchQueueBase
{
    public :

    DispatchQueue () : DispatchQueueBase(dispatch_get_main_queue()) {}
}; // class DispatchQueue

/// \brief The global (concurrent) dispatch queue (`dispatch_get_gloal_queue`)
/// \ingroup Dispatch
template <>
class DispatchQueue<DispatchGlobal> : public DispatchQueueBase
{
    public :

#if VSMC_USE_GCD_LION
    DispatchQueue (dispatch_queue_priority_t priority =
            DISPATCH_QUEUE_PRIORITY_DEFAULT, unsigned long flags = 0) :
        DispatchQueueBase(::dispatch_get_global_queue(priority, flags)) {}
#else // VSMC_USE_GCD_LION
    DispatchQueue (long priority = 0, unsigned long flags = 0) :
        DispatchQueueBase(::dispatch_get_global_queue(priority, flags)) {}
#endif // VSMC_USE_GCD_LION
}; // class DispatchQueue

/// \brief A private dispatch queue (`dispatch_queue_create`)
/// \ingroup Dispatch
template <>
class DispatchQueue<DispatchPrivate> : public DispatchQueueBase
{
    public :

    DispatchQueue (const char *label = VSMC_NULLPTR,
            dispatch_queue_attr_t attr = VSMC_NULLPTR) :
        DispatchQueueBase(::dispatch_queue_create(label, attr)) {}

    DispatchQueue (const DispatchQueue<DispatchPrivate> &other) :
        DispatchQueueBase(other)
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_retain(this->object());
    }

    DispatchQueue<DispatchPrivate> &operator= (
            const DispatchQueue<DispatchPrivate> &other)
    {
        if (this != &other) {
            DispatchQueueBase::operator=(other);
            if (this->object() != VSMC_NULLPTR)
                ::dispatch_retain(this->object());
        }

        return *this;
    }

    ~DispatchQueue ()
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_release(this->object());
    }
}; // class DispatchQueue

/// \brief A Dispatch group
/// \ingroup Dispatch
class DispatchGroup : public DispatchObject<dispatch_group_t>
{
    public :

    DispatchGroup () :
        DispatchObject<dispatch_group_t>(::dispatch_group_create()) {}

    DispatchGroup (const DispatchGroup &other) :
        DispatchObject<dispatch_group_t>(other)
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_retain(this->object());
    }

    DispatchGroup &operator= (const DispatchGroup &other)
    {
        if (this != &other) {
            DispatchObject<dispatch_group_t>::operator=(other);
            if (this->object() != VSMC_NULLPTR)
                ::dispatch_retain(this->object());
        }

        return *this;
    }

    ~DispatchGroup ()
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_release(this->object());
    }

    void enter () const {::dispatch_group_enter(this->object());}

    void leave () const {::dispatch_group_leave(this->object());}

    long wait (dispatch_time_t timeout) const
    {return ::dispatch_group_wait(this->object(), timeout);}

    template <DispatchQueueType Type>
    void async_f (const DispatchQueue<Type> &queue, void *context,
            dispatch_function_t work) const
    {::dispatch_group_async_f(this->object(), queue.object(), context, work);}

    void async_f (dispatch_queue_t queue, void *context,
            dispatch_function_t work) const
    {::dispatch_group_async_f(this->object(), queue, context, work);}

    template <DispatchQueueType Type>
    void notify_f (const DispatchQueue<Type> &queue, void *context,
            dispatch_function_t work) const
    {::dispatch_group_notify_f(this->object(), queue.object(), context, work);}

    void notify_f (dispatch_queue_t queue, void *context,
            dispatch_function_t work) const
    {::dispatch_group_notify_f(this->object(), queue, context, work);}

#ifdef __BLOCKS__
    template <DispatchQueueType Type>
    void async (const DispatchQueue<Type> &queue,
            dispatch_block_t block) const
    {::dispatch_group_async(this->object(), queue.object(), block);}

    void async (dispatch_queue_t queue,
            dispatch_block_t block) const
    {::dispatch_group_async(this->object(), queue, block);}

    template <DispatchQueueType Type>
    void notify (const DispatchQueue<Type> &queue,
            dispatch_block_t block) const
    {::dispatch_group_notify(this->object(), queue.object(), block);}

    void notify (dispatch_queue_t queue,
            dispatch_block_t block) const
    {::dispatch_group_notify(this->object(), queue, block);}
#endif // __BLOCKS__
}; // class DispatchGroup

/// \brief Base class of DispatchSource
/// \ingroup Dispatch
///
/// \bug A DispachSource object is manually retained when created. It is
/// supposed to be retained by `dispatch_source_create` according to the
/// documents. But this seems not to be the case in the current implementation
/// (Mac OS X 10.9). The worst case is that a source object is retained one
/// more time than it is released. A simple test example is,
/// ~~~{.cpp}
/// dispatch_source_t source = dispatch_source_create( /* arguments */ );
/// dispatch_release(source); // generate error
/// ~~~
template <DispatchSourceType Type>
class DispatchSourceBase : public DispatchObject<dispatch_source_t>
{
    public :

    void resume () const {::dispatch_resume(this->object());}

    void suspend () const {::dispatch_suspend(this->object());}

    void cancel () const {::dispatch_source_cancel(this->object());}

    long testcancel () const
    {return ::dispatch_source_testcancel(this->object());}

    unsigned long get_data () const
    {return ::dispatch_source_get_data(this->object());}

    uintptr_t get_handle () const
    {return ::dispatch_source_get_handle(this->object());}

    unsigned long get_mask () const
    {return ::dispatch_source_get_mask(this->object());}

    void set_cancel_handler_f (dispatch_function_t cancel_handler) const
    {::dispatch_source_set_cancel_handler_f(this->object(), cancel_handler);}

    void set_event_handler_f (dispatch_function_t event_handler) const
    {::dispatch_source_set_event_handler_f(this->object(), event_handler);}

#if VSMC_USE_GCD_LION
    void set_registration_handler_f (dispatch_function_t
            registration_handler) const
    {
        ::dispatch_source_set_registration_handler_f(
                this->object(), registration_handler);
    }
#endif // VSMC_USE_GCD_LION

#ifdef __BLOCKS__
    void set_cancel_handler (dispatch_block_t cancel_handler) const
    {::dispatch_source_set_cancel_handler(this->object(), cancel_handler);}

    void set_event_handler (dispatch_block_t event_handler) const
    {::dispatch_source_set_event_handler(this->object(), event_handler);}

#if VSMC_USE_GCD_LION
    void set_registration_handler (dispatch_block_t
            registration_handler) const
    {
        ::dispatch_source_set_registration_handler(
                this->object(), registration_handler);
    }
#endif // VSMC_USE_GCD_LION
#endif // __BLOCKS__

    private :

    template <DispatchSourceType> struct source_type {};

    protected :

    DispatchSourceBase (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchObject<dispatch_source_t>(::dispatch_source_create(
                    source_type_t(source_type<Type>()),
                    handle, mask, queue))
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_retain(this->object());
    }

    DispatchSourceBase (const DispatchSourceBase &other) :
        DispatchObject<dispatch_source_t>(other)
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_retain(this->object());
    }

    DispatchSourceBase &operator= (const DispatchSourceBase &other)
    {
        if (this != &other) {
            DispatchObject<dispatch_source_t>::operator=(other);
            if (this->object() != VSMC_NULLPTR)
                ::dispatch_retain(this->object());
        }

        return *this;
    }

    ~DispatchSourceBase ()
    {
        if (this->object() != VSMC_NULLPTR)
            ::dispatch_release(this->object());
    }

    private :

    static dispatch_source_type_t source_type_t (source_type<DispatchDataAdd>)
    {return DISPATCH_SOURCE_TYPE_DATA_ADD;}

    static dispatch_source_type_t source_type_t (source_type<DispatchDataOr>)
    {return DISPATCH_SOURCE_TYPE_DATA_OR;}

    static dispatch_source_type_t source_type_t (source_type<DispatchMachRecv>)
    {return DISPATCH_SOURCE_TYPE_MACH_RECV;}

    static dispatch_source_type_t source_type_t (source_type<DispatchMachSend>)
    {return DISPATCH_SOURCE_TYPE_MACH_SEND;}

    static dispatch_source_type_t source_type_t (source_type<DispatchProc>)
    {return DISPATCH_SOURCE_TYPE_PROC;}

    static dispatch_source_type_t source_type_t (source_type<DispatchRead>)
    {return DISPATCH_SOURCE_TYPE_READ;}

    static dispatch_source_type_t source_type_t (source_type<DispatchSignal>)
    {return DISPATCH_SOURCE_TYPE_SIGNAL;}

    static dispatch_source_type_t source_type_t (source_type<DispatchTimer>)
    {return DISPATCH_SOURCE_TYPE_TIMER;}

    static dispatch_source_type_t source_type_t (source_type<DispatchVnode>)
    {return DISPATCH_SOURCE_TYPE_VNODE;}

    static dispatch_source_type_t source_type_t (source_type<DispatchWrite>)
    {return DISPATCH_SOURCE_TYPE_WRITE;}
}; // class DispatchSourceBase

/// \brief A dispatch source
/// \ingroup Dispatch
template <DispatchSourceType Type>
class DispatchSource : public DispatchSourceBase<Type>
{
    public :

    template <DispatchQueueType QType>
    DispatchSource (uintptr_t handle, unsigned long mask,
            const DispatchQueue<QType> &queue) :
        DispatchSourceBase<Type>(handle, mask, queue.object()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<Type>(handle, mask, queue) {}
}; // class DispatchSource

/// \brief Data (ADD) dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchDataAdd> :
    public DispatchSourceBase<DispatchDataAdd>
{
    public :

    template <DispatchQueueType QType>
    DispatchSource (uintptr_t handle, unsigned long mask,
            const DispatchQueue<QType> &queue) :
        DispatchSourceBase<DispatchDataAdd>(handle, mask, queue.object()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchDataAdd>(handle, mask, queue) {}

    void merge_data (unsigned long value) const
    {::dispatch_source_merge_data(this->object(), value);}
}; // class DispatchSource

/// \brief Data (OR) dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchDataOr> :
    public DispatchSourceBase<DispatchDataOr>
{
    public :

    template <DispatchQueueType QType>
    DispatchSource (uintptr_t handle, unsigned long mask,
            const DispatchQueue<QType> &queue) :
        DispatchSourceBase<DispatchDataOr>(handle, mask, queue.object()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchDataOr>(handle, mask, queue) {}

    void merge_data (unsigned long value) const
    {::dispatch_source_merge_data(this->object(), value);}
}; // class DispatchSource

/// \brief Timer dispatch source
/// \ingroup Dispatch
template <>
class DispatchSource<DispatchTimer> :
    public DispatchSourceBase<DispatchTimer>
{
    public :

    template <DispatchQueueType QType>
    DispatchSource (uintptr_t handle, unsigned long mask,
            const DispatchQueue<QType> &queue) :
        DispatchSourceBase<DispatchTimer>(handle, mask, queue.object()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchTimer>(handle, mask, queue) {}

    void set_timer (dispatch_time_t start,
            uint64_t interval, uint64_t leeway) const
    {::dispatch_source_set_timer(this->object(), start, interval, leeway);}
}; // class DispatchSource

} // namespace vsmc

#endif // VSMC_UTILITY_DISPATCH_HPP
