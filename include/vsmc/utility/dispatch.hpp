#ifndef VSMC_UTILITY_DISPATCH_HPP
#define VSMC_UTILITY_DISPATCH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <dispatch/dispatch.h>
#include <iostream>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:4351)
#endif

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

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void *get_specific (const void *key) const
    {return ::dispatch_queue_get_specific(this->object(), key);}

    void set_specific (const void *key, void *context,
            dispatch_function_t destructor) const
    {::dispatch_queue_set_specific(this->object(), key, context, destructor);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

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

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void barrier_async_f (void *context, dispatch_function_t work) const
    {::dispatch_barrier_async_f(this->object(), context, work);}

    void barrier_sync_f (void *context, dispatch_function_t work) const
    {::dispatch_barrier_sync_f(this->object(), context, work);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

#ifdef __BLOCKS__
    void after (dispatch_time_t when, dispatch_block_t block) const
    {::dispatch_after(when, this->object(), block);}

    void apply (std::size_t iterations, void (^block) (std::size_t)) const
    {::dispatch_apply(iterations, this->object(), block);}

    void async (dispatch_block_t block) const
    {::dispatch_async(this->object(), block);}

    void sync (dispatch_block_t block) const
    {::dispatch_sync(this->object(), block);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void barrier_async (dispatch_block_t block) const
    {::dispatch_barrier_async(this->object(), block);}

    void barrier_sync (dispatch_block_t block) const
    {::dispatch_barrier_sync(this->object(), block);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
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

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    DispatchQueue (dispatch_queue_priority_t priority =
            DISPATCH_QUEUE_PRIORITY_DEFAULT, unsigned long flags = 0) :
        DispatchQueueBase(::dispatch_get_global_queue(priority, flags)) {}
#else // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    DispatchQueue (long priority = 0, unsigned long flags = 0) :
        DispatchQueueBase(::dispatch_get_global_queue(priority, flags)) {}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
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

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void set_registration_handler_f (dispatch_function_t
            registration_handler) const
    {
        ::dispatch_source_set_registration_handler_f(
                this->object(), registration_handler);
    }
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

#ifdef __BLOCKS__
    void set_cancel_handler (dispatch_block_t cancel_handler) const
    {::dispatch_source_set_cancel_handler(this->object(), cancel_handler);}

    void set_event_handler (dispatch_block_t event_handler) const
    {::dispatch_source_set_event_handler(this->object(), event_handler);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void set_registration_handler (dispatch_block_t
            registration_handler) const
    {
        ::dispatch_source_set_registration_handler(
                this->object(), registration_handler);
    }
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
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

/// \brief Display a progress bar while algorithm proceed
/// \ingroup Dispatch
class DispatchProgress
{
    public :

    /// \brief Construct a DispatchProgress with total amount of work
    DispatchProgress (std::ostream &os = std::cout) :
        queue_("vsmc::DispatchProgress::queue_"), timer_(0, 0, queue_),
        total_(0), iter_(0), print_first_(true),
        num_equal_(0), percent_(0), elapsed_second_(0), last_iter_(0),
        display_progress_(), display_percent_(), display_time_(),
        display_iter_(), os_(os) {}

    explicit DispatchProgress (const DispatchQueue<DispatchPrivate> &queue,
            std::ostream &os = std::cout) :
        queue_(queue), timer_(0, 0, queue_),
        total_(0), iter_(0), print_first_(true),
        num_equal_(0), percent_(0), elapsed_second_(0), last_iter_(0),
        display_progress_(), display_percent_(), display_time_(),
        display_iter_(), os_(os) {}

    /// \brief Start to print the progress
    ///
    /// \param total Total amount of work represented by an integer, for
    /// example file size or SMC algorithm total number of iterations
    /// \param interval The interval in nanoseconds of timer
    /// \param leeway The leeway in nanoseconds of timer
    void start (uint64_t total,
            uint64_t interval = NSEC_PER_SEC / 10,
            uint64_t leeway = NSEC_PER_SEC / 10)
    {
        total_ = total;
        iter_ = 0;
        print_first_ = true;

        timer_.set_context(static_cast<void *>(this));
        timer_.set_event_handler_f(print_start_);
        timer_.set_timer(DISPATCH_TIME_NOW, interval, leeway);

        watch_.reset();
        watch_.start();
        timer_.resume();
    }

    /// \brief Stop to print the progress
    ///
    /// \param finished If true, then it is assumed that all work has been
    /// finished, and at the end the progress will be shown as `100%` and
    /// `total/total`, where total is the first parameter of `start`.
    /// Otherwise, whatever progress has been made will be shown. For example,
    /// consider that there are a large number of iterations, say `N`, which is
    /// not a multiple of `1000`, but because each iteration progress very
    /// fast, it will be very inefficient to call `increment` every time the
    /// algorithm moves to the next iteration. Instead, one may call it every
    /// `1000` iterations with `increment(1000)`. At the end, the
    /// DispatchProgress object will only know that there is a multiple of
    /// `1000` iterations. One may either call
    /// `increment(N - (N / 1000) * 1000); stop();` or simply `stop(true)`.
    void stop (bool finished = false)
    {
        timer_.suspend();
        if (finished && iter_ < total_)
            increment(total_ - iter_);
        queue_.sync_f(static_cast<void *>(this), print_stop_);
        watch_.stop();
    }

    private :

    struct step_context
    {
        step_context (uint64_t s, DispatchProgress *p) : step(s), timer(p) {}

        uint64_t step;
        DispatchProgress *timer;
    };

    public :

    /// \brief Increase the progress count
    void increment (uint64_t step = 1)
    {
        if (step == 0) {
            return;
        } else if (step == 1) {
            queue_.async_f(static_cast<void *>(this), increment_);
        } else {
            step_context *context = new step_context(step, this);
            queue_.async_f(static_cast<void *>(context), increment_step_);
        }
    }

    private :

    DispatchQueue<DispatchPrivate> queue_;
    DispatchSource<DispatchTimer> timer_;
    StopWatch watch_;

    uint64_t total_;
    uint64_t iter_;
    bool print_first_;

    unsigned num_equal_;
    unsigned percent_;
    uint64_t elapsed_second_;
    uint64_t last_iter_;

    char display_progress_[128];
    char display_percent_[32];
    char display_time_[32];
    char display_iter_[64];

    std::ostream &os_;

    static VSMC_CONSTEXPR const unsigned num_equal_max_ = 60;
    static VSMC_CONSTEXPR const unsigned num_dash_max_ = 1;
    static VSMC_CONSTEXPR const unsigned percent_max_ = 100;

    template <typename UIntType>
    static void uint_to_char (UIntType num, char *cstr, std::size_t &offset)
    {
        if (num == 0) {
            cstr[offset++] = '0';
            return;
        }

        char utmp[32];
        std::size_t unum = 0;
        while (num) {
            utmp[unum++] = '0' + static_cast<char>(num % 10);
            num /= 10;
        }
        for (std::size_t i = unum; i != 0; --i)
            cstr[offset++] = utmp[i - 1];
    }

    template <typename UIntType>
    static unsigned uint_digit (UIntType num)
    {
        if (num == 0)
            return 1;

        unsigned digit = 0;
        while (num != 0) {
            ++digit;
            num /= 10;
        }

        return digit;
    }

    static void print_progress (void *context)
    {
        DispatchProgress *timer_ptr = static_cast<DispatchProgress *>(context);

        uint64_t iter = timer_ptr->iter_;
        uint64_t total = timer_ptr->total_;

        const StopWatch &elapsed = timer_ptr->watch_;
        elapsed.stop();
        elapsed.start();
        uint64_t elapsed_second = static_cast<uint64_t>(elapsed.seconds());

        uint64_t display_iter = iter <= total ? iter : total;
        unsigned num_equal = total == 0 ? num_equal_max_ :
            static_cast<unsigned>(
                    static_cast<double>(num_equal_max_) *
                    static_cast<double>(display_iter) /
                    static_cast<double>(total));
        num_equal = num_equal <= num_equal_max_ ? num_equal : num_equal_max_;
        unsigned percent = total == 0 ? percent_max_ :
            static_cast<unsigned>(
                    static_cast<double>(percent_max_) *
                    static_cast<double>(display_iter) /
                    static_cast<double>(total));
        percent = percent <= percent_max_ ? percent : percent_max_;

        if (timer_ptr->print_first_) {
            timer_ptr->print_first_ = false;
            timer_ptr->num_equal_ = num_equal + 1;
            timer_ptr->percent_ = percent + 1;
            timer_ptr->elapsed_second_ = elapsed_second + 1;
            timer_ptr->last_iter_ = iter + 1;
        }

        if (timer_ptr->num_equal_ != num_equal) {
            timer_ptr->num_equal_ = num_equal;
            unsigned num_space = num_equal_max_ - num_equal;
            unsigned num_dash = 0;
            while (num_space > num_dash) {
                if (num_dash == num_dash_max_)
                    break;
                --num_space;
                ++num_dash;
            }

            char *cstr = timer_ptr->display_progress_;
            std::size_t offset = 0;
            cstr[offset++] = ' ';
            cstr[offset++] = '[';
            for (unsigned i = 0; i != num_equal; ++i)
                cstr[offset++] = '=';
            for (unsigned i = 0; i != num_dash; ++i)
                cstr[offset++] = '-';
            for (unsigned i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (timer_ptr->percent_ != percent) {
            timer_ptr->percent_ = percent;
            const unsigned num_space = 3 - uint_digit(percent);

            char *cstr = timer_ptr->display_percent_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (unsigned i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(percent, cstr, offset);
            cstr[offset++] = '%';
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (timer_ptr->elapsed_second_ != elapsed_second) {
            timer_ptr->elapsed_second_ = elapsed_second;
            uint64_t display_second = elapsed_second % 60;
            uint64_t display_minute = (elapsed_second / 60) % 60;
            uint64_t display_hour   = elapsed_second / 3600;

            char *cstr = timer_ptr->display_time_;
            std::size_t offset = 0;
            cstr[offset++] = '[';
            if (display_hour > 0) {
                uint_to_char(display_hour, cstr, offset);
                cstr[offset++] = ':';
            }
            cstr[offset++] = '0' + static_cast<char>(display_minute / 10);
            cstr[offset++] = '0' + static_cast<char>(display_minute % 10);
            cstr[offset++] = ':';
            cstr[offset++] = '0' + static_cast<char>(display_second / 10);
            cstr[offset++] = '0' + static_cast<char>(display_second % 10);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        if (timer_ptr->last_iter_ != iter) {
            timer_ptr->last_iter_ = iter;
            unsigned dtotal = uint_digit(total);
            unsigned diter = uint_digit(iter);
            unsigned num_space = dtotal > diter ? dtotal - diter : 0;
            char *cstr = timer_ptr->display_iter_;

            std::size_t offset = 0;
            cstr[offset++] = '[';
            for (unsigned i = 0; i < num_space; ++i)
                cstr[offset++] = ' ';
            uint_to_char(iter, cstr, offset);
            cstr[offset++] = '/';
            uint_to_char(total, cstr, offset);
            cstr[offset++] = ']';
            cstr[offset++] = '\0';
        }

        timer_ptr->os_ << timer_ptr->display_progress_;
        timer_ptr->os_ << timer_ptr->display_percent_;
        timer_ptr->os_ << timer_ptr->display_time_;
        timer_ptr->os_ << timer_ptr->display_iter_;
    }

    static void increment_ (void *context)
    {
        DispatchProgress *timer_ptr = static_cast<DispatchProgress *>(context);
        ++timer_ptr->iter_;
    }

    static void increment_step_ (void *context)
    {
        step_context *inc = static_cast<step_context *>(context);
        inc->timer->iter_ += inc->step;
        delete inc;
    }

    static void print_start_ (void *context)
    {
        print_progress(context);
        static_cast<DispatchProgress *>(context)->os_ << '\r';
    }

    static void print_stop_ (void *context)
    {
        print_progress(context);
        static_cast<DispatchProgress *>(context)->os_ << '\n';
    }
}; // class DispatchProgress

} // namespace vsmc

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif // VSMC_UTILITY_DISPATCH_HPP
