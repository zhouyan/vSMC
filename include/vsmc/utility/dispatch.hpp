#ifndef VSMC_UTILITY_DISPATCH_HPP
#define VSMC_UTILITY_DISPATCH_HPP

#include <vsmc/internal/common.hpp>
#include <vsmc/utility/stop_watch.hpp>
#include <dispatch/dispatch.h>

namespace vsmc {

/// \brief Types of DispatchQueue
/// \ingroup Dispatch
enum DispatchQueueType {
    DispatchMain,    ///< The queue obtained through `dispatch_get_main_queue`
    DispatchGlobal,  ///< The queue obtained through `dispatch_get_gloal_queue`
    DispatchPrivate  ///< The queue created by `dispatch_queue_create`
};

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
};

template <DispatchQueueType> class DispatchQueue;
template <DispatchSourceType> class DispatchSource;

/// \brief Wrap a callable object into a `dispatch_function_t` type pointer
/// \ingroup Dispatch
template <typename T>
class DispatchFunction
{
    public :

    typedef dispatch_function_t function_type;

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

    void *context () const {return (void *) &work_;}

    function_type function () const {return function_;}

    private :

    T work_;

    static void function_ (void *work) {(*static_cast<T *>(work))();}
}; // class DispatchFunction

/// \brief Wrap a callable object into a `void (*) (void *, std::size_t)` type
/// pointer
/// \ingroup Dispatch
template <typename T>
class DispatchFunctionApply
{
    public :

    typedef void (*function_type) (void *, std::size_t);

    DispatchFunctionApply (const T &work) : work_(work) {}

    DispatchFunctionApply (const DispatchFunctionApply<T> &other) :
        work_(other.work_) {}

    DispatchFunctionApply<T> &operator= (const DispatchFunctionApply<T> &other)
    {
        if (this != &other)
            work_ = other.work_;

        return *this;
    }

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
    DispatchFunctionApply (T &&work) VSMC_NOEXCEPT :
        work_(cxx11::move(work)) {}

    DispatchFunctionApply (DispatchFunctionApply<T> &&other) VSMC_NOEXCEPT :
        work_(cxx11::move(other.work_)) {}

    DispatchFunctionApply<T> &operator= (
            DispatchFunctionApply<T> &&other) VSMC_NOEXCEPT
    {
        if (this != &other)
            work_ = cxx11::move(other.work_);

        return *this;
    }
#endif

    void *context () const {return (void *) &work_;}

    function_type function () const {return function_;}

    private :

    T work_;

    static void function_ (void *work, std::size_t i)
    {(*static_cast<T *>(work))(i);}
}; // class DispatchFunctionApply

/// \brief Make a DispatchFunction object from an arbitrary callable object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(void)`
template <typename T>
DispatchFunction<T> dispatch_make_function (T &work)
{return DispatchFunction<T>(work);}

/// \brief Make a DispatchFunctionApply object from an arbitrary callable
/// object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(std::size_t)`
template <typename T>
DispatchFunctionApply<T> dispatch_make_function_apply (T &work)
{return DispatchFunctionApply<T>(work);}

#if VSMC_HAS_CXX11_RVALUE_REFERENCES
/// \brief Make a DispatchFunction object from an arbitrary callable object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(void)`
template <typename T>
DispatchFunction<T> dispatch_make_function (T &&work) VSMC_NOEXCEPT
{return DispatchFunction<T>(cxx11::move(work));}

/// \brief Make a DispatchFunctionApply object from an arbitrary callable
/// object
/// \ingroup Dispatch
///
/// \param work A callable object with signature `void f(std::size_t)`
template <typename T>
DispatchFunctionApply<T> dispatch_make_function_apply (T &&work) VSMC_NOEXCEPT
{return DispatchFunctionApply<T>(cxx11::move(work));}
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
    explicit DispatchObject (DispatchType object) : object_(object)
    {dispatch_retain(object);}

    DispatchObject (const DispatchObject &other) : object_(other.object_)
    {dispatch_retain(object_);}

    DispatchObject &operator= (const DispatchObject &other)
    {
        if (this != &other) {
            object_ = other.object_;
            dispatch_retain(object_);
        }

        return *this;
    }

    ~DispatchObject () {dispatch_release(object_);}

    /// \brief Return the underlying Dispatch object
    DispatchType get () const {return object_;}

    void *get_context () const
    {return dispatch_get_context(object_);}

    void set_context (void *context) const
    {dispatch_set_context(object_, context);}

    void set_finalizer_f (dispatch_function_t finalizer) const
    {dispatch_set_finalizer_t(object_, finalizer);}

    protected :

    void set (DispatchType object) {object_ = object;}

    private :

    DispatchType object_;
}; // class DispatchObject

/// \brief Base class of DispatchQueue
/// \ingroup Dispatch
class DispatchQueueBase : public DispatchObject<dispatch_queue_t>
{
    public :

    void resume () const {dispatch_resume(this->get());}

    void suspend () const {dispatch_suspend(this->get());}

    const char *get_label () const
    {return dispatch_queue_get_label(this->get());}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void *get_specific (const void *key) const
    {return dispatch_queue_get_specific(this->get(), key);}

    void set_specific (const void *key, void *context,
            dispatch_function_t destructor) const
    {dispatch_queue_set_specific(this->get(), key, context, destructor);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

    template <typename DispatchType>
    void set_target_queue (const DispatchObject<DispatchType> &object) const
    {dispatch_set_target_queue(object.get(), this->get());}

    void set_target_queue (dispatch_object_t object) const
    {dispatch_set_target_queue(object, this->get());}

    void after_f (dispatch_time_t when, void *context,
            dispatch_function_t f) const
    {dispatch_after_f(when, this->get(), context, f);}

    void apply_f (std::size_t iterations, void *context,
            void (*work) (void *, std::size_t)) const
    {dispatch_apply_f(iterations, this->get(), context, work);}

    void async_f (void *context, dispatch_function_t work) const
    {dispatch_async_f(this->get(), context, work);}

    void sync_f (void *context, dispatch_function_t work) const
    {dispatch_sync_f(this->get(), context, work);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void barrier_async_f (void *context, dispatch_function_t work) const
    {dispatch_barrier_async_f(this->get(), context, work);}

    void barrier_sync_f (void *context, dispatch_function_t work) const
    {dispatch_barrier_sync_f(this->get(), context, work);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

#ifdef __BLOCKS__
    void after (dispatch_time_t when, dispatch_block_t block) const
    {dispatch_after(when, this->get(), block);}

    void apply (std::size_t iterations, void (^block) (std::size_t)) const
    {dispatch_apply(iterations, this->get(), block);}

    void async (dispatch_block_t block) const
    {dispatch_async(this->get(), block);}

    void sync (dispatch_block_t block) const
    {dispatch_sync(this->get(), block);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void barrier_async (dispatch_block_t block) const
    {dispatch_barrier_async(this->get(), block);}

    void barrier_sync (dispatch_block_t block) const
    {dispatch_barrier_sync(this->get(), block);}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
#endif // __BLOCKS__

    protected :

    DispatchQueueBase (dispatch_queue_t queue) :
        DispatchObject<dispatch_queue_t>(queue) {}

    DispatchQueueBase (const DispatchQueueBase &other) :
        DispatchObject<dispatch_queue_t>(other) {}

    DispatchQueueBase &operator= (const DispatchQueueBase &other)
    {DispatchObject<dispatch_queue_t>::operator=(other); return *this;}
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
        DispatchQueueBase(dispatch_get_global_queue(priority, flags)) {}
#else // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    DispatchQueue (long priority = 0, unsigned long flags = 0) :
        DispatchQueueBase(dispatch_get_global_queue(priority, flags)) {}
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
}; // class DispatchQueue

/// \brief A private dispatch queue (`dispatch_queue_create`)
/// \ingroup Dispatch
template <>
class DispatchQueue<DispatchPrivate> : public DispatchQueueBase
{
    public :

    DispatchQueue (const char *name, dispatch_queue_attr_t attr = NULL) :
        DispatchQueueBase(dispatch_queue_create(name, attr)) {}

    DispatchQueue (const DispatchQueue<DispatchPrivate> &other) :
        DispatchQueueBase(other) {dispatch_retain(this->get());}

    DispatchQueue<DispatchPrivate> &operator= (
            const DispatchQueue<DispatchPrivate> &other)
    {
        if (this != &other) {
            DispatchQueueBase::operator=(other); return *this;
            dispatch_retain(this->get());
        }

        return *this;
    }

    ~DispatchQueue () {dispatch_release(this->get());}
}; // class DispatchQueue

/// \brief A Dispatch group
/// \ingroup Dispatch
class DispatchGroup : public DispatchObject<dispatch_group_t>
{
    public :

    DispatchGroup () :
        DispatchObject<dispatch_group_t>(dispatch_group_create()) {}

    DispatchGroup (const DispatchGroup &other) :
        DispatchObject<dispatch_group_t>(other) {dispatch_retain(this->get());}

    DispatchGroup &operator= (const DispatchGroup &other)
    {
        if (this != &other) {
            DispatchObject<dispatch_group_t>::operator=(other);
            dispatch_retain(this->get());
        }

        return *this;
    }

    ~DispatchGroup () {dispatch_release(this->get());}

    void enter () const {dispatch_group_enter(this->get());}

    void leave () const {dispatch_group_leave(this->get());}

    long wait (dispatch_time_t timeout) const
    {return dispatch_group_wait(this->get(), timeout);}

    template <DispatchQueueType Type>
    void async_f (const DispatchQueue<Type> &queue, void *context,
            dispatch_function_t work) const
    {dispatch_group_async_f(this->get(), queue.get(), context, work);}

    void async_f (dispatch_queue_t queue, void *context,
            dispatch_function_t work) const
    {dispatch_group_async_f(this->get(), queue, context, work);}

    template <DispatchQueueType Type>
    void notify_f (const DispatchQueue<Type> &queue, void *context,
            dispatch_function_t work) const
    {dispatch_group_notify_f(this->get(), queue.get(), context, work);}

    void notify_f (dispatch_queue_t queue, void *context,
            dispatch_function_t work) const
    {dispatch_group_notify_f(this->get(), queue, context, work);}

#ifdef __BLOCKS__
    template <DispatchQueueType Type>
    void async (const DispatchQueue<Type> &queue,
            dispatch_block_t block) const
    {dispatch_group_async(this->get(), queue.get(), block);}

    void async (dispatch_queue_t queue,
            dispatch_block_t block) const
    {dispatch_group_async(this->get(), queue, block);}

    template <DispatchQueueType Type>
    void notify (const DispatchQueue<Type> &queue,
            dispatch_block_t block) const
    {dispatch_group_notify(this->get(), queue.get(), block);}

    void notify (dispatch_queue_t queue,
            dispatch_block_t block) const
    {dispatch_group_notify(this->get(), queue, block);}
#endif // __BLOCKS__
}; // class DispatchGroup

/// \brief Base class of DispatchSource
/// \ingroup Dispatch
template <DispatchSourceType Type>
class DispatchSourceBase : public DispatchObject<dispatch_source_t>
{
    public :

    void resume () const {dispatch_resume(this->get());}

    void suspend () const {dispatch_suspend(this->get());}

    void cancel () const {dispatch_source_cancel(this->get());}

    long testcancel () const {return dispatch_source_testcancel(this->get());}

    unsigned long get_data () const
    {return dispatch_source_get_data(this->get());}

    uintptr_t get_handle () const
    {return dispatch_source_get_handle(this->get());}

    unsigned long get_mask () const
    {return dispatch_source_get_mask(this->get());}

    void set_cancel_handler_f (dispatch_function_t cancel_handler) const
    {dispatch_source_set_cancel_handler_f(this->get(), cancel_handler);}

    void set_event_handler_f (dispatch_function_t event_handler) const
    {dispatch_source_set_event_handler_f(this->get(), event_handler);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void set_registration_handler_f (dispatch_function_t
            registration_handler) const
    {
        dispatch_source_set_registration_handler_f(
                this->get(), registration_handler);
    }
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)

#ifdef __BLOCKS__
    void set_cancel_handler (dispatch_block_t cancel_handler) const
    {dispatch_source_set_cancel_handler(this->get(), cancel_handler);}

    void set_event_handler (dispatch_block_t event_handler) const
    {dispatch_source_set_event_handler(this->get(), event_handler);}

#if VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
    void set_registration_handler (dispatch_block_t
            registration_handler) const
    {
        dispatch_source_set_registration_handler(
                this->get(), registration_handler);
    }
#endif // VSMC_MAC_VERSION_MIN_REQUIRED(VSMC_MAC_10_7)
#endif // __BLOCKS__

    protected :

    DispatchSourceBase (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchObject<dispatch_source_t>(
                dispatch_source_create(source_type(cxx11::integral_constant<
                        DispatchSourceType, Type>()), handle, mask, queue)) {}

    DispatchSourceBase (const DispatchSourceBase &other) :
        DispatchObject<dispatch_source_t>(other)
    {dispatch_retain(this->get());}

    DispatchSourceBase &operator= (const DispatchSourceBase &other)
    {
        if (this != &other) {
            DispatchObject<dispatch_source_t>::operator=(other);
            dispatch_retain(this->get());
        }

        return *this;
    }

    ~DispatchSourceBase () {if (!testcancel()) dispatch_release(this->get());}

    private :

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchDataAdd>)
    {return DISPATCH_SOURCE_TYPE_DATA_ADD;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchDataOr>)
    {return DISPATCH_SOURCE_TYPE_DATA_OR;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchMachRecv>)
    {return DISPATCH_SOURCE_TYPE_MACH_RECV;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchMachSend>)
    {return DISPATCH_SOURCE_TYPE_MACH_SEND;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchProc>)
    {return DISPATCH_SOURCE_TYPE_PROC;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchRead>)
    {return DISPATCH_SOURCE_TYPE_READ;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchSignal>)
    {return DISPATCH_SOURCE_TYPE_SIGNAL;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchTimer>)
    {return DISPATCH_SOURCE_TYPE_TIMER;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchVnode>)
    {return DISPATCH_SOURCE_TYPE_VNODE;}

    static dispatch_source_type_t source_type (
            cxx11::integral_constant<DispatchSourceType, DispatchWrite>)
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
        DispatchSourceBase<Type>(handle, mask, queue.get()) {}

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
        DispatchSourceBase<DispatchDataAdd>(handle, mask, queue.get()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchDataAdd>(handle, mask, queue) {}

    void merge_data (unsigned long value) const
    {dispatch_source_merge_data(this->get(), value);}
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
        DispatchSourceBase<DispatchDataOr>(handle, mask, queue.get()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchDataOr>(handle, mask, queue) {}

    void merge_data (unsigned long value) const
    {dispatch_source_merge_data(this->get(), value);}
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
        DispatchSourceBase<DispatchTimer>(handle, mask, queue.get()) {}

    DispatchSource (uintptr_t handle, unsigned long mask,
            dispatch_queue_t queue) :
        DispatchSourceBase<DispatchTimer>(handle, mask, queue) {}

    void set_timer (dispatch_time_t start,
            uint64_t interval, uint64_t leeway) const
    {dispatch_source_set_timer(this->get(), start, interval, leeway);}
}; // class DispatchSource

/// \brief Display a progress bar while algorithm proceed
/// \ingroup Dispatch
///
/// \details
/// Example:
/// \code
/// class myProgress : public vsmc::DispatchProgress
/// {
///     public :
///
///     myProgress (vsmc::Sampler<T> &sampler, std::size_t IterNum) :
///         vsmc::DispatchProgress(IterNum), sampler_(sampler) {}
///
///     // implement the following pure virtual base class function. You may
///     // want to lock `sampler_` in case `iter_num` is read while it is
///     // being updated
///     uint64_t current () const {return sampler_.iter_num();}
///
///     private :
///
///     const vsmc::Sampler<T> &sampler_;
/// };
///
/// vsmc::Sampler<T> sampler(N);
/// // configure the sampler
/// myProgress progress(sampler, IterNum);
/// progress.start();
/// sampler.initialize();
/// sampler.iterate(IterNum);
/// progress.stop();
/// \endcode
class DispatchProgress
{
    public :

    /// \brief Construct a DispatchProgress with total amount of work
    ///
    /// \param total Total amount of work represented by an integer, for
    /// example file size or SMC algorithm total number of iterations
    DispatchProgress (uint64_t total) :
        total_(total), queue_("DispatchProgress"), timer_(0, 0, queue_) {}

    ~DispatchProgress () {timer_.cancel();}

    /// \brief Start to print the progress
    void start ()
    {
        num_equal_ = 1000000;
        elapsed_second_ = 1000000;

        timer_.set_context((void *) this);
        timer_.set_event_handler_f(print_start);
        timer_.set_timer(DISPATCH_TIME_NOW,
                NSEC_PER_SEC / 10, NSEC_PER_SEC / 10);

        watch_.reset();
        watch_.start();
        timer_.resume();
    }

    /// \brief Stop to print the progress
    void stop ()
    {
        timer_.suspend();
        queue_.sync_f((void *) this, print_stop);
        watch_.stop();
    }

    /// \brief Returnt the current progress
    ///
    /// \return The amount of work already down represented by an integer in
    /// the range zero to total, where total is passed into the constructor
    virtual uint64_t current () const = 0;

    private :

    uint64_t total_;
    DispatchQueue<DispatchPrivate> queue_;
    DispatchSource<DispatchTimer> timer_;
    StopWatch watch_;
    mutable std::size_t num_equal_;
    mutable uint64_t elapsed_second_;
    mutable char display_progress_[128];
    mutable char display_time_[128];

    static void print_progress (void *context)
    {
        const DispatchProgress *timer_ptr =
            static_cast<const DispatchProgress *>(context);

        uint64_t iter = timer_ptr->current();
        uint64_t totl = timer_ptr->total_;
        iter = (iter <= totl) ? iter : totl;
        std::size_t num_equal = static_cast<std::size_t>(60 * iter / totl);

        timer_ptr->watch_.stop();
        timer_ptr->watch_.start();
        StopWatch elapsed = timer_ptr->watch_;
        uint64_t elapsed_second = static_cast<uint64_t>(elapsed.seconds());

        if (timer_ptr->num_equal_ != num_equal) {
            timer_ptr->num_equal_ = num_equal;
            int percent = static_cast<int>(100 * iter / totl);
            std::size_t num_space = 60 - num_equal;
            std::size_t num_dash = 0;
            if (num_space > 0) {
                --num_space;
                num_dash = 1;
            }
            char p1 = static_cast<char>((percent / 100) % 10);
            char p2 = static_cast<char>((percent / 10) % 10);
            char p3 = static_cast<char>(percent % 10);

            char *cstr = timer_ptr->display_progress_;
            std::size_t offset = 0;
            cstr[offset++] = ' ';
            cstr[offset++] = '[';
            for (std::size_t i = 0; i != num_equal; ++i)
                cstr[offset++] = '=';
            for (std::size_t i = 0; i != num_dash; ++i)
                cstr[offset++] = '-';
            for (std::size_t i = 0; i != num_space; ++i)
                cstr[offset++] = ' ';
            cstr[offset++] = ']';
            cstr[offset++] = '[';
            cstr[offset++] = (percent >= 100) ? ('0' + p1) : ' ';
            cstr[offset++] = (percent >=  10) ? ('0' + p2) : ' ';
            cstr[offset++] = (percent >=   1) ? ('0' + p3) : '0';
            cstr[offset++] = '%';
            cstr[offset++] = ']';
            cstr[offset++] = ' ';
            cstr[offset++] = '\0';
        }

        if (timer_ptr->elapsed_second_ != elapsed_second) {
            timer_ptr->elapsed_second_ = elapsed_second;
            uint64_t display_second = elapsed_second % 60;
            uint64_t display_minute = (elapsed_second / 60) % 60;
            uint64_t display_hour   = elapsed_second / 3600;

            char *cstr = timer_ptr->display_time_;
            std::size_t offset = 0;
            char htmp[100];
            std::size_t hnum = 0;
            if (display_hour > 0) {
                while (display_hour) {
                    htmp[hnum++] = '0' + static_cast<char>(display_hour % 10);
                    display_hour /= 10;
                }
                for (std::size_t i = hnum; i != 0; --i)
                    cstr[offset++] = htmp[hnum - 1];
                cstr[offset++] = ':';
            }
            cstr[offset++] = '0' + static_cast<char>(display_minute / 10);
            cstr[offset++] = '0' + static_cast<char>(display_minute % 10);
            cstr[offset++] = ':';
            cstr[offset++] = '0' + static_cast<char>(display_second / 10);
            cstr[offset++] = '0' + static_cast<char>(display_second % 10);
            cstr[offset++] = '\0';
        }

        std::cout << const_cast<const char *>(timer_ptr->display_progress_);
        std::cout << const_cast<const char *>(timer_ptr->display_time_);
    }

    static void print_start (void *context)
    {
        print_progress(context);
        std::cout << '\r' << std::flush;
    }

    static void print_stop (void *context)
    {
        print_progress(context);
        std::cout << std::endl;
    }
}; // class DispatchProgress

/// \brief A DispatchProgress that monitors a vSMC Sampler
/// \ingroup Dispatch
///
/// \details
/// Example:
/// \code
/// vsmc::Sampler<T> sampler(N);
/// // configure the sampler
/// vsmc::DispatchProgressSampler<T> progress(sampler, IterNum);
/// progress.start();
/// sampler.initialize();
/// sampler.iterate(IterNum);
/// progress.stop();
/// \endcode
template <typename T>
class DispatchProgressSampler : public DispatchProgress
{
    public :

    DispatchProgressSampler (const Sampler<T> &sampler, std::size_t iter_num) :
        DispatchProgress(iter_num), sampler_(sampler) {}

    uint64_t current () const
    {return static_cast<uint64_t>(sampler_.iter_num());}

    private :

    const Sampler<T> &sampler_;
};

} // namespace vsmc

#endif // VSMC_UTILITY_DISPATCH_HPP
