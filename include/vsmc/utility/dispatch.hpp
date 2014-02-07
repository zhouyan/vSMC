#ifndef VSMC_UTILITY_DISPATCH_HPP
#define VSMC_UTILITY_DISPATCH_HPP

#include <vsmc/internal/common.hpp>
#include <dispatch/dispatch.h>

namespace vsmc {

/// \brief Types of DispatchQueue
/// \ingroup Dispatch
enum DispatchQueueType {
    Main,    ///< The queue obtained through `dispatch_get_main_queue`
    Global,  ///< The queue obtained through `dispatch_get_gloal_queue`
    Private  ///< The queue created by `dispatch_queue_create`
};

template <DispatchQueueType> class DispatchQueue;

/// \brief Base class of all Dispatch objects
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
        object_ = other.object_;
        dispatch_retain(object_);

        return *this;
    }

    ~DispatchObject () {dispatch_release(object_);}

    /// \brief Return the underlying Dispatch object
    const DispatchType get () const {return object_;}

    void *get_context () const {return dispatch_get_context(object_);}

    void set_context (void *ctx) {dispatch_set_context(object_, ctx);}

    void set_finalizer_f (dispatch_function_t finalizer)
    {dispatch_set_finalizer_t(object_, finalizer);}

    private :

    DispatchType object_;
}; // class DispatchObject

/// \brief Base class of DispatchQueue
/// \ingroup Dispatch
class DispatchQueueBase : public DispatchObject<dispatch_queue_t>
{
    public :

    const char *get_label () const
    {return dispatch_queue_get_label(this->get());}

    void *get_specific (const void *key) const
    {return dispatch_queue_get_specific(this->get(), key);}

    void set_specific (const void *key, void *context,
            dispatch_function_t destructor)
    {dispatch_queue_set_specific(this->get(), key, context, destructor);}

    /// \brief Set this queue as the target queue for the object
    ///
    /// \details
    /// Note that set this queue as the target of an dispatch object will
    /// retain the queue.
    template <typename DispatchType>
    void set_as_target (const DispatchObject<DispatchType> &object) const
    {dispatch_set_target_queue(object.get(), this->get());}

    void resume () {dispatch_resume(this->get());}

    void suspend () {dispatch_suspend(this->get());}

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

    void barrier_async_f (void *context, dispatch_function_t work) const
    {dispatch_barrier_async_f(this->get(), context, work);}

    void barrier_sync_f (void *context, dispatch_function_t work) const
    {dispatch_barrier_sync_f(this->get(), context, work);}

#ifdef __BLOCKS__
    void after (dispatch_time_t when, dispatch_block_t block) const
    {dispatch_after(when, this->get(), block);}

    void apply (std::size_t iterations, void (^block) (std::size_t)) const
    {dispatch_apply(iterations, this->get(), block);}

    void async (dispatch_block_t block) const
    {dispatch_async(this->get(), block);}

    void sync (dispatch_block_t block) const
    {dispatch_sync(this->get(), block);}

    void barrier_async (dispatch_block_t block) const
    {dispatch_barrier_async(this->get(), block);}

    void barrier_sync (dispatch_block_t block) const
    {dispatch_barrier_sync(this->get(), block);}

    void read (dispatch_fd_t fd, std::size_t length,
            void (^handler) (dispatch_data_t, int)) const
    {dispatch_read(fd, length, this->get(), handler);}

    void write (dispatch_fd_t fd, dispatch_data_t data,
            void (^handler) (dispatch_data_t, int)) const
    {dispatch_write(fd, data, this->get(), handler);}
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
class DispatchQueue<Main> : public DispatchQueueBase
{
    public :

    DispatchQueue () : DispatchQueueBase(dispatch_get_main_queue()) {}
}; // class DispatchQueue

/// \brief The global (concurrent) dispatch queue (`dispatch_get_gloal_queue`)
/// \ingroup Dispatch
template <>
class DispatchQueue<Global> : public DispatchQueueBase
{
    public :

    DispatchQueue (dispatch_queue_priority_t priority =
            DISPATCH_QUEUE_PRIORITY_DEFAULT, unsigned long flags = 0) :
        DispatchQueueBase(dispatch_get_global_queue(priority, flags)) {}
}; // class DispatchQueue

/// \brief A private dispatch queue (`dispatch_queue_create`)
/// \ingroup Dispatch
template <>
class DispatchQueue<Private> : public DispatchQueueBase
{
    public :

    DispatchQueue (const char *name, dispatch_queue_attr_t attr = NULL) :
        DispatchQueueBase(dispatch_queue_create(name, attr)) {}

    ~DispatchQueue () {dispatch_release(this->get());}
}; // class DispatchQueue

/// \brief A Dispatch group
/// \ingroup Dispatch
class DispatchGroup : public DispatchObject<dispatch_group_t>
{
    public :

    DispatchGroup () :
        DispatchObject<dispatch_group_t>(dispatch_group_create()) {}

    ~DispatchGroup () {dispatch_release(this->get());}

    void enter () {dispatch_group_enter(this->get());}

    void leave () {dispatch_group_leave(this->get());}

    long wait (dispatch_time_t timeout)
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

} // namespace vsmc

#endif // VSMC_UTILITY_DISPATCH_HPP
